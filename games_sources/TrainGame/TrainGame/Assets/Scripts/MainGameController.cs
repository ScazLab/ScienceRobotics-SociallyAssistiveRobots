using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;

namespace TrainGame {

	public class MainGameController : MonoBehaviour {

		// mode of playing the game (controlls whether we send/receive ROS messages or not
		public bool sendReceiveROSMessages ;

		// performance metrics to track
		//public Dictionary<string,float> performanceMetrics = new Dictionary<string,float> ();
        public float performanceMetricsTrainErrors = 0;
        public float performanceMetricsOrderErrors = 0;

		// the different kinds of rocket pieces
		public List<GameObject> frontPieceOptions;
		public List<GameObject> backPieceOptions;
		public List<GameObject> bodyPieceOptions;
        public List<GameObject> cabinPieceOptions;
        public List<GameObject> smokePieceOptions;
        public List<GameObject> wheelPieceOptions;

		// game state variables
		public int currentScene;
		public int explainerMode;
		public bool firstRound = true;
		public int internalGameState = Constants.WAITING_FOR_GAME_START;
		public int levelNumber;
        public int scene;
        public bool tutorial;
        public int saveLevel;
        public int previousLevel;


        // ROS configuration and websocket variables
        private GameConfig gameConfig;
		public RosbridgeWebSocketClient clientSocket = null;

		// robot script 
		private Dictionary<string, List<List<string>>> robotUtterances;
		public bool recentScreenInteraction = false;

        

        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        public static Rect BottomRegion = new Rect(Screen.height / 2, Screen.width / 2 + Screen.width / 8, 580, 64);
        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        void Awake () {
			// ensure that the game manager won't be destroyed when a new scene is loaded
			DontDestroyOnLoad (gameObject);

			// set the currentScene to the start scene
			currentScene = Constants.START_SCENE;

			// get the configuration settings for the rosbridge server
			string configPath = "";

			#if UNITY_EDITOR
			configPath = Application.dataPath + Constants.CONFIG_PATH_OSX + Constants.CONFIG_FILE;
			Logger.Log("trying os x path for config: " + configPath);
			#endif

			#if UNITY_STANDALONE
			configPath = Application.dataPath + Constants.CONFIG_PATH_LINUX + Constants.CONFIG_FILE;
			Logger.Log("trying linux path for config: " + configPath);
			#endif

			// read config file
			if(!Utilities.ParseConfig(configPath, out gameConfig)) {
				Logger.LogWarning("Could not read config file! Will try default "
					+ "values of toucan=true, server IP=192.168.1.254, port=9090.");
			}
			else {
				Logger.Log("Got game config!");
			}

			// read and parse the robot utterances file
			string robotUtterancesPath = "";

			#if UNITY_EDITOR
			robotUtterancesPath = Application.dataPath + Constants.ROBOT_UTTERANCES_PATH_OSX + Constants.ROBOT_UTTERANCES_FILE;
			Logger.Log("trying os x path for robot utterances file: " + robotUtterancesPath);
			#endif

			#if UNITY_STANDALONE
			robotUtterancesPath = Application.dataPath + Constants.ROBOT_UTTERANCES_PATH_LINUX + Constants.ROBOT_UTTERANCES_FILE;
			Logger.Log("trying linux path for robot utterances file: " + robotUtterancesPath);
			#endif

			// read robot utterances file
			if(!Utilities.ParseRobotUtterancesFile(robotUtterancesPath, out robotUtterances)) {
				Logger.LogWarning("Could not read robot utterances file!");
			}
			else {
				Logger.Log("Got robot utterances!");
			}
        }

		void Start () {
			// set up rosbridge websocket client
			// note: does not attempt to reconnect if connection fails!
			if (this.clientSocket == null && sendReceiveROSMessages) {
				// load file
				if (this.gameConfig.server.Equals ("") || this.gameConfig.port.Equals ("")) {
					Logger.LogWarning ("Do not have opal configuration... trying hardcoded IP 192.168.1.254 and port 9090");
					this.clientSocket = new RosbridgeWebSocketClient (
						"192.168.1.254",// server, // can pass hostname or IP address
						"9090"); //port);   
				} else {
					this.clientSocket = new RosbridgeWebSocketClient (
						this.gameConfig.server, // can pass hostname or IP address
						this.gameConfig.port);  
				}

				if (this.clientSocket.SetupSocket ()) {
					this.clientSocket.receivedMsgEvent += 
						new ReceivedMessageEventHandler (HandleClientSocketReceivedMsgEvent);

					// advertise that we will publish log messages
					this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonAdvertiseMsg (
						Constants.LOG_ROSTOPIC, Constants.LOG_ROSMSG_TYPE));

					// advertise that we'll publish game state messages
					this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonAdvertiseMsg (
						Constants.STATE_ROSTOPIC, Constants.STATE_ROSMSG_TYPE));

					// advertise that we'll publish robot command messages
					this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonAdvertiseMsg (
						Constants.ROBOT_CMD_ROSTOPIC, Constants.ROBOT_CMD_ROSMSG_TYPE));

					// subscribe to opal command messages
					this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonSubscribeMsg (
						Constants.CMD_ROSTOPIC, Constants.CMD_ROSMSG_TYPE));
				} else {
					Logger.LogError ("Could not set up websocket!");
				}
                
            }

            // Send ROS message to show that game is ready
            if (sendReceiveROSMessages)
            {
                this.clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishGameStateMsg(
                    Constants.STATE_ROSTOPIC, Constants.GAME_STATE_READY, new Dictionary<string, float>()));
            }

            // if we're not sending/receiving ROS messages, then we'll start the game right away
            else if (!sendReceiveROSMessages) {
				internalGameState = Constants.START_GAME;
			}

            saveLevel = levelNumber;
        }

		void Update ()
        {
            if (internalGameState == Constants.END_GAME)
            {
                SendRobotUtterance("end-game", false, -1, -1, -1, -1);
                sendPerformanceMetrics();
                Logger.Log("GoingToQuit");
                Application.Quit();
            }

            if ((sendReceiveROSMessages) && (internalGameState == Constants.START_GAME))
            {
                this.clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishGameStateMsg(
                    Constants.STATE_ROSTOPIC, Constants.GAME_STATE_START, new Dictionary<string, float>()));
            }
            if ((sendReceiveROSMessages) && (internalGameState == Constants.TUTORIAL))
            {
                this.clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishGameStateMsg(Constants.STATE_ROSTOPIC, Constants.TUTORIAL, new Dictionary<string, float>()));
                internalGameState = Constants.STARTED_GAME;
            }
            if (internalGameState == Constants.START_GAME) {
				
				// set the explainerMode to building
				explainerMode = Constants.EXPLAINER_BUILDING;


				if (firstRound) {
					// robot utterance that introduces the game
					SendRobotUtterance ("game-start", true, -1,-1,-1,-1);

					// no longer the first round (after this round)
					firstRound = false;
				}
                //SceneManager.LoadScene("Builder_L1");
                SceneManager.LoadScene("Start_menu");

               
                internalGameState = Constants.STARTED_GAME;
				
			}
        }

		void OnDestroy () {
			// close websocket
			if(this.clientSocket != null && sendReceiveROSMessages) {
				this.clientSocket.CloseSocket();

				// unsubscribe from received message events
				this.clientSocket.receivedMsgEvent -= HandleClientSocketReceivedMsgEvent;
			}
		}

        public void sendPerformanceMetrics()
        {
            if (sendReceiveROSMessages)
            {
                performanceMetricsOrderErrors = 1- performanceMetricsOrderErrors;
                performanceMetricsTrainErrors = 1 - performanceMetricsTrainErrors;
                Dictionary<string, float> PerformanceMetrics = new Dictionary<string, float>();
                PerformanceMetrics.Add("train-piece-errors", performanceMetricsTrainErrors);
                PerformanceMetrics.Add("train-order-errors", performanceMetricsOrderErrors);
                clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishGameStateMsg(Constants.STATE_ROSTOPIC, Constants.GAME_STATE_END, PerformanceMetrics));
            }
        }


		void HandleClientSocketReceivedMsgEvent (object sender, int game, int command, int level) {

			Logger.Log("MSG received from remote: " + game.ToString() + ", " + command.ToString() + ", " + level.ToString());
			if (this.clientSocket != null)
			{
				this.clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishStringMsg(
					Constants.LOG_ROSTOPIC, "got message"));
			}

			if (game != Constants.SAR_GAME_IDENTITY) {
				Debug.Log ("Received a game command message for another game - command unhandled");
				return;
			}

			if (command == Constants.GAME_COMMAND_START) {

				if (level > 4 || level < 0) {
					Debug.Log ("Game level must be provided on game start and must be within the range 1-4");
					return;
				} else {
					if (!(currentScene == Constants.START_SCENE)) {
						Debug.Log ("Game must be in the start scene to start the game");
						return;
					} else {
						// set the level
						levelNumber = level;
						// and indicate that we can start the game
						internalGameState = Constants.START_GAME;
					}
				}
			} else if (command == Constants.GAME_COMMAND_END) {
				internalGameState = Constants.END_GAME;
			}
		}

		public void SendRobotUtterance (string utteranceCategory, bool interrupt, int pieceType1, int pieceType2, int pieceType3, int pieceType4) {

			if (robotUtterances.ContainsKey(utteranceCategory)) {

				int randIndex = Random.Range (0, robotUtterances [utteranceCategory].Count - 1);
				List<string> chosenUtterance = robotUtterances [utteranceCategory] [randIndex];

				foreach (string chosenUtteranceSection in chosenUtterance) {
					// first prepare the utterance for sending
					string preparedUtterance = Utilities.PrepareUtteranceForSending (chosenUtteranceSection, true);
                    preparedUtterance = Utilities.replaceTypeUtterances(preparedUtterance, pieceType1, pieceType2, pieceType3, pieceType4);
                    Logger.Log(preparedUtterance);
                    //Logger.Log(preparedUtterance);

                    // string topic, int command, string id, bool interrupt, string robotActionAndSpeech
                    if (sendReceiveROSMessages) {
						this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonPublishRobotCommandMsg (
							Constants.ROBOT_CMD_ROSTOPIC,
							Constants.ROBOT_COMMAND_DO,
							"",
							interrupt,
							preparedUtterance
						));
					}
				}
			} else {
				Debug.Log ("Utterance category not found in robotUtterances");
				return;
			}
            

		}
	


		public void StartNextGame () {

            if (internalGameState == Constants.END_GAME)
            {
                SendRobotUtterance("end-game", false, -1, -1, -1, -1);
                sendPerformanceMetrics();
                Logger.Log("GoingToQuit");
                Application.Quit();
            }
            internalGameState = Constants.START_GAME;

        }


    }
}
