using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;

namespace RocketBarrierGame {


	public class ConstructedRocket {

		public List<GameObject> bodyPieces;
		public List<GameObject> boosterPieces;
		public List<GameObject> conePieces;
		public List<GameObject> finPieces;

		public ConstructedRocket(int numBodyPieces, int numBoosterPieces, int numConePieces, int numFinPieces) {
			// initialize the bodyPieces list
			bodyPieces = new List<GameObject> ();
			for (int i = 0; i < numBodyPieces; i++) {
				bodyPieces.Add (null);
			}
			// initialize the boosterPieces list
			boosterPieces = new List<GameObject> ();
			for (int i = 0; i < numBoosterPieces; i++) {
				boosterPieces.Add (null);
			}
			// initialize the conePieces list
			conePieces = new List<GameObject> ();
			for (int i = 0; i < numConePieces; i++) {
				conePieces.Add (null);
			}
			// initialize the bodyPieces list
			finPieces = new List<GameObject> ();
			for (int i = 0; i < numFinPieces; i++) {
				finPieces.Add (null);
			}
		}
	}

	public class MainGameController : MonoBehaviour {

		// mode of playing the game (controlls whether we send/receive ROS messages or not
		public bool sendReceiveROSMessages;

		// whether the child is the explainer/builder
		public bool childExplainer;

		// performance metrics to track
		private Dictionary<int,List<float>> performanceMetrics = new Dictionary<int,List<float>> ();

		// dashed outline pieces
		public GameObject bodyDashedOutline;
		public GameObject boosterDashedOutline;
		public GameObject coneDashedOutline;
		public GameObject leftFinDashedOutline;
		public GameObject rightFinDashedOutline;

		// selected outline pieces
		public GameObject bodySelectedOutline;
		public GameObject boosterSelectedOutline;
		public GameObject coneSelectedOutline;
		public GameObject leftFinSelectedOutline;
		public GameObject rightFinSelectedOutline;

		// the different kinds of rocket pieces
		public List<GameObject> bodyRocketPieceOptions;
		public List<GameObject> boosterRocketPieceOptions;
		public List<GameObject> coneRocketPieceOptions;
		public List<GameObject> finRocketPieceOptions;

		// panels where the outline and rocket pieces will be placed
		private Transform DashedOutlinesPanel;
		private Transform SelectedOutlinesPanel;
		private Transform RocketPiecesLeftPanel;
		private Transform RocketPiecesRightPanel;

		// outline slots
		private List<GameObject> dashedBodyOutlineSlots = new List<GameObject> ();
		private List<GameObject> dashedBoosterOutlineSlots = new List<GameObject> ();
		private List<GameObject> dashedConeOutlineSlot = new List<GameObject> ();
		private List<GameObject> dashedFinOutlineSlots = new List<GameObject> ();
		private List<GameObject> selectedBodyOutlineSlots = new List<GameObject> ();
		private List<GameObject> selectedBoosterOutlineSlots = new List<GameObject> ();
		private List<GameObject> selectedConeOutlineSlot = new List<GameObject> ();
		private List<GameObject> selectedFinOutlineSlots = new List<GameObject> ();

		// panel body pieces
		private List<GameObject> bodyPanelRocketPieces = new List<GameObject> ();
		private List<GameObject> boosterPanelRocketPieces = new List<GameObject> ();
		private List<GameObject> conePanelRocketPieces = new List<GameObject> ();
		private List<GameObject> finPanelRocketPieces = new List<GameObject> ();

		// keep track of the 2 rockets and all of their pieces (and # of pieces)
		public ConstructedRocket builderRocket;
		public ConstructedRocket explainerRocket;

		private int totalNumBuilderRocketPieces = 0;
		private int totalNumExplainerRocketPieces = 0;

		// animator and related variables
		private Animator panelsAnimator;
		private bool firstStateChangeOccured = false;

		// indicates which graceful exit procedure we're executing once we've received the game end command
		private int gracefulExitProcedure;

		// game state variables
		public int currentScene;
		public int explainerMode;
		public bool firstRound = true;
		private int internalGameState = Constants.WAITING_FOR_GAME_START;
		public int levelNumber;
		private bool sentPerformanceMetricsAtEndOfGame = false;
		private bool tutorialCompleted = false;
		public int tutorialMode = Constants.TUTORIAL_OFF;

		// type of piece selected
		private int currentPieceTypeSelected;
		private int lastPieceTypeSelected;

		// ROS configuration and websocket variables
		private GameConfig gameConfig;
		private RosbridgeWebSocketClient clientSocket = null;

		// robot script 
		private Dictionary<string, List<List<string>>> robotUtterances;

		// some timing variables that control robot utterances
		private int frequencyOfEncouragementSecs = 45;
		private int frequencyOfPromptingPlayersFromInactivity = 30;
		private int maxAcceptableRoundTimeSecs = 600;
		private bool recentScreenInteraction = false;
		private int secondsElapsed = 0;
		private int secondsOfInactivity = 0;
		private float timeElapsed = 0f;
		private int secondsAfterGracefulExit = 0;

		void Awake () {
			// ensure that the game manager won't be destroyed when a new scene is loaded
			DontDestroyOnLoad (gameObject);

			// ensure full screen
			Screen.fullScreen = true;

			// set the currentScene to the start scene
			currentScene = Constants.START_SCENE;

			// initialize pieceTypeSelected
			currentPieceTypeSelected = Constants.NONE_SELECTED;
			lastPieceTypeSelected = Constants.NONE_SELECTED;

			// initialize the results dictionary
			performanceMetrics.Add (Constants.CHILD_EXPLAINER_LOCATION_ACCURACY, new List<float> ());
			performanceMetrics.Add (Constants.CHILD_EXPLAINER_CHOICE_ACCURACY, new List<float> ());
			performanceMetrics.Add (Constants.CHILD_BUILDER_LOCATION_ACCURACY, new List<float> ());
			performanceMetrics.Add (Constants.CHILD_BUILDER_CHOICE_ACCURACY, new List<float> ());

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
			//rocket piece counter

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
			// if we're not sending/receiving ROS messages, then we'll start the game right away
			else if (!sendReceiveROSMessages) {
				internalGameState = Constants.START_GAME;
			}

			// send the 'ready' message
			if (sendReceiveROSMessages) {
				this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonPublishGameStateMsg (
					Constants.STATE_ROSTOPIC, Constants.GAME_STATE_READY, new Dictionary<string, float> ()));
			}


		}


		void Update () {

			// once we receive the game start command from the ROS controller, we load the options of
			// play or tutorial
			if (internalGameState == Constants.RECEIVED_GAME_START) {

				SceneManager.LoadScene ("ChoosePlayOrTutorial");
				internalGameState = Constants.WAITING_FOR_TUTORIAL_OR_PLAY_CHOICE;

			} 

			// handles the beginning of the tutorial, beginning of the first game, and beginning of 
			// every following game
			else if (internalGameState == Constants.START_GAME) {

				// send a ROS message to indicate the beginning of the tutorial/game

				if (sendReceiveROSMessages) {
					if (tutorialMode == Constants.TUTORIAL_ON) {
						// tutorial start message
						this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonPublishGameStateMsg (
							Constants.STATE_ROSTOPIC, Constants.GAME_STATE_TUTORIAL, new Dictionary<string, float> ()));
					} else if (tutorialMode == Constants.TUTORIAL_OFF) {
						// game start message 
						this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonPublishGameStateMsg (
							Constants.STATE_ROSTOPIC, Constants.GAME_STATE_START, new Dictionary<string, float> ()));
					}
				}

				// set the explainerMode to building
				explainerMode = Constants.EXPLAINER_BUILDING;

				// set the psudo level number for use in this function - 
				// the levelNumber may be 4 but the psudoLevelNumber may be 1, as in during the tutorial
				int psudoLevelNumber;
				if (tutorialMode == Constants.TUTORIAL_ON) {
					psudoLevelNumber = 1;
				} else {
					psudoLevelNumber = levelNumber;
				}

				// initialize the explainer and builder rockets
				explainerRocket = new ConstructedRocket (
					Constants.LEVEL_INFO [psudoLevelNumber].numBodyOutlines, 
					Constants.LEVEL_INFO [psudoLevelNumber].numBoosterOutlines, 
					1, 2);

				builderRocket = new ConstructedRocket (
					Constants.LEVEL_INFO [psudoLevelNumber].numBodyOutlines, 
					Constants.LEVEL_INFO [psudoLevelNumber].numBoosterOutlines, 
					1, 2);

				// re-initialize the timing variables
				secondsElapsed = 0;
				secondsOfInactivity = 0;
				timeElapsed = 0f;

				// when we start the game, say the appropriate introductory phrases
				if (tutorialMode == Constants.TUTORIAL_OFF) {
					if (firstRound) {
						
						// say robot utterance that introduces the game if the tutorial has
						// not been covered to remind participants of the rules
						if (!tutorialCompleted) {
							SendRobotUtterance ("game-start", true);
						}

						// no longer the first round (after this round)
						firstRound = false;
					}

					// let the players know who is in which role - round start
					SendRobotUtterance ("explainer-start-building", false);
				} else if (tutorialMode == Constants.TUTORIAL_ON) {
					// do the intro to the tutorial
					SendRobotUtterance ("tutorial-01-intro", false);
				}

				// load the explainer scene and indicate that we've started the game
				SceneManager.LoadScene ("Explainer");
				internalGameState = Constants.STARTED_GAME;

			} 

			//Logic that controls the delegation of how the game will end
			else if ((internalGameState == Constants.END_GAME) || 
				(Input.GetKey (KeyCode.Q) && !sendReceiveROSMessages && internalGameState != Constants.GRACEFUL_EXIT)) {
				
				Debug.Log ("start graceful exit procedures");
				
				internalGameState = Constants.GRACEFUL_EXIT;

				// determine which graceful exit to execute
				if (currentScene == Constants.EXPLAINER_SCENE && explainerMode == Constants.EXPLAINER_BUILDING) {

					gracefulExitProcedure = Constants.GRACEFUL_EXIT_EXPLAINER_FINISH;

				} else if (currentScene == Constants.BUILDER_SCENE ||
					(explainerMode == Constants.EXPLAINER_EXPLAINING && currentScene == Constants.EXPLAINER_SCENE)) {

					UpdateRocketPieceCounts ();

					//halfway values
					//- level 1- 4 pieces
					//- level 2- 6 pieces
					//- level 3- 8 pieces
					//- level 4- 10 pieces

					if (levelNumber == 1 || tutorialMode == Constants.TUTORIAL_ON) {

						if (totalNumBuilderRocketPieces > 4) {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_COMPLETELY;
						} else {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_HALFWAY;
						}

					} else if (levelNumber == 2) {

						if (totalNumBuilderRocketPieces > 6) {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_COMPLETELY;
						} else {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_HALFWAY;
						}

					} else if (levelNumber == 3) {

						if (totalNumBuilderRocketPieces > 8) {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_COMPLETELY;
						} else {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_HALFWAY;
						}

					} else {

						if (totalNumBuilderRocketPieces > 10) {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_COMPLETELY;
						} else {
							gracefulExitProcedure = Constants.GRACEFUL_EXIT_BUILDER_FINISH_HALFWAY;
						}
					}

				}
				// otherwise the round is at an end
				else {
					gracefulExitProcedure = Constants.GRACEFUL_EXIT_END_OF_ROUND;
				}

				// make the appropriate utterance to indicate to the players that we're about to gracefully exit
				if (sendReceiveROSMessages) {
					if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_EXPLAINER_FINISH) {
						SendRobotUtterance ("graceful-exit-explainer-finish", false);
					} else if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_BUILDER_FINISH_HALFWAY) {
						SendRobotUtterance ("graceful-exit-builder-finish-halfway", false);
					} else if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_BUILDER_FINISH_COMPLETELY) {
						SendRobotUtterance ("graceful-exit-builder-finish-completely", false);
					} else if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_END_OF_ROUND) {
						SendRobotUtterance ("graceful-exit-end-of-round", false);
					}
				}

			} else if (internalGameState == Constants.GRACEFUL_EXIT) {

				//// <--------------- Begin Graceful Exit Procedures -----------------> ///


				if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_EXPLAINER_FINISH) {

					UpdateRocketPieceCounts ();

					//- (1) first chance to quit after the explainer has finished building
					//- check to see whether or not the explainer is building 
					//- check to see what level the game is on
					//- based on level, check to see whether totalNumExplainerRocketPieces == to number
					//  of pieces total required to compose a rocket in that level
					//- if this is true, Application.Quit();
					//- if this is false, call Ender(); again with the Update(); function

					if (levelNumber == 1 || tutorialMode == Constants.TUTORIAL_ON) {

						if (totalNumExplainerRocketPieces == 9) {
							internalGameState = Constants.QUIT_GAME;
						} 

					} else if (levelNumber == 2) {

						if (totalNumExplainerRocketPieces == 12) {
							internalGameState = Constants.QUIT_GAME;
						} 

					} else if (levelNumber == 3) {

						if (totalNumExplainerRocketPieces == 15) {
							internalGameState = Constants.QUIT_GAME;
						}

					} else {

						if (totalNumExplainerRocketPieces == 19) {
							internalGameState = Constants.QUIT_GAME;
						} 
					}
				}

				//(2) and (3) chance to quit
				//comes if the builder has completed building or if the builder is less than half-way complete,
				//after the builder has reached half-way complete

				//check to see the level of the builder and accordingly call build to completion,
				//meaning the builder has already built past half-way complete or call
				//build to half, meaning the builder has built less than half-way to completion.


				else if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_BUILDER_FINISH_HALFWAY) {

					UpdateRocketPieceCounts ();

					if (levelNumber == 1 || tutorialMode == Constants.TUTORIAL_ON) {

						if (totalNumBuilderRocketPieces == 4) {
							internalGameState = Constants.QUIT_GAME;
						} 

					} else if (levelNumber == 2) {

						if (totalNumBuilderRocketPieces == 6) {
							internalGameState = Constants.QUIT_GAME;
						} 

					} else if (levelNumber == 3) {

						if (totalNumBuilderRocketPieces == 8) {
							internalGameState = Constants.QUIT_GAME;
						} 

					} else {

						if (totalNumBuilderRocketPieces == 10) {
							internalGameState = Constants.QUIT_GAME;
						} 
					}

				}

				// Check whether the builder has built to completion

				else if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_BUILDER_FINISH_COMPLETELY) {

					UpdateRocketPieceCounts ();

					if (levelNumber == 1 || tutorialMode == Constants.TUTORIAL_ON) {

						if (totalNumBuilderRocketPieces == 9) {
							internalGameState = Constants.QUIT_GAME;
						}

					} else if (levelNumber == 2) {

						if (totalNumBuilderRocketPieces == 12) {
							internalGameState = Constants.QUIT_GAME;
						}

					} else if (levelNumber == 3) {

						if (totalNumBuilderRocketPieces == 15) {
							internalGameState = Constants.QUIT_GAME;
						} 

					} else {

						if (totalNumBuilderRocketPieces == 19) {
							internalGameState = Constants.QUIT_GAME;
						}
					}
				} 

				else if (gracefulExitProcedure == Constants.GRACEFUL_EXIT_END_OF_ROUND) {
					internalGameState = Constants.QUIT_GAME;
				}

				//// <--------------- End Graceful Exit Procedures -----------------> ///

			}

			if ((int)timeElapsed > secondsElapsed) {
				secondsElapsed = (int)timeElapsed;

				if (recentScreenInteraction) {
					secondsOfInactivity = 0;
					recentScreenInteraction = false;
				} else if (currentScene == Constants.BUILDER_SCENE || currentScene == Constants.EXPLAINER_SCENE) {
					secondsOfInactivity += 1;
				}

				if (tutorialMode == Constants.TUTORIAL_OFF) {

					// have the robot make an encouragement utterance every so often if we're in the explainer or builder scenes
					if (secondsElapsed % frequencyOfEncouragementSecs == 0 &&
					    (currentScene == Constants.EXPLAINER_SCENE || currentScene == Constants.BUILDER_SCENE)) {
						SendRobotUtterance ("encouragement", false);
					}

					// have the robot prompt the child/parent to continue working if they've been inactive for a 
					// specified amount of time
					if (secondsOfInactivity % frequencyOfPromptingPlayersFromInactivity == 0 && secondsOfInactivity > 0) {
						SendRobotUtterance ("prompt-inactivity", false);
					}

					// if the round has been going on for the max acceptable round time, prompt the players
					// every minute
					if (secondsElapsed % 60 == 0 && secondsElapsed > maxAcceptableRoundTimeSecs &&
					    (currentScene == Constants.EXPLAINER_SCENE || currentScene == Constants.BUILDER_SCENE)) {
						SendRobotUtterance ("prompt-hurry-up", false);
					}
				}

				if (internalGameState == Constants.QUIT_GAME) {
					
					// send a goodbye robot utterance after the game has ended
					if (secondsAfterGracefulExit == 3) {
						SceneManager.LoadScene ("End");
						SendRobotUtterance ("goodbye", false);
					}
					// quit the application a few seconds after the message for the robot utterance has been sent
					else if (secondsAfterGracefulExit == 10) {
						
						if (!sentPerformanceMetricsAtEndOfGame) {
							// update and send the performance metrics
							UpdatePerformanceMetrics (false);
							SendPerformanceMetrics ();

							sentPerformanceMetricsAtEndOfGame = true;
						}

						Debug.Log ("Quitting game application");
						Application.Quit ();
					}

					// increment the secondsAfterGracefulExit
					secondsAfterGracefulExit++;
				}
			}

			timeElapsed += Time.deltaTime;

		}

		void OnDestroy () {
			// close websocket
			if(this.clientSocket != null && sendReceiveROSMessages) {
				this.clientSocket.CloseSocket();

				// unsubscribe from received message events
				this.clientSocket.receivedMsgEvent -= HandleClientSocketReceivedMsgEvent;
			}
		}

		public void ClearSceneVariables () {

			// dashed outline slots
			dashedBodyOutlineSlots.Clear ();
			dashedBoosterOutlineSlots.Clear ();
			dashedConeOutlineSlot.Clear ();
			dashedFinOutlineSlots.Clear ();

			// selected outline slots
			selectedBodyOutlineSlots.Clear ();
			selectedBoosterOutlineSlots.Clear ();
			selectedConeOutlineSlot.Clear ();
			selectedFinOutlineSlots.Clear ();

			// panel pieces
			bodyPanelRocketPieces.Clear ();
			boosterPanelRocketPieces.Clear ();
			conePanelRocketPieces.Clear ();
			finPanelRocketPieces.Clear ();

		}

		public void DisableDragAndDropGameplay () {

			// subscribe to the events that indicate clicks on outline pieces
			Slot.OnClickForPanelChangeOutlinePiece -= TriggerPanelChange;

			// subscribe to the event that alerts the game manager of pieces added to the rocket
			Slot.OnPieceAddedToRocket -= PieceAddedToRocket;

			// subscribe to the event that alerts the game manager of the panel going in
			PanelAnimationEventHandler.OnTriggerPanelIn -= PanelIn;
			DragHandler.OnClickForPanelChangeRocketPiece -= TriggerPanelChange;

			// subscribe to the event that alerts the game manager of cloned pieces added to the panel
			DragHandler.OnPieceClonedToPanel -= PieceAddedToPanel;

			// subscribe to the event that alerts the game manager of a deleted piece (via trash)
			DragHandler.OnPieceRemovedByTrash -= PieceRemoved;

		}

		public void DisableTouchOfRocketPieces () {
			// get the panel transforms the pieces are containted within
			Transform bodySelectedOutlinesPanel = SelectedOutlinesPanel.FindChild ("BodySelectedOutlines"); 
			Transform boosterSelectedOutlinesPanel = SelectedOutlinesPanel.FindChild ("BoosterSelectedOutlines");
			Transform coneSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("ConeSelectedOutline");
			Transform leftFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("LeftFinSelectedOutline");
			Transform rightFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("RightFinSelectedOutline");

			// body pieces 
			for (int i = 0; i < bodySelectedOutlinesPanel.childCount; i++) {
				if (bodySelectedOutlinesPanel.GetChild (i).childCount > 0) {
					bodySelectedOutlinesPanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = false;
				}
			}
			// booster pieces 
			for (int i = 0; i < boosterSelectedOutlinesPanel.childCount; i++) {
				if (boosterSelectedOutlinesPanel.GetChild (i).childCount > 0) {
					boosterSelectedOutlinesPanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = false;
				}
			}
			// cone pieces 
			for (int i = 0; i < coneSelectedOutlinePanel.childCount; i++) {
				if (coneSelectedOutlinePanel.GetChild (i).childCount > 0) {
					coneSelectedOutlinePanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = false;
				}
			}
			// left fin pieces 
			for (int i = 0; i < leftFinSelectedOutlinePanel.childCount; i++) {
				if (leftFinSelectedOutlinePanel.GetChild (i).childCount > 0) {
					leftFinSelectedOutlinePanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = false;
				}
			}
			// right fin pieces 
			for (int i = 0; i < rightFinSelectedOutlinePanel.childCount; i++) {
				if (rightFinSelectedOutlinePanel.GetChild (i).childCount > 0) {
					rightFinSelectedOutlinePanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = false;
				}
			}
		}

		public void EnableDragAndDropGameplay () {

			// subscribe to the events that indicate clicks on outline pieces
			Slot.OnClickForPanelChangeOutlinePiece += TriggerPanelChange;

			// subscribe to the event that alerts the game manager of pieces added to the rocket
			Slot.OnPieceAddedToRocket += PieceAddedToRocket;

			// subscribe to the event that alerts the game manager of the panel going in
			PanelAnimationEventHandler.OnTriggerPanelIn += PanelIn;
			DragHandler.OnClickForPanelChangeRocketPiece += TriggerPanelChange;

			// subscribe to the event that alerts the game manager of cloned pieces added to the panel
			DragHandler.OnPieceClonedToPanel += PieceAddedToPanel;

			// subscribe to the event that alerts the game manager of a deleted piece (via trash)
			DragHandler.OnPieceRemovedByTrash += PieceRemoved;

		}

		public void EnableTouchOfRocketPieces () {
			// get the panel transforms the pieces are containted within
			Transform bodySelectedOutlinesPanel = SelectedOutlinesPanel.FindChild ("BodySelectedOutlines"); 
			Transform boosterSelectedOutlinesPanel = SelectedOutlinesPanel.FindChild ("BoosterSelectedOutlines");
			Transform coneSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("ConeSelectedOutline");
			Transform leftFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("LeftFinSelectedOutline");
			Transform rightFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("RightFinSelectedOutline");

			// body pieces 
			for (int i = 0; i < bodySelectedOutlinesPanel.childCount; i++) {
				if (bodySelectedOutlinesPanel.GetChild (i).childCount > 0) {
					bodySelectedOutlinesPanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = true;
				}
			}
			// booster pieces 
			for (int i = 0; i < boosterSelectedOutlinesPanel.childCount; i++) {
				if (boosterSelectedOutlinesPanel.GetChild (i).childCount > 0) {
					boosterSelectedOutlinesPanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = true;
				}
			}
			// cone pieces 
			for (int i = 0; i < coneSelectedOutlinePanel.childCount; i++) {
				if (coneSelectedOutlinePanel.GetChild (i).childCount > 0) {
					coneSelectedOutlinePanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = true;
				}
			}
			// left fin pieces 
			for (int i = 0; i < leftFinSelectedOutlinePanel.childCount; i++) {
				if (leftFinSelectedOutlinePanel.GetChild (i).childCount > 0) {
					leftFinSelectedOutlinePanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = true;
				}
			}
			// right fin pieces 
			for (int i = 0; i < rightFinSelectedOutlinePanel.childCount; i++) {
				if (rightFinSelectedOutlinePanel.GetChild (i).childCount > 0) {
					rightFinSelectedOutlinePanel.GetChild (i).GetChild (0).GetComponent<DragHandler> ().enabled = true;
				}
			}
		}


		public void EndTutorial () {

			tutorialCompleted = true;

		}


		public List<GameObject> FindOneMistakeBetweenRockets () {

			List<List<GameObject>> allMistakes = new List<List<GameObject>> ();

			// body pieces
			for (int i = 0; i < explainerRocket.bodyPieces.Count; i++) {
				if (explainerRocket.bodyPieces [i].name != builderRocket.bodyPieces [i].name) {
					allMistakes.Add (new List<GameObject> {explainerRocket.bodyPieces [i], builderRocket.bodyPieces [i]});
				}
			}
			// booster pieces
			for (int i = 0; i < explainerRocket.boosterPieces.Count; i++) {
				if (explainerRocket.boosterPieces [i].name != builderRocket.boosterPieces [i].name) {
					allMistakes.Add (new List<GameObject> {explainerRocket.boosterPieces [i], builderRocket.boosterPieces [i]});
				}
			}
			// cone piece
			for (int i = 0; i < explainerRocket.conePieces.Count; i++) {
				if (explainerRocket.conePieces [i].name != builderRocket.conePieces [i].name) {
					allMistakes.Add (new List<GameObject> {explainerRocket.conePieces [i], builderRocket.conePieces [i]});
				}
			}
			// fin pieces
			for (int i = 0; i < explainerRocket.finPieces.Count; i++) {
				if (explainerRocket.finPieces [i].name != builderRocket.finPieces [i].name) {
					allMistakes.Add (new List<GameObject> {explainerRocket.finPieces [i], builderRocket.finPieces [i]});
				}
			}

			if (allMistakes.Count > 0) {
				int randIndex = Random.Range (0, allMistakes.Count - 1);
				return allMistakes [randIndex];
			} else {
				return null;
			}

		}


		public int GetCurrentPieceTypeSelected () {

			return currentPieceTypeSelected;

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
						// and indicate that we've received the game start so we can get this ball rolling and the user
						// interacting with the game! 
						internalGameState = Constants.RECEIVED_GAME_START;
					}
				}
			} else if (command == Constants.GAME_COMMAND_END) {
				internalGameState = Constants.END_GAME;
			}
		}


		void HidePieces (List<GameObject> pieces) {
			foreach (GameObject piece in pieces) {
				piece.GetComponent<Image> ().enabled = false;
			}
		}

		public bool IsRocketComplete () {
			ConstructedRocket currentRocket;
			if (currentScene == Constants.EXPLAINER_SCENE) {
				currentRocket = explainerRocket;
			} else if (currentScene == Constants.BUILDER_SCENE) {
				currentRocket = builderRocket;
			} else {
				Debug.Log ("Cannot see if rocket is complete unless we're in the explainer or builder scene");
				return false;
			}

			// check body
			foreach (GameObject bodyPiece in currentRocket.bodyPieces) {
				if (bodyPiece == null) {
					return false;
				}
			}
			// check boosters
			foreach (GameObject boosterPiece in currentRocket.boosterPieces) {
				if (boosterPiece == null) {
					return false;
				}
			}
			// check cone
			foreach (GameObject conePiece in currentRocket.conePieces) {
				if (conePiece == null) {
					return false;
				}
			}
			// check fin
			foreach (GameObject finPiece in currentRocket.finPieces) {
				if (finPiece == null) {
					return false;
				}
			}
			return true;
		}

		// This is our cue to hide the old pieces on the panels and show the new ones
		void PanelIn () {

			// hide the rocket pieces of the old selected piece type
			if (lastPieceTypeSelected == Constants.BODY) {
				HidePieces (bodyPanelRocketPieces);
			} else if (lastPieceTypeSelected == Constants.BOOSTER) {
				HidePieces (boosterPanelRocketPieces);
			} else if (lastPieceTypeSelected == Constants.CONE) {
				HidePieces (conePanelRocketPieces);
			} else if (lastPieceTypeSelected == Constants.FIN) {
				HidePieces (finPanelRocketPieces);
			}

			// show the rocket pieces of the new selected piece type
			if (currentPieceTypeSelected == Constants.BODY) {
				ShowPieces (bodyPanelRocketPieces);
			} else if (currentPieceTypeSelected == Constants.BOOSTER) {
				ShowPieces (boosterPanelRocketPieces);
			} else if (currentPieceTypeSelected == Constants.CONE) {
				ShowPieces (conePanelRocketPieces);
			} else if (currentPieceTypeSelected == Constants.FIN) {
				ShowPieces (finPanelRocketPieces);

			}

			// indicate activity
			recentScreenInteraction = true;
		}

		void PieceAddedToPanel (GameObject pieceAdded) {
			if (pieceAdded.tag == "Body") {
				bodyPanelRocketPieces.Add (pieceAdded);
			} else if (pieceAdded.tag == "Engine") {
				boosterPanelRocketPieces.Add (pieceAdded);
			} else if (pieceAdded.tag == "LeftFin" || pieceAdded.tag == "RightFin") {
				finPanelRocketPieces.Add (pieceAdded);
			} else if (pieceAdded.tag == "TopCone") {
				conePanelRocketPieces.Add (pieceAdded);
			}

			// indicate activity
			recentScreenInteraction = true;
		}

		void PieceAddedToRocket (GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex) {

			// if we've moved a piece within the rocket, we must remove it from its previously stored location
			if (oldParentType == Constants.PARENT_ROCKET_OUTLINE) {
				if (pieceType == Constants.BODY) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.bodyPieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.bodyPieces [oldParentOutlineIndex] = null;
					}
				} else if (pieceType == Constants.BOOSTER) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.boosterPieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.boosterPieces [oldParentOutlineIndex] = null;
					}
				} else if (pieceType == Constants.CONE) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.conePieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.conePieces [oldParentOutlineIndex] = null;
					}
				} else if (pieceType == Constants.FIN) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.finPieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.finPieces [oldParentOutlineIndex] = null;
					}
				}
			}
			// if the piece added to the rocket was taken from the panel, remove it from the panel pieces list
			// and remove the piece from the current lists of rocket pieces
			else {
				int removalIndex = -1;
				if (pieceType == Constants.BODY) {
					for (int i = 0; i < bodyPanelRocketPieces.Count; i++) {
						if (pieceAdded.name == bodyPanelRocketPieces [i].name) {
							removalIndex = i;
							break;
						}
					}
					bodyPanelRocketPieces.RemoveAt (removalIndex);
				} else if (pieceType == Constants.BOOSTER) {
					for (int i = 0; i < boosterPanelRocketPieces.Count; i++) {
						if (pieceAdded.name == boosterPanelRocketPieces [i].name) {
							removalIndex = i;
							break;
						}
					}
					boosterPanelRocketPieces.RemoveAt (removalIndex);
				} else if (pieceType == Constants.CONE) {
					for (int i = 0; i < conePanelRocketPieces.Count; i++) {
						if (pieceAdded.name == conePanelRocketPieces [i].name) {
							removalIndex = i;
							break;
						}
					}
					conePanelRocketPieces.RemoveAt (removalIndex);
				} else if (pieceType == Constants.FIN) {
					for (int i = 0; i < finPanelRocketPieces.Count; i++) {
						if (pieceAdded.name == finPanelRocketPieces [i].name) {
							removalIndex = i;
							break;
						}
					}
					finPanelRocketPieces.RemoveAt (removalIndex);
				}
			}

			// add the piece added to the new location in the appropriate rocket pieces list
			if (pieceType == Constants.BODY) {
				GameObject bodyPrefabAdded = null;
				foreach (GameObject bodyPrefab in bodyRocketPieceOptions) {
					if (pieceAdded.name.Contains (bodyPrefab.name)) {
						bodyPrefabAdded = bodyPrefab;
						break;
					}
				}
				if (currentScene == Constants.EXPLAINER_SCENE) {
					explainerRocket.bodyPieces [newParentOutlineIndex] = bodyPrefabAdded;
				} else if (currentScene == Constants.BUILDER_SCENE) {
					builderRocket.bodyPieces [newParentOutlineIndex] = bodyPrefabAdded;
				}
			} else if (pieceType == Constants.BOOSTER) {
				GameObject boosterPrefabAdded = null;
				foreach (GameObject boosterPrefab in boosterRocketPieceOptions) {
					if (pieceAdded.name.Contains (boosterPrefab.name)) {
						boosterPrefabAdded = boosterPrefab;
						break;
					}
				}
				if (currentScene == Constants.EXPLAINER_SCENE) {
					explainerRocket.boosterPieces [newParentOutlineIndex] = boosterPrefabAdded;
				} else if (currentScene == Constants.BUILDER_SCENE) {
					builderRocket.boosterPieces [newParentOutlineIndex] = boosterPrefabAdded;
				}
			} else if (pieceType == Constants.CONE) {
				GameObject conePrefabAdded = null;
				foreach (GameObject conePrefab in coneRocketPieceOptions) {
					if (pieceAdded.name.Contains (conePrefab.name)) {
						conePrefabAdded = conePrefab;
						break;
					}
				}
				if (currentScene == Constants.EXPLAINER_SCENE) {
					explainerRocket.conePieces [newParentOutlineIndex] = conePrefabAdded;
				} else if (currentScene == Constants.BUILDER_SCENE) {
					builderRocket.conePieces [newParentOutlineIndex] = conePrefabAdded;
				}
			} else if (pieceType == Constants.FIN) {
				GameObject finPrefabAdded = null;
				foreach (GameObject finPrefab in finRocketPieceOptions) {
					if (pieceAdded.name.Contains (finPrefab.name)) {
						finPrefabAdded = finPrefab;
						break;
					}
				}
				if (currentScene == Constants.EXPLAINER_SCENE) {
					explainerRocket.finPieces [newParentOutlineIndex] = finPrefabAdded;
				} else if (currentScene == Constants.BUILDER_SCENE) {
					builderRocket.finPieces [newParentOutlineIndex] = finPrefabAdded;  
				}
			}
			//			PrintCurrentBodyPieces ();

			// indicate activity
			recentScreenInteraction = true;

		}

		void PieceRemoved (GameObject pieceToRemove, int pieceType, int oldParentType, int oldParentOutlineIndex) {
			// remove the piece from our current rocket pieces list
			if (oldParentType == Constants.PARENT_ROCKET_OUTLINE) {
				if (pieceType == Constants.BODY) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.bodyPieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.bodyPieces [oldParentOutlineIndex] = null;
					}
				} else if (pieceType == Constants.BOOSTER) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.boosterPieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.boosterPieces [oldParentOutlineIndex] = null;
					}
				} else if (pieceType == Constants.CONE) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.conePieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.conePieces [oldParentOutlineIndex] = null;
					}
				} else if (pieceType == Constants.FIN) {
					if (currentScene == Constants.EXPLAINER_SCENE) {
						explainerRocket.finPieces [oldParentOutlineIndex] = null;
					} else if (currentScene == Constants.BUILDER_SCENE) {
						builderRocket.finPieces [oldParentOutlineIndex] = null;
					}
				}
			}
			//			PrintCurrentBodyPieces ();


			// indicate activity
			recentScreenInteraction = true;
		}

		public void PlaceExistingRocketPiecesOnRocketOutlines () {

			ConstructedRocket rocketToPlace;
			if (currentScene == Constants.EXPLAINER_SCENE) {
				rocketToPlace = explainerRocket;
			} else if (currentScene == Constants.BUILDER_SCENE) {
				rocketToPlace = builderRocket;
			} else {
				Debug.Log ("PlaceExistingRocketPiecesOnRocketOutlines () cannot be called unless in the explainer or builder scene");
				return;
			}

			// get the panel transforms we'll be placing pieces onto
			Transform bodySelectedOutlinesPanel = SelectedOutlinesPanel.FindChild ("BodySelectedOutlines"); 
			Transform boosterSelectedOutlinesPanel = SelectedOutlinesPanel.FindChild ("BoosterSelectedOutlines");
			Transform coneSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("ConeSelectedOutline");
			Transform leftFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("LeftFinSelectedOutline");
			Transform rightFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild ("RightFinSelectedOutline");

			// place the body pieces 
			for (int i = 0; i < rocketToPlace.bodyPieces.Count; i++) {
				if (rocketToPlace.bodyPieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (rocketToPlace.bodyPieces [i]);
					rocketPieceClone.transform.SetParent (bodySelectedOutlinesPanel.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}
			// place the booster pieces 
			for (int i = 0; i < rocketToPlace.boosterPieces.Count; i++) {
				if (rocketToPlace.boosterPieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (rocketToPlace.boosterPieces [i]);
					rocketPieceClone.transform.SetParent (boosterSelectedOutlinesPanel.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}
			// place the cone pieces
			for (int i = 0; i < rocketToPlace.conePieces.Count; i++) {
				if (rocketToPlace.conePieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (rocketToPlace.conePieces [i]);
					rocketPieceClone.transform.SetParent (coneSelectedOutlinePanel.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}
			// place the fin pieces 
			if (rocketToPlace.finPieces [0] != null) {
				GameObject rocketPieceClone = Instantiate (rocketToPlace.finPieces [0]);
				rocketPieceClone.transform.SetParent (leftFinSelectedOutlinePanel.GetChild (0));
				rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				rocketPieceClone.GetComponent<Image> ().enabled = true;
			}
			if (rocketToPlace.finPieces [1] != null) {
				GameObject rocketPieceClone = Instantiate (rocketToPlace.finPieces [1]);
				rocketPieceClone.transform.SetParent (rightFinSelectedOutlinePanel.GetChild (0));
				rocketPieceClone.tag = "RightFin";
				rocketPieceClone.transform.GetChild (0).tag = "LeftFin";
				rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (-1f, 1f, 1f);
				rocketPieceClone.GetComponent<Image> ().enabled = true;
			}

		}

		void PlaceOutlineAndRocketPieces () {

			/*-------------------------------------- Outline Pieces --------------------------------------*/

			// get the body oulines panel transforms
			Transform bodyDashedOutlinesPanel = DashedOutlinesPanel.FindChild("BodyDashedOutlines");
			Transform bodySelectedOutlinesPanel = SelectedOutlinesPanel.FindChild ("BodySelectedOutlines");

			// set the psudo level number for use in this function - 
			// the levelNumber may be 4 but the psudoLevelNumber may be 1, as in during the tutorial
			int psudoLevelNumber;
			if (tutorialMode == Constants.TUTORIAL_ON) {
				psudoLevelNumber = 1;
			} else {
				psudoLevelNumber = levelNumber;
			}

			// get the constraints of the rows, columns, and number of body pieces
			int numBodyOutlines = Constants.LEVEL_INFO [psudoLevelNumber].numBodyOutlines;
			int numColumns = Constants.LEVEL_INFO [psudoLevelNumber].numBoosterOutlines;
			int numRows = numBodyOutlines / numColumns;

			// resize the panels
			bodyDashedOutlinesPanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, (float)numRows / 4f, 1f);
			bodySelectedOutlinesPanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, (float)numRows / 4f, 1f);

			// alter the panels' properties so that their children are the right size and in the right layout 
			float currentCellSize = bodyDashedOutlinesPanel.GetComponent<GridLayoutGroup> ().cellSize.x;
			float currentSpacing = bodyDashedOutlinesPanel.GetComponent<GridLayoutGroup> ().spacing.x;
			bodyDashedOutlinesPanel.GetComponent<GridLayoutGroup> ().cellSize = new Vector2 ((4f / (float)numColumns) * currentCellSize, (4f / (float)numRows) * currentCellSize);
			bodySelectedOutlinesPanel.GetComponent<GridLayoutGroup> ().cellSize = new Vector2 ((4f / (float)numColumns) * currentCellSize, (4f / (float)numRows) * currentCellSize);
			bodyDashedOutlinesPanel.GetComponent<GridLayoutGroup> ().spacing = new Vector2 ((4f / (float)numColumns) * currentSpacing, (4f / (float)numRows) * currentSpacing);
			bodySelectedOutlinesPanel.GetComponent<GridLayoutGroup> ().spacing = new Vector2 ((4f / (float)numColumns) * currentSpacing, (4f / (float)numRows) * currentSpacing);
			bodyDashedOutlinesPanel.GetComponent<GridLayoutGroup> ().constraintCount = numRows;
			bodySelectedOutlinesPanel.GetComponent<GridLayoutGroup> ().constraintCount = numRows;

			// position the panels for the body outline pieces 
			Vector3 currentBodyDashedOutlinesPanelPosition = bodyDashedOutlinesPanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentBodySelectedOutlinesPanelPosition = bodySelectedOutlinesPanel.GetComponent<RectTransform> ().transform.position;
			bodyDashedOutlinesPanel.GetComponent<RectTransform> ().transform.position = new Vector3 (0, 5, currentBodyDashedOutlinesPanelPosition.z);
			bodySelectedOutlinesPanel.GetComponent<RectTransform> ().transform.position = new Vector3 (0, 5, currentBodySelectedOutlinesPanelPosition.z);

			// put the body outline pieces in the panel
			int bodyOutlineNumber = 1;
			for (int i = 0; i < numBodyOutlines; i++) {
				// dashed ouline pieces 
				GameObject bodyDashedOutlineClone = Instantiate (bodyDashedOutline);
				bodyDashedOutlineClone.transform.SetParent (bodyDashedOutlinesPanel);
				bodyDashedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				bodyDashedOutlineClone.name = "BodyDashedOutline" + bodyOutlineNumber.ToString ();
				dashedBodyOutlineSlots.Add (bodyDashedOutlineClone);

				// selected outline pieces
				GameObject bodySelectedOutlineClone = Instantiate (bodySelectedOutline);
				bodySelectedOutlineClone.transform.SetParent (bodySelectedOutlinesPanel);
				bodySelectedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				bodySelectedOutlineClone.name = "BodySelectedOutline" + bodyOutlineNumber.ToString ();
				selectedBodyOutlineSlots.Add (bodySelectedOutlineClone);

				bodyOutlineNumber++;
			}

			// get the booster outlines panel transforms
			Transform boosterDashedOutlinesPanel = DashedOutlinesPanel.FindChild("BoosterDashedOutlines");
			Transform boosterSelectedOutlinesPanel = SelectedOutlinesPanel.FindChild("BoosterSelectedOutlines");

			// position the panels for the booster outline pieces
			Vector3 currentBoosterDashedOutlinesPanelPosition = boosterDashedOutlinesPanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentBoosterSelectedOutlinesPanelPosition = boosterSelectedOutlinesPanel.GetComponent<RectTransform> ().transform.position;
			boosterDashedOutlinesPanel.GetComponent<RectTransform> ().transform.position = new Vector3 (0, 5 - (numRows * 10), currentBoosterDashedOutlinesPanelPosition.z); 
			boosterSelectedOutlinesPanel.GetComponent<RectTransform> ().transform.position = new Vector3 (0, 5 - (numRows * 10), currentBoosterSelectedOutlinesPanelPosition.z); 

			// place booster outlines
			int numBoosterOutlines = Constants.LEVEL_INFO [psudoLevelNumber].numBoosterOutlines;
			int boosterOutlineNumber = 1;
			for (int i = 0; i < numBoosterOutlines; i++) {
				// dashed outline pieces
				GameObject boosterDashedOutlineClone = Instantiate (boosterDashedOutline);
				boosterDashedOutlineClone.transform.SetParent (boosterDashedOutlinesPanel);
				boosterDashedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				boosterDashedOutlineClone.name = "BoosterDashedOutline" + boosterOutlineNumber.ToString ();
				dashedBoosterOutlineSlots.Add (boosterDashedOutlineClone);

				// selected outline pieces
				GameObject boosterSelectedOutlineClone = Instantiate (boosterSelectedOutline);
				boosterSelectedOutlineClone.transform.SetParent (boosterSelectedOutlinesPanel);
				boosterSelectedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				boosterSelectedOutlineClone.name = "BoosterSelectedOutline" + boosterOutlineNumber.ToString ();
				selectedBoosterOutlineSlots.Add (boosterSelectedOutlineClone);

				boosterOutlineNumber++;
			}

			// get the cone outlines panel transforms
			Transform coneDashedOutlinePanel = DashedOutlinesPanel.FindChild("ConeDashedOutline");
			Transform coneSelectedOutlinePanel = SelectedOutlinesPanel.FindChild("ConeSelectedOutline");

			// resize and position the panel for the cone outline piece
			coneDashedOutlinePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, 1f, 1f);
			coneSelectedOutlinePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, 1f, 1f);
			Vector3 currentConeDashedOutlinePanelPosition = coneDashedOutlinePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentConeSelectedOutlinePanelPosition = coneSelectedOutlinePanel.GetComponent<RectTransform> ().transform.position;
			coneDashedOutlinePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (0, 9 + (numRows * 9), currentConeDashedOutlinePanelPosition.z); 
			coneSelectedOutlinePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (0, 9 + (numRows * 9), currentConeSelectedOutlinePanelPosition.z); 

			// place the cone - dashed
			GameObject coneDashedOutlineClone = Instantiate (coneDashedOutline);
			coneDashedOutlineClone.transform.SetParent (coneDashedOutlinePanel);
			coneDashedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			dashedConeOutlineSlot.Add (coneDashedOutlineClone);

			// place the cone - selected
			GameObject coneSelectedOutlineClone = Instantiate (coneSelectedOutline);
			coneSelectedOutlineClone.transform.SetParent (coneSelectedOutlinePanel);
			coneSelectedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			selectedConeOutlineSlot.Add (coneSelectedOutlineClone);

			// get the left fin outline panel transforms
			Transform leftFinDashedOutlinePanel = DashedOutlinesPanel.FindChild("LeftFinDashedOutline");
			Transform leftFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild("LeftFinSelectedOutline");

			// resize and position the panel for the left fin piece
			leftFinDashedOutlinePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			leftFinSelectedOutlinePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			Vector3 currentLeftFinDashedOutlinePanelPosition = leftFinDashedOutlinePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentLeftFinSelectedOutlinePanelPosition = leftFinSelectedOutlinePanel.GetComponent<RectTransform> ().transform.position;
			leftFinDashedOutlinePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (-(numColumns * 9 + 5), 5, currentLeftFinDashedOutlinePanelPosition.z); 
			leftFinSelectedOutlinePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (-(numColumns * 9 + 5), 5, currentLeftFinSelectedOutlinePanelPosition.z); 

			// place the left fin piece - dashed
			GameObject leftFinDashedOutlineClone = Instantiate (leftFinDashedOutline);
			leftFinDashedOutlineClone.transform.SetParent (leftFinDashedOutlinePanel);
			leftFinDashedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			dashedFinOutlineSlots.Add (leftFinDashedOutlineClone);

			// place the left fin piece - selected
			GameObject leftFinSelectedOutlineClone = Instantiate (leftFinSelectedOutline);
			leftFinSelectedOutlineClone.transform.SetParent (leftFinSelectedOutlinePanel);
			leftFinSelectedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			selectedFinOutlineSlots.Add (leftFinSelectedOutlineClone);

			// get the right fin outline panel transforms
			Transform rightFinDashedOutlinePanel = DashedOutlinesPanel.FindChild("RightFinDashedOutline");
			Transform rightFinSelectedOutlinePanel = SelectedOutlinesPanel.FindChild("RightFinSelectedOutline");

			// resize and position the panel for the right fin piece
			rightFinDashedOutlinePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			rightFinSelectedOutlinePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			Vector3 currentRightFinDashedOutlinePanelPosition = rightFinDashedOutlinePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentRightFinSelectedOutlinePanelPosition = rightFinSelectedOutlinePanel.GetComponent<RectTransform> ().transform.position;
			rightFinDashedOutlinePanel.GetComponent<RectTransform> ().transform.position = new Vector3 ((numColumns * 9 + 5), 5, currentRightFinDashedOutlinePanelPosition.z); 
			rightFinSelectedOutlinePanel.GetComponent<RectTransform> ().transform.position = new Vector3 ((numColumns * 9 + 5), 5, currentRightFinSelectedOutlinePanelPosition.z); 

			// place the right fin piece - dashed
			GameObject rightFinDashedOutlineClone = Instantiate (rightFinDashedOutline);
			rightFinDashedOutlineClone.transform.SetParent (rightFinDashedOutlinePanel);
			rightFinDashedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			rightFinDashedOutlineClone.tag = "RightFin";
			dashedFinOutlineSlots.Add (rightFinDashedOutlineClone);

			// place the right fin piece - selected
			GameObject rightFinSelectedOutlineClone = Instantiate (rightFinSelectedOutline);
			rightFinSelectedOutlineClone.transform.SetParent (rightFinSelectedOutlinePanel);
			rightFinSelectedOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			rightFinSelectedOutlineClone.tag = "RightFin";
			selectedFinOutlineSlots.Add (rightFinSelectedOutlineClone);


			/*-------------------------------------- Rocket Pieces on the Panels --------------------------------------*/

			// body pieces
			Transform bodyPieceParentPanelLeft = RocketPiecesLeftPanel.FindChild ("LeftBodyPieces");
			Transform bodyPieceParentPanelRight = RocketPiecesRightPanel.FindChild ("RightBodyPieces");
			int totalBodyPieceOptions = Constants.LEVEL_INFO [psudoLevelNumber].bodyPieceOptionNames.Count;
			int currentBodyPieceOptions = 0;
			for (int i = 0; i < totalBodyPieceOptions; i++) {
				for (int j = 0; j < bodyRocketPieceOptions.Count; j++) {
					if (Constants.LEVEL_INFO [psudoLevelNumber].bodyPieceOptionNames [i] == bodyRocketPieceOptions [j].name) {
						GameObject panelBodyPieceClone = Instantiate (bodyRocketPieceOptions [j]);
						if (currentBodyPieceOptions * 2 < totalBodyPieceOptions) {
							panelBodyPieceClone.transform.SetParent (bodyPieceParentPanelLeft);
						} else {
							panelBodyPieceClone.transform.SetParent (bodyPieceParentPanelRight);
						}
						panelBodyPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
						bodyPanelRocketPieces.Add (panelBodyPieceClone);
						currentBodyPieceOptions++;
					}
				}
			}

			// booster pieces
			Transform boosterPieceParentPanelLeft = RocketPiecesLeftPanel.FindChild ("LeftBoosterPieces");
			Transform boosterPieceParentPanelRight = RocketPiecesRightPanel.FindChild ("RightBoosterPieces");
			int totalBoosterPieceOptions = Constants.LEVEL_INFO [psudoLevelNumber].boosterPieceOptionNames.Count;
			int currentBoosterPieceOptions = 0;
			for (int i = 0; i < totalBoosterPieceOptions; i++) {
				for (int j = 0; j < boosterRocketPieceOptions.Count; j++) {
					if (Constants.LEVEL_INFO [psudoLevelNumber].boosterPieceOptionNames [i] == boosterRocketPieceOptions [j].name) {
						GameObject panelBoosterPieceClone = Instantiate (boosterRocketPieceOptions [j]);
						if (currentBoosterPieceOptions * 2 < totalBoosterPieceOptions) {
							panelBoosterPieceClone.transform.SetParent (boosterPieceParentPanelLeft);
						} else {
							panelBoosterPieceClone.transform.SetParent (boosterPieceParentPanelRight);
						}
						panelBoosterPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
						boosterPanelRocketPieces.Add (panelBoosterPieceClone);
						currentBoosterPieceOptions++;
					}
				}
			}

			// cone pieces
			Transform conePieceParentPanelLeft = RocketPiecesLeftPanel.FindChild ("LeftConePieces");
			Transform conePieceParentPanelRight = RocketPiecesRightPanel.FindChild ("RightConePieces");
			int totalConePieceOptions = Constants.LEVEL_INFO [psudoLevelNumber].conePieceOptionNames.Count;
			int currentConePieceOptions = 0;
			for (int i = 0; i < totalConePieceOptions; i++) {
				for (int j = 0; j < coneRocketPieceOptions.Count; j++) {
					if (Constants.LEVEL_INFO [psudoLevelNumber].conePieceOptionNames [i] == coneRocketPieceOptions [j].name) {
						GameObject panelConePieceClone = Instantiate (coneRocketPieceOptions [j]);
						if (currentConePieceOptions * 2 < totalConePieceOptions) {
							panelConePieceClone.transform.SetParent (conePieceParentPanelLeft);
						} else {
							panelConePieceClone.transform.SetParent (conePieceParentPanelRight);
						}
						panelConePieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
						conePanelRocketPieces.Add (panelConePieceClone);
						currentConePieceOptions++;
					}
				}
			}

			// fin pieces
			Transform finPieceParentPanelLeft = RocketPiecesLeftPanel.FindChild ("LeftFinPieces");
			Transform finPieceParentPanelRight = RocketPiecesRightPanel.FindChild ("RightFinPieces");
			int totalFinPieceOptions = Constants.LEVEL_INFO [psudoLevelNumber].finPieceOptionNames.Count;
			int currentFinPieceOptions = 0;
			for (int i = 0; i < totalFinPieceOptions; i++) {
				for (int j = 0; j < finRocketPieceOptions.Count; j++) {
					if (Constants.LEVEL_INFO [psudoLevelNumber].finPieceOptionNames [i] == finRocketPieceOptions [j].name) {
						GameObject panelFinPieceClone = Instantiate (finRocketPieceOptions [j]);
						if (currentFinPieceOptions * 2 < totalFinPieceOptions) {
							panelFinPieceClone.transform.SetParent (finPieceParentPanelLeft);
						} else {
							panelFinPieceClone.transform.SetParent (finPieceParentPanelRight);
						}
						panelFinPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
						panelFinPieceClone.tag = "LeftFin";
						panelFinPieceClone.transform.GetChild (0).gameObject.tag = "RightFin";
						finPanelRocketPieces.Add (panelFinPieceClone);
						currentFinPieceOptions++;
					}
				}
			}


		}

		public void PlaceRocketsOnScreenSideBySide () {

			// get all of the explainer panels
			GameObject explainerRocketPanel = GameObject.Find ("ExplainerRocket");
			GameObject explainerBodyPiecePanel = explainerRocketPanel.transform.FindChild ("ExplainerBodyPieces").gameObject;
			GameObject explainerBoosterPiecePanel = explainerRocketPanel.transform.FindChild ("ExplainerBoosterPieces").gameObject;
			GameObject explainerConePiecePanel = explainerRocketPanel.transform.FindChild ("ExplainerConePiece").gameObject;
			GameObject explainerLeftFinPiecePanel = explainerRocketPanel.transform.FindChild ("ExplainerLeftFinPiece").gameObject;
			GameObject explainerRightFinPiecePanel = explainerRocketPanel.transform.FindChild ("ExplainerRightFinPiece").gameObject;

			// get all of the builder panels
			GameObject builderRocketPanel = GameObject.Find ("BuilderRocket");
			GameObject builderBodyPiecePanel = builderRocketPanel.transform.FindChild ("BuilderBodyPieces").gameObject;
			GameObject builderBoosterPiecePanel = builderRocketPanel.transform.FindChild ("BuilderBoosterPieces").gameObject;
			GameObject builderConePiecePanel = builderRocketPanel.transform.FindChild ("BuilderConePiece").gameObject;
			GameObject builderLeftFinPiecePanel = builderRocketPanel.transform.FindChild ("BuilderLeftFinPiece").gameObject;
			GameObject builderRightFinPiecePanel = builderRocketPanel.transform.FindChild ("BuilderRightFinPiece").gameObject;

			// set the psudo level number for use in this function - 
			// the levelNumber may be 4 but the psudoLevelNumber may be 1, as in during the tutorial
			int psudoLevelNumber;
			if (tutorialMode == Constants.TUTORIAL_ON) {
				psudoLevelNumber = 1;
			} else {
				psudoLevelNumber = levelNumber;
			}

			// get the constraints of the rows, columns, and number of body pieces
			int numBodyOutlines = Constants.LEVEL_INFO [psudoLevelNumber].numBodyOutlines;
			int numColumns = Constants.LEVEL_INFO [psudoLevelNumber].numBoosterOutlines;
			int numRows = numBodyOutlines / numColumns;
			float offsetFromCenter = 35f;

			/*---------------------------------------------- BODY PIECES ----------------------------------------------*/

			// adjust the size of the body panels
			explainerBodyPiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, (float)numRows / 4f, 1f);
			builderBodyPiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, (float)numRows / 4f, 1f);

			// alter the panels' properties so that their children are the right size and in the right layout 
			float currentCellSize = explainerBodyPiecePanel.GetComponent<GridLayoutGroup> ().cellSize.x;
			float currentSpacing = explainerBodyPiecePanel.GetComponent<GridLayoutGroup> ().spacing.x;
			explainerBodyPiecePanel.GetComponent<GridLayoutGroup> ().cellSize = new Vector2 ((4f / (float)numColumns) * currentCellSize, (4f / (float)numRows) * currentCellSize);
			builderBodyPiecePanel.GetComponent<GridLayoutGroup> ().cellSize = new Vector2 ((4f / (float)numColumns) * currentCellSize, (4f / (float)numRows) * currentCellSize);
			explainerBodyPiecePanel.GetComponent<GridLayoutGroup> ().spacing = new Vector2 ((4f / (float)numColumns) * currentSpacing, (4f / (float)numRows) * currentSpacing);
			builderBodyPiecePanel.GetComponent<GridLayoutGroup> ().spacing = new Vector2 ((4f / (float)numColumns) * currentSpacing, (4f / (float)numRows) * currentSpacing);
			explainerBodyPiecePanel.GetComponent<GridLayoutGroup> ().constraintCount = numRows;
			builderBodyPiecePanel.GetComponent<GridLayoutGroup> ().constraintCount = numRows;

			// position the panels for the body outline pieces 
			Vector3 currentExplainerBodyPiecePanelPosition = explainerBodyPiecePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentBuilderBodyPiecePanelPosition = builderBodyPiecePanel.GetComponent<RectTransform> ().transform.position;
			explainerBodyPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (-offsetFromCenter, 5, currentExplainerBodyPiecePanelPosition.z);
			builderBodyPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (offsetFromCenter, 5, currentBuilderBodyPiecePanelPosition.z);

			// put the body outline pieces in the panel - hide them
			int bodyOutlineNumber = 1;
			for (int i = 0; i < numBodyOutlines; i++) {
				// dashed ouline pieces 
				GameObject explainerBodyOutlineClone = Instantiate (bodySelectedOutline);
				explainerBodyOutlineClone.transform.SetParent (explainerBodyPiecePanel.transform);
				explainerBodyOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				explainerBodyOutlineClone.GetComponent<Image> ().enabled = false;
				explainerBodyOutlineClone.name = "BodyDashedOutline" + bodyOutlineNumber.ToString ();

				// selected outline pieces
				GameObject builderBodyOutlineClone = Instantiate (bodySelectedOutline);
				builderBodyOutlineClone.transform.SetParent (builderBodyPiecePanel.transform);
				builderBodyOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				builderBodyOutlineClone.GetComponent<Image> ().enabled = false;
				builderBodyOutlineClone.name = "BodySelectedOutline" + bodyOutlineNumber.ToString ();

				bodyOutlineNumber++;
			}

			// place the body pieces from the two rockets in their slots
			for (int i = 0; i < explainerRocket.bodyPieces.Count; i++) {
				if (explainerRocket.bodyPieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (explainerRocket.bodyPieces [i]);
					rocketPieceClone.transform.SetParent (explainerBodyPiecePanel.transform.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}
			for (int i = 0; i < builderRocket.bodyPieces.Count; i++) {
				if (builderRocket.bodyPieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (builderRocket.bodyPieces [i]);
					rocketPieceClone.transform.SetParent (builderBodyPiecePanel.transform.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}

			/*---------------------------------------------- BOOSTER PIECES ----------------------------------------------*/

			// position the panels for the booster outline pieces
			Vector3 currentExplainerBoosterPiecePanelPosition = explainerBoosterPiecePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentBuilderBoosterPiecePanelPosition = builderBoosterPiecePanel.GetComponent<RectTransform> ().transform.position;
			explainerBoosterPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (-offsetFromCenter, 5f - (numRows * 5f), currentExplainerBoosterPiecePanelPosition.z); 
			builderBoosterPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (offsetFromCenter, 5f - (numRows * 5f), currentBuilderBoosterPiecePanelPosition.z); 

			// place booster outlines
			int numBoosterOutlines = Constants.LEVEL_INFO [psudoLevelNumber].numBoosterOutlines;
			int boosterOutlineNumber = 1;
			for (int i = 0; i < numBoosterOutlines; i++) {
				// dashed outline pieces
				GameObject explainerBoosterOutlineClone = Instantiate (boosterSelectedOutline);
				explainerBoosterOutlineClone.transform.SetParent (explainerBoosterPiecePanel.transform);
				explainerBoosterOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				explainerBoosterOutlineClone.GetComponent<Image> ().enabled = false;
				explainerBoosterOutlineClone.name = "BoosterDashedOutline" + boosterOutlineNumber.ToString ();

				// selected outline pieces
				GameObject builderBoosterOutlineClone = Instantiate (boosterSelectedOutline);
				builderBoosterOutlineClone.transform.SetParent (builderBoosterPiecePanel.transform);
				builderBoosterOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				builderBoosterOutlineClone.GetComponent<Image> ().enabled = false;
				builderBoosterOutlineClone.name = "BoosterSelectedOutline" + boosterOutlineNumber.ToString ();

				boosterOutlineNumber++;
			}

			// place the booster pieces from the two rockets in their slots
			for (int i = 0; i < explainerRocket.boosterPieces.Count; i++) {
				if (explainerRocket.boosterPieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (explainerRocket.boosterPieces [i]);
					rocketPieceClone.transform.SetParent (explainerBoosterPiecePanel.transform.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}
			for (int i = 0; i < builderRocket.boosterPieces.Count; i++) {
				if (builderRocket.boosterPieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (builderRocket.boosterPieces [i]);
					rocketPieceClone.transform.SetParent (builderBoosterPiecePanel.transform.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}

			/*---------------------------------------------- CONE PIECES ----------------------------------------------*/

			// resize and position the panel for the cone outline piece
			explainerConePiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, 1f, 1f);
			builderConePiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((float)numColumns / 4f, 1f, 1f);
			Vector3 currentExplainerConePiecePanelPosition = explainerConePiecePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentBuilderConePiecePanelPanelPosition = builderConePiecePanel.GetComponent<RectTransform> ().transform.position;
			explainerConePiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (-offsetFromCenter, 7.5f + (numRows * 4.5f), currentExplainerConePiecePanelPosition.z); 
			builderConePiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (offsetFromCenter, 7.5f + (numRows * 4.5f), currentBuilderConePiecePanelPanelPosition.z); 

			// place the cone - dashed
			GameObject explainerConeOutlineClone = Instantiate (coneSelectedOutline);
			explainerConeOutlineClone.transform.SetParent (explainerConePiecePanel.transform);
			explainerConeOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			explainerConeOutlineClone.GetComponent<Image> ().enabled = false;

			// place the cone - selected
			GameObject builderConeOutlineClone = Instantiate (coneSelectedOutline);
			builderConeOutlineClone.transform.SetParent (builderConePiecePanel.transform);
			builderConeOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			builderConeOutlineClone.GetComponent<Image> ().enabled = false;

			// place the cone pieces from the two rockets in their slots
			for (int i = 0; i < explainerRocket.conePieces.Count; i++) {
				if (explainerRocket.conePieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (explainerRocket.conePieces [i]);
					rocketPieceClone.transform.SetParent (explainerConePiecePanel.transform.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}
			for (int i = 0; i < builderRocket.conePieces.Count; i++) {
				if (builderRocket.conePieces [i] != null) {
					GameObject rocketPieceClone = Instantiate (builderRocket.conePieces [i]);
					rocketPieceClone.transform.SetParent (builderConePiecePanel.transform.GetChild (i));
					rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
					rocketPieceClone.GetComponent<Image> ().enabled = true;
				}
			}

			/*---------------------------------------------- FIN PIECES ----------------------------------------------*/

			// resize and position the panel for the left fin piece
			explainerLeftFinPiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			builderLeftFinPiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			Vector3 currentExplainerLeftFinPiecePanelPosition = explainerLeftFinPiecePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentBuilderLeftFinPiecePanelPosition = builderLeftFinPiecePanel.GetComponent<RectTransform> ().transform.position;
			explainerLeftFinPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (-((float)numColumns * 4.5f + 2.5f) - offsetFromCenter, 5, currentExplainerLeftFinPiecePanelPosition.z); 
			builderLeftFinPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (-((float)numColumns * 4.5f + 2.5f) + offsetFromCenter, 5, currentBuilderLeftFinPiecePanelPosition.z); 

			// place the left fin piece - dashed
			GameObject explainerLeftFinOutlineClone = Instantiate (leftFinSelectedOutline);
			explainerLeftFinOutlineClone.transform.SetParent (explainerLeftFinPiecePanel.transform);
			explainerLeftFinOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			explainerLeftFinOutlineClone.GetComponent<Image> ().enabled = false;

			// place the left fin piece - selected
			GameObject builderLeftFinOutlineClone = Instantiate (leftFinSelectedOutline);
			builderLeftFinOutlineClone.transform.SetParent (builderLeftFinPiecePanel.transform);
			builderLeftFinOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			builderLeftFinOutlineClone.GetComponent<Image> ().enabled = false;

			// resize and position the panel for the right fin piece
			explainerRightFinPiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			builderRightFinPiecePanel.GetComponent<RectTransform> ().transform.localScale = new Vector3 ((0.5f * (float)numColumns / 4f) + 0.5f, (float)numRows / 4f, 1f);
			Vector3 currentExplainerRightFinPiecePanelPosition = explainerRightFinPiecePanel.GetComponent<RectTransform> ().transform.position;
			Vector3 currentBuilderRightFinPiecePanelPosition = builderRightFinPiecePanel.GetComponent<RectTransform> ().transform.position;
			explainerRightFinPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (((float)numColumns * 4.5f + 2.5f) - offsetFromCenter, 5, currentExplainerRightFinPiecePanelPosition.z); 
			builderRightFinPiecePanel.GetComponent<RectTransform> ().transform.position = new Vector3 (((float)numColumns * 4.5f + 2.5f) + offsetFromCenter, 5, currentBuilderRightFinPiecePanelPosition.z); 

			// place the right fin piece - dashed
			GameObject explainerRightFinOutlineClone = Instantiate (rightFinSelectedOutline);
			explainerRightFinOutlineClone.transform.SetParent (explainerRightFinPiecePanel.transform);
			explainerRightFinOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			explainerRightFinOutlineClone.GetComponent<Image> ().enabled = false;

			// place the right fin piece - selected
			GameObject builderRightFinOutlineClone = Instantiate (rightFinSelectedOutline);
			builderRightFinOutlineClone.transform.SetParent (builderRightFinPiecePanel.transform);
			builderRightFinOutlineClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
			builderRightFinOutlineClone.GetComponent<Image> ().enabled = false;

			// place the fin pieces from the two rockets in their slots
			if (explainerRocket.finPieces [0] != null) {
				GameObject rocketPieceClone = Instantiate (explainerRocket.finPieces [0]);
				rocketPieceClone.transform.SetParent (explainerLeftFinPiecePanel.transform.GetChild (0));
				rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				rocketPieceClone.GetComponent<Image> ().enabled = true;
			}
			if (explainerRocket.finPieces [1] != null) {
				GameObject rocketPieceClone = Instantiate (explainerRocket.finPieces [1]);
				rocketPieceClone.transform.SetParent (explainerRightFinPiecePanel.transform.GetChild (0));
				rocketPieceClone.tag = "RightFin";
				rocketPieceClone.transform.GetChild (0).tag = "LeftFin";
				rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (-1f, 1f, 1f);
				rocketPieceClone.GetComponent<Image> ().enabled = true;
			}

			if (builderRocket.finPieces [0] != null) {
				GameObject rocketPieceClone = Instantiate (builderRocket.finPieces [0]);
				rocketPieceClone.transform.SetParent (builderLeftFinPiecePanel.transform.GetChild (0));
				rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (1f, 1f, 1f);
				rocketPieceClone.GetComponent<Image> ().enabled = true;
			}
			if (builderRocket.finPieces [1] != null) {
				GameObject rocketPieceClone = Instantiate (builderRocket.finPieces [1]);
				rocketPieceClone.transform.SetParent (builderRightFinPiecePanel.transform.GetChild (0));
				rocketPieceClone.tag = "RightFin";
				rocketPieceClone.transform.GetChild (0).tag = "LeftFin";
				rocketPieceClone.GetComponent<RectTransform> ().transform.localScale = new Vector3 (-1f, 1f, 1f);
				rocketPieceClone.GetComponent<Image> ().enabled = true;
			}

		}

		void PrintCurrentBodyPieces () {
			ConstructedRocket tempRocket;
			// header and set up piece list to be the current one
			if (currentScene == Constants.EXPLAINER_SCENE) {
				Debug.Log ("Explainer Rocket: ");
				tempRocket = explainerRocket;
			} else if (currentScene == Constants.BUILDER_SCENE) {
				Debug.Log ("Builder Rocket: ");
				tempRocket = builderRocket;
			} else {
				Debug.Log ("Cannot print out rocket while not in builder or explainer scene");
				return;
			}
			// body
			string bodyPiecesString = "Body pieces: ";
			foreach (GameObject bodyPiece in tempRocket.bodyPieces) {
				if (bodyPiece == null) {
					bodyPiecesString += "- , ";
				} else {
					bodyPiecesString += bodyPiece.name + ", ";
				}
			}
			Debug.Log (bodyPiecesString);
			// boosters
			string boosterPiecesString = "Booster pieces: ";
			foreach (GameObject boosterPiece in tempRocket.boosterPieces) {
				if (boosterPiece == null) {
					boosterPiecesString += "- , ";
				} else {
					boosterPiecesString += boosterPiece.name + ", ";
				}
			}
			Debug.Log (boosterPiecesString);
			// cone(s)
			string conePieceString = "Cone piece: ";
			foreach (GameObject conePiece in tempRocket.conePieces) {
				if (conePiece == null) {
					conePieceString += "- , ";
				} else {
					conePieceString += conePiece.name + ", ";
				}
			}
			Debug.Log (conePieceString);
			// fins
			string finPiecesString = "Fin pieces: ";
			foreach (GameObject finPiece in tempRocket.finPieces) {
				if (finPiece == null) {
					finPiecesString += "- , ";
				} else {
					finPiecesString += finPiece.name + ", ";
				}
			}
			Debug.Log (finPiecesString);
		}


		public void SendRobotUtterance (string utteranceCategory, bool interrupt) {

			if (robotUtterances.ContainsKey(utteranceCategory)) {

				int randIndex = Random.Range (0, robotUtterances [utteranceCategory].Count - 1);
				List<string> chosenUtterance = robotUtterances [utteranceCategory] [randIndex];

				foreach (string chosenUtteranceSection in chosenUtterance) {
					// first prepare the utterance for sending
					string preparedUtterance = Utilities.PrepareUtteranceForSending (chosenUtteranceSection, childExplainer);

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


		private void SendPerformanceMetrics () {

			Dictionary<string, float> averagePerformanceMetrics = new Dictionary<string, float> ();

			foreach (KeyValuePair<int, List<float>> performanceMetricEntry in performanceMetrics) {
				if (performanceMetricEntry.Value.Count > 0) {

					// get the average value
					float averageMetricValue = 0f;
					foreach (float value in performanceMetricEntry.Value) {
						averageMetricValue += value;
					}
					averageMetricValue = averageMetricValue / (float)performanceMetricEntry.Value.Count;

					// get the name of the metric in there
					if (performanceMetricEntry.Key == Constants.CHILD_BUILDER_CHOICE_ACCURACY) {
						averagePerformanceMetrics.Add ("child-builder-piece-choice-accuracy", averageMetricValue);
					} else if (performanceMetricEntry.Key == Constants.CHILD_BUILDER_LOCATION_ACCURACY) {
						averagePerformanceMetrics.Add ("child-builder-location-accuracy", averageMetricValue);
					} else if (performanceMetricEntry.Key == Constants.CHILD_EXPLAINER_CHOICE_ACCURACY) {
						averagePerformanceMetrics.Add ("child-explainer-piece-choice-accuracy", averageMetricValue);
					} else if (performanceMetricEntry.Key == Constants.CHILD_EXPLAINER_LOCATION_ACCURACY) {
						averagePerformanceMetrics.Add ("child-explainer-location-accuracy", averageMetricValue);
					} else {
						Debug.Log ("Error: performance metric type not recognized in the sender");
						return;
					}
				}
			}

			if (sendReceiveROSMessages) {
				this.clientSocket.SendMessage (RosbridgeUtilities.GetROSJsonPublishGameStateMsg (
					Constants.STATE_ROSTOPIC, 
					Constants.GAME_STATE_END, 
					averagePerformanceMetrics));
			}

		}


		public void SetUpBuilderScene () {

			// initialize pieceTypeSelected
			currentPieceTypeSelected = Constants.NONE_SELECTED;
			lastPieceTypeSelected = Constants.NONE_SELECTED;

			// initialize firstStateChangeOccured
			firstStateChangeOccured = false;

			// find all of the objects we need access to
			DashedOutlinesPanel = GameObject.Find ("DashedOutlines").transform;
			SelectedOutlinesPanel = GameObject.Find ("SelectedOutlines").transform;
			RocketPiecesLeftPanel = GameObject.Find ("RocketPiecesLeft").transform;
			RocketPiecesRightPanel = GameObject.Find ("RocketPiecesRight").transform;
			panelsAnimator = GameObject.Find ("RocketField").GetComponent<Animator> ();

			PlaceOutlineAndRocketPieces ();

			UpdateOutlineAndRocketPanelPieces ();

		}

		public void SetUpExplainerScene () {

			// initialize pieceTypeSelected
			currentPieceTypeSelected = Constants.NONE_SELECTED;
			lastPieceTypeSelected = Constants.NONE_SELECTED;

			// initialize firstStateChangeOccured
			firstStateChangeOccured = false;

			// find all of the objects we need access to
			DashedOutlinesPanel = GameObject.Find ("DashedOutlines").transform;
			SelectedOutlinesPanel = GameObject.Find ("SelectedOutlines").transform;
			RocketPiecesLeftPanel = GameObject.Find ("RocketPiecesLeft").transform;
			RocketPiecesRightPanel = GameObject.Find ("RocketPiecesRight").transform;
			panelsAnimator = GameObject.Find ("RocketField").GetComponent<Animator> ();

			PlaceOutlineAndRocketPieces ();

			UpdateOutlineAndRocketPanelPieces ();

		}

		void ShowPieces (List<GameObject> pieces) {
			foreach (GameObject piece in pieces) {
				piece.GetComponent<Image> ().enabled = true;
			}
		}


		public void StartFirstGame () {

			// when we start the first game, the tutorial (if entered) is exited
			tutorialMode = Constants.TUTORIAL_OFF;

			internalGameState = Constants.START_GAME;

		}


		public void StartNextGame () {

			internalGameState = Constants.START_GAME;

			// swap the child and the caregiver's roles
			if (childExplainer) {
				childExplainer = false;
			} else {
				childExplainer = true;
			}

		}

		public void StartTutorial () {

			// turn the tutorial mode on
			tutorialMode = Constants.TUTORIAL_ON;

			internalGameState = Constants.START_GAME;

		}

		void TriggerPanelChange (int selectedOutlineType) {
			// only switch the panels if we're wanting to put on a different type of piece
			if (selectedOutlineType != currentPieceTypeSelected) {
				// play the animations to hide the sidebars
				if (firstStateChangeOccured == true) {
					panelsAnimator.SetTrigger ("movePanelsInOut");
				} else {
					panelsAnimator.SetTrigger ("movePanelsIn");
					firstStateChangeOccured = true;
				}

				// update the current and last piece type selected varaibles
				lastPieceTypeSelected = currentPieceTypeSelected;
				currentPieceTypeSelected = selectedOutlineType;

				// show/hide the appropriate ouline and rocket panel pieces
				UpdateOutlineAndRocketPanelPieces();

				// indicate activity
				recentScreenInteraction = true;
			}
		}

		void UpdateRocketPieceCounts () {

			totalNumBuilderRocketPieces = 0;
			totalNumExplainerRocketPieces = 0;

			//logic to calculate the total number of pieces on the builder rocket
			foreach (GameObject bodyPiece in builderRocket.bodyPieces) {
				if (bodyPiece != null) {
					totalNumBuilderRocketPieces += 1;
				}
			}
			foreach (GameObject boosterPiece in builderRocket.boosterPieces) {
				if (boosterPiece != null) {
					totalNumBuilderRocketPieces += 1;
				}
			} 
			foreach (GameObject conePiece in builderRocket.conePieces) {
				if (conePiece != null) {
					totalNumBuilderRocketPieces += 1;
				}
			}
			foreach (GameObject finPiece in builderRocket.finPieces) {
				if (finPiece != null) {
					totalNumBuilderRocketPieces += 1;
				}
			}


			//logic to calculate the total number of pieces on the explainer rocket
			foreach (GameObject bodyPiece in explainerRocket.bodyPieces) {
				if (bodyPiece != null) {
					totalNumExplainerRocketPieces += 1;
				}
			}
			foreach (GameObject boosterPiece in explainerRocket.boosterPieces) {
				if (boosterPiece != null) {
					totalNumExplainerRocketPieces += 1;
				}
			} 
			foreach (GameObject conePiece in explainerRocket.conePieces) {
				if (conePiece != null) {
					totalNumExplainerRocketPieces += 1;
				}
			}
			foreach (GameObject finPiece in explainerRocket.finPieces) {
				if (finPiece != null) {
					totalNumExplainerRocketPieces += 1;
				}
			}

		}

		void UpdateOutlineAndRocketPanelPieces() {

			// hide the selected outlines and show the dashed outlines of the old selected piece type
			if (lastPieceTypeSelected == Constants.BODY) {
				HidePieces (selectedBodyOutlineSlots);
				ShowPieces (dashedBodyOutlineSlots);
			} else if (lastPieceTypeSelected == Constants.BOOSTER) {
				HidePieces (selectedBoosterOutlineSlots);
				ShowPieces (dashedBoosterOutlineSlots);
			} else if (lastPieceTypeSelected == Constants.CONE) {
				HidePieces (selectedConeOutlineSlot);
				ShowPieces (dashedConeOutlineSlot);
			} else if (lastPieceTypeSelected == Constants.FIN) {
				HidePieces (selectedFinOutlineSlots);
				ShowPieces (dashedFinOutlineSlots);
			}

			// hide the dashed outlines and show the selected outlines of the new selected piece type
			if (currentPieceTypeSelected == Constants.NONE_SELECTED) {

				// show all dashed pieces 
				ShowPieces (dashedBodyOutlineSlots);
				ShowPieces (dashedBoosterOutlineSlots);
				ShowPieces (dashedConeOutlineSlot);
				ShowPieces (dashedFinOutlineSlots);

				// hide all selected pieces
				HidePieces (selectedBodyOutlineSlots);
				HidePieces (selectedBoosterOutlineSlots);
				HidePieces (selectedConeOutlineSlot);
				HidePieces (selectedFinOutlineSlots);

				// hide all the body pieces
				HidePieces (bodyPanelRocketPieces);
				HidePieces (boosterPanelRocketPieces);
				HidePieces (conePanelRocketPieces);
				HidePieces (finPanelRocketPieces);

			} else if (currentPieceTypeSelected == Constants.BODY) {
				HidePieces (dashedBodyOutlineSlots);
				ShowPieces (selectedBodyOutlineSlots);

			} else if (currentPieceTypeSelected == Constants.BOOSTER) {
				HidePieces (dashedBoosterOutlineSlots);
				ShowPieces (selectedBoosterOutlineSlots);

			} else if (currentPieceTypeSelected == Constants.CONE) {
				HidePieces (dashedConeOutlineSlot);
				ShowPieces (selectedConeOutlineSlot);
			} else if (currentPieceTypeSelected == Constants.FIN) {
				HidePieces (dashedFinOutlineSlots);
				ShowPieces (selectedFinOutlineSlots);

			}

		}



		public void UpdatePerformanceMetrics (bool rocketComplete) {

			// update the counts of the number of pieces on both rockets
			UpdateRocketPieceCounts ();

			// if there weren't any pieces on the builder rocket, don't add anything
			// to the performance stats
			if (!rocketComplete && totalNumBuilderRocketPieces == 0) {
				return;
			}

			// performance count of # of correct pieces in the correct slots
			int numCorrectRocketPiecesLoc = 0;

			// body pieces
			for (int i = 0; i < explainerRocket.bodyPieces.Count; i++) {
				if (explainerRocket.bodyPieces [i] != null && builderRocket.bodyPieces [i] != null) {
					if (explainerRocket.bodyPieces [i].name == builderRocket.bodyPieces [i].name) {
						numCorrectRocketPiecesLoc += 1;
					}
				}
			}
			// booster pieces
			for (int i = 0; i < explainerRocket.boosterPieces.Count; i++) {
				if (explainerRocket.boosterPieces [i] != null && builderRocket.boosterPieces [i] != null) {
					if (explainerRocket.boosterPieces [i].name == builderRocket.boosterPieces [i].name) {
						numCorrectRocketPiecesLoc += 1;
					}
				}
			}
			// cone piece
			for (int i = 0; i < explainerRocket.conePieces.Count; i++) {
				if (explainerRocket.conePieces [i] != null && builderRocket.conePieces [i] != null) {
					if (explainerRocket.conePieces [i].name == builderRocket.conePieces [i].name) {
						numCorrectRocketPiecesLoc += 1;
					}
				}
			}
			// fin pieces
			for (int i = 0; i < explainerRocket.finPieces.Count; i++) {
				if (explainerRocket.finPieces [i] != null && builderRocket.finPieces [i] != null) {
					if (explainerRocket.finPieces [i].name == builderRocket.finPieces [i].name) {
						numCorrectRocketPiecesLoc += 1;
					}
				}
			}

			// add metric of # of correct pieces in the correct places to the performanceMetrics global variable
			float proportionCorrectRocketPiecesLoc = (float)numCorrectRocketPiecesLoc / (float)totalNumBuilderRocketPieces;
			if (childExplainer) {
				performanceMetrics [Constants.CHILD_EXPLAINER_LOCATION_ACCURACY].Add (proportionCorrectRocketPiecesLoc);
			} else {
				performanceMetrics [Constants.CHILD_BUILDER_LOCATION_ACCURACY].Add (proportionCorrectRocketPiecesLoc);
			}

			// temp lists
			List<string> builderBodyPieceNames = new List<string> ();
			List<string> builderBoosterPieceNames = new List<string> ();
			List<string> builderConePieceNames = new List<string> ();
			List<string> builderFinPieceNames = new List<string> ();

			// populate them
			for (int i = 0; i < builderRocket.bodyPieces.Count; i++) {
				if (builderRocket.bodyPieces [i] != null) {
					builderBodyPieceNames.Add (builderRocket.bodyPieces [i].name);
				}
			}
			for (int i = 0; i < builderRocket.boosterPieces.Count; i++) {
				if (builderRocket.boosterPieces [i] != null) {
					builderBoosterPieceNames.Add (builderRocket.boosterPieces [i].name);
				}
			}
			for (int i = 0; i < builderRocket.conePieces.Count; i++) {
				if (builderRocket.conePieces [i] != null) {
					builderConePieceNames.Add (builderRocket.conePieces [i].name);
				}
			}
			for (int i = 0; i < builderRocket.finPieces.Count; i++) {
				if (builderRocket.finPieces [i] != null) {
					builderFinPieceNames.Add (builderRocket.finPieces [i].name);
				}
			}

			// remove items in each builder string piece list for every piece in the explainer rocket 
			// body pieces
			int tempIndex = -1;
			for (int i = 0; i < explainerRocket.bodyPieces.Count; i++) {
				tempIndex = builderBodyPieceNames.FindIndex (searchString => searchString == explainerRocket.bodyPieces [i].name);
				if (tempIndex != -1) {
					builderBodyPieceNames.RemoveAt (tempIndex);
					tempIndex = -1;
				}
			}
			// booster pieces
			for (int i = 0; i < explainerRocket.boosterPieces.Count; i++) {
				tempIndex = builderBoosterPieceNames.FindIndex (searchString => searchString == explainerRocket.boosterPieces [i].name);
				if (tempIndex != -1) {
					builderBoosterPieceNames.RemoveAt (tempIndex);
					tempIndex = -1;
				}
			}
			// cone piece
			for (int i = 0; i < explainerRocket.conePieces.Count; i++) {
				tempIndex = builderConePieceNames.FindIndex (searchString => searchString == explainerRocket.conePieces [i].name);
				if (tempIndex != -1) {
					builderConePieceNames.RemoveAt (tempIndex);
					tempIndex = -1;
				}
			}
			// fin pieces
			for (int i = 0; i < explainerRocket.finPieces.Count; i++) {
				tempIndex = builderFinPieceNames.FindIndex (searchString => searchString == explainerRocket.finPieces [i].name);
				if (tempIndex != -1) {
					builderFinPieceNames.RemoveAt (tempIndex);
					tempIndex = -1;
				}
			}

			int remainingPieces = builderBodyPieceNames.Count + builderBoosterPieceNames.Count + builderConePieceNames.Count + builderFinPieceNames.Count;
			float proportionCorrectRocketPiecesNotLoc = ((float)totalNumBuilderRocketPieces - (float)remainingPieces) / (float)totalNumBuilderRocketPieces;

			// add metric of # of correct pieces in any location to the performanceMetrics global variable
			if (childExplainer) {
				performanceMetrics [Constants.CHILD_EXPLAINER_CHOICE_ACCURACY].Add (proportionCorrectRocketPiecesNotLoc);
			} else {
				performanceMetrics [Constants.CHILD_BUILDER_CHOICE_ACCURACY].Add (proportionCorrectRocketPiecesNotLoc);
			}

			//			string tempStr = "";
			//			foreach (KeyValuePair<int,List<float>> performanceMetricEntry in performanceMetrics) {
			//				tempStr = "";
			//				foreach (float performanceMetric in performanceMetricEntry.Value) {
			//					tempStr += performanceMetric.ToString () + ", ";
			//				}
			//				Debug.Log ("Performance metric type " + performanceMetricEntry.Key + ": " + tempStr);
			//			}

		}
	}
}