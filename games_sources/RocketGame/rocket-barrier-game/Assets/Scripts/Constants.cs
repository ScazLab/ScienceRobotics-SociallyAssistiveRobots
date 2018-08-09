using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace RocketBarrierGame {

	/* Information that makes levels different from one another */
	public class LevelInfo {
		
		public int numBodyOutlines;
		public int numBoosterOutlines;
		public int numPieceOptions;
		public List<string> bodyPieceOptionNames;
		public List<string> boosterPieceOptionNames;
		public List<string> conePieceOptionNames;
		public List<string> finPieceOptionNames;

		public LevelInfo(int newNumBodyOutlines, int newNumBoosterOutlines, int newNumPieceOptions, List<string> newBodyPieceOptionNames, 
			List<string> newBoosterPieceOptionNames, List<string> newConePieceOptionNames, List<string> newFinPieceOptionNames) {
			numBodyOutlines = newNumBodyOutlines;
			numBoosterOutlines = newNumBoosterOutlines;
			numPieceOptions = newNumPieceOptions;
			bodyPieceOptionNames = newBodyPieceOptionNames;
			boosterPieceOptionNames = newBoosterPieceOptionNames;
			conePieceOptionNames = newConePieceOptionNames;
			finPieceOptionNames = newFinPieceOptionNames;
		}
	}

	public struct GameConfig
	{
		public string server;
		public string port; 
		public bool logDebugToROS;
	}

	
	public static class Constants {

		/* Different types of rocket pieces */
		public const int NONE_SELECTED = 0;
		public const int BODY = 1;
		public const int FIN = 2;
		public const int BOOSTER = 3;
		public const int CONE = 4;

		/* Scenes */
		public const int BUILDER_SCENE = 0;
		public const int EXPLAINER_SCENE = 1;
		public const int END_GAME_COMPARISON_SCENE = 2;
		public const int START_SCENE = 4;
		public const int CHOOSE_PLAY_OR_TUTORIAL_SCENE = 5;

		/* Types of parents */
		public const int PARENT_PIECE_PANEL = 0;
		public const int PARENT_ROCKET_OUTLINE = 1;

		/* Explainer modes */
		public const int EXPLAINER_BUILDING = 0;
		public const int EXPLAINER_EXPLAINING = 1;

		/* Builder modes */
		public const int BUILDER_FIRST_BUILDING = 0;
		public const int BUILDER_CONTINUED_BUILDING = 1;

		/* Tutorial modes */
		public const int TUTORIAL_ON = 0;
		public const int TUTORIAL_OFF = 1;

		/* Tutorial button texts */
		public const string TUTORIAL_ON_BUTTON_TEXT = "Exit Tutorial";
		public const string TUTORIAL_OFF_BUTTON_TEXT = "Start Tutorial";

		/* Tutorial steps */
		public const int TUTORIAL_STATE_NONE = -1;
		public const int TUTORIAL_STATE_TOUCH_BODY_FRAME = 0;
		public const int TUTORIAL_STATE_PUT_ON_VENT = 1;
		public const int TUTORIAL_STATE_TRASH_VENT = 2;
		public const int TUTORIAL_STATE_FUEL_BODY_PIECES = 3;
		public const int TUTORIAL_STATE_TOUCH_BOOSTER_FRAME = 4;
		public const int TUTORIAL_STATE_BOOSTER_PIECES = 5;
		public const int TUTORIAL_STATE_TOUCH_FIN_FRAME = 6;
		public const int TUTORIAL_STATE_FIN_PIECES = 7;
		public const int TUTORIAL_STATE_TOUCH_CONE_FRAME = 8;
		public const int TUTORIAL_STATE_CONE_PIECE = 9;
		public const int TUTORIAL_STATE_FINISH_BUILDING_EXPLAINER = 10;
		public const int TUTORIAL_STATE_BEGIN_BUILDER = 11;
		public const int TUTORIAL_STATE_END = 12;

		/* Child/caregiver labels for builder/explainer scenes */
		public const string CHILD_ROCKET_STRING = "Child's Rocket";
		public const string CAREGIVER_ROCKET_STRING = "Caregiver's Rocket";

		/* Internal Unity game state variables */
		public const int WAITING_FOR_GAME_START = 0;
		public const int RECEIVED_GAME_START = 1;
		public const int WAITING_FOR_TUTORIAL_OR_PLAY_CHOICE = 2;
		public const int START_GAME = 3;
		public const int STARTED_GAME = 4;
		public const int END_GAME = 5;
		public const int GRACEFUL_EXIT = 6;
		public const int QUIT_GAME = 7;

		/* Variables related to the result type */
		public const int CHILD_EXPLAINER_LOCATION_ACCURACY = 0;
		public const int CHILD_EXPLAINER_CHOICE_ACCURACY = 1;
		public const int CHILD_BUILDER_LOCATION_ACCURACY = 2;
		public const int CHILD_BUILDER_CHOICE_ACCURACY = 3;

		/* Rocket piece colors */
		public const int RED = 0;
		public const int YELLOW = 1;
		public const int GREEN = 2;
		public const int BLUE = 3;
		public const int PURPLE = 4;
		public const int BROWN = 5;
		public const int GREY = 6;
		public const int WHITE = 7;

		/* Booster piece flame colors */
		public const int NO_FLAME = -1;
		public const int RED_FLAME = 0;
		public const int YELLOW_FLAME = 1;
		public const int BLUE_FLAME = 2;

		/* Rocket piece shapes */
		public const int SHAPE_BODY_METAL = 0;
		public const int SHAPE_BODY_FUEL = 1;
		public const int SHAPE_BODY_ARM = 2;
		public const int SHAPE_BODY_BALLOON = 3;
		public const int SHAPE_BODY_SOLAR = 4;
		public const int SHAPE_BODY_VENT = 5;
		public const int SHAPE_BODY_PERSON = 6;
		public const int SHAPE_BODY_HAMSTER = 7;

		public const int SHAPE_BOOSTER_NORMAL_FIN = 0;
		public const int SHAPE_BOOSTER_NORMAL = 1;
		public const int SHAPE_BOOSTER_CYLINDER = 2;
		public const int SHAPE_BOOSTER_BALLOON = 3;
		public const int SHAPE_BOOSTER_DYNAMITE = 4;
		public const int SHAPE_BOOSTER_SOLAR = 5;

		public const int SHAPE_CONE_TALL_TRIANGLE = 0;
		public const int SHAPE_CONE_SMALL_TRIANGLE = 1;
		public const int SHAPE_CONE_ROUND = 2;
		public const int SHAPE_CONE_SLOPED_TRIANGLE = 3;

		public const int SHAPE_FIN_TRIANGLE = 0;
		public const int SHAPE_FIN_ANGLED_WING = 1;
		public const int SHAPE_FIN_ROUNDED_TRIANGLE = 2;
		public const int SHAPE_FIN_FIRE = 3;
		public const int SHAPE_FIN_SHORT = 4;
		public const int SHAPE_FIN_ANGLED_WRONG_WAY = 5;

		/* Graceful exit options */
		public const int GRACEFUL_EXIT_EXPLAINER_FINISH = 0;
		public const int GRACEFUL_EXIT_BUILDER_FINISH_HALFWAY = 1;
		public const int GRACEFUL_EXIT_BUILDER_FINISH_COMPLETELY = 2;
		public const int GRACEFUL_EXIT_END_OF_ROUND = 3;

		/* Level specific information */
		public static readonly Dictionary<int, LevelInfo> LEVEL_INFO =  new Dictionary<int, LevelInfo>{
			{1, new LevelInfo(4, 2, 2, 
				new List<string> (new string[] {"body_Chrome", "body_Solar", "body_Vent", "body_MTank", "body_Window", "body_Hamster"}), 
				new List<string> (new string[] {"engine_Fin_Yellow", "engine_Fin_Blue", "engine_Cylinder_Red", "engine_Solar"}), 
				new List<string> (new string[] {"cone_Isosceles", "cone_Concave"}), 
				new List<string> (new string[] {"fin_Fin", "fin_Shark", "fin_XMas", "fin_Skinny"}))}, 
			{2, new LevelInfo(6, 3, 4, 
				new List<string> (new string[] {"body_Chrome", "body_Solar", "body_Vent", "body_MTank", "body_Window", "body_Hamster", "body_Copper", "body_Balloon"}), 
				new List<string> (new string[] {"engine_Fin_Yellow", "engine_Fin_Blue", "engine_Cylinder_Red", "engine_Solar", "engine_Cylinder_Yellow", "engine_Balloon"}), 
				new List<string> (new string[] {"cone_Isosceles", "cone_Concave", "cone_Onion", "cone_Round"}), 
				new List<string> (new string[] {"fin_Fin", "fin_Shark", "fin_XMas", "fin_Skinny", "fin_Engine", "fin_Solar"}))}, 
			{3, new LevelInfo(9, 3, 6, 
				new List<string> (new string[] {"body_Chrome", "body_Solar", "body_Vent", "body_MTank", "body_Window", "body_Hamster", "body_Copper", "body_Balloon", "body_STank", "body_Arm"}), 
				new List<string> (new string[] {"engine_Fin_Yellow", "engine_Fin_Blue", "engine_Cylinder_Red", "engine_Solar", "engine_Cylinder_Yellow", "engine_Balloon", "engine_Fin_Red", "engine_Dynamite"}), 
				new List<string> (new string[] {"cone_Isosceles", "cone_Concave", "cone_Onion", "cone_Round", "cone_Solar", "cone_Hollow"}), 
				new List<string> (new string[] {"fin_Fin", "fin_Shark", "fin_XMas", "fin_Skinny", "fin_Engine", "fin_Solar", "fin_Double", "fin_Hollow"}))}, 
			{4, new LevelInfo(12, 4, 8, 
				new List<string> (new string[] {"body_Chrome", "body_Solar", "body_Vent", "body_MTank", "body_Window", "body_Hamster", "body_Copper", "body_Balloon", "body_STank", "body_Arm", "body_Platinum", "body_LTank"}), 
				new List<string> (new string[] {"engine_Fin_Yellow", "engine_Fin_Blue", "engine_Cylinder_Red", "engine_Solar", "engine_Cylinder_Yellow", "engine_Balloon", "engine_Fin_Red", "engine_Dynamite", "engine_Cylinder_Blue", "engine_Normal_Red"}), 
				new List<string> (new string[] {"cone_Isosceles", "cone_Concave", "cone_Onion", "cone_Round", "cone_Solar", "cone_Hollow", "cone_Rod", "cone_Equilateral"}), 
				new List<string> (new string[] {"fin_Fin", "fin_Shark", "fin_XMas", "fin_Skinny", "fin_Engine", "fin_Solar", "fin_Double", "fin_Hollow", "fin_Reverse", "fin_Wing"}))}
		};

		/* SAR game identity */
		public const int SAR_GAME_IDENTITY = 1;

		/* config file name and paths */
		public const string CONFIG_FILE = "yurp_config.txt";
		public const string CONFIG_PATH_OSX = @"/Resources/";
		public const string CONFIG_PATH_LINUX = "/Resources/";

		/* robot utterances name and paths */
		public const string ROBOT_UTTERANCES_FILE = "robot_behavior_rocket_barrier_game.json";
		public const string ROBOT_UTTERANCES_PATH_OSX = @"/Resources/";
		public const string ROBOT_UTTERANCES_PATH_LINUX = "/Resources/";

		/* ROS-related constants: topics and message types */
		public const string LOG_ROSTOPIC = "/yurp_log";
		public const string LOG_ROSMSG_TYPE = "std_msgs/String";

		public const string CMD_ROSTOPIC = "/sar/game_command";
		public const string CMD_ROSMSG_TYPE = "sar_game_command_msgs/GameCommand";

		public const string STATE_ROSTOPIC = "/sar/game_state";
		public const string STATE_ROSMSG_TYPE = "sar_game_command_msgs/GameState";

		public const string ROBOT_CMD_ROSTOPIC = "/sar/robot_command";
		public const string ROBOT_CMD_ROSMSG_TYPE = "sar_robot_command_msgs/RobotCommand";

		/* ROS game command types */
		public const int GAME_COMMAND_START = 0;
		public const int GAME_COMMAND_CONTINUE = 1;
		public const int GAME_COMMAND_PAUSE = 2;
		public const int GAME_COMMAND_END = 3;
		public const int GAME_COMMAND_WAIT_FOR_RESPONSE = 4;
		public const int GAME_COMMAND_SKIP_RESPONSE = 5;

		/* ROS game state states */
		public const int GAME_STATE_START = 0;
		public const int GAME_STATE_IN_PROGRESS = 1;
		public const int GAME_STATE_PAUSED = 2;
		public const int GAME_STATE_USER_TIMEOUT = 3;
		public const int GAME_STATE_END = 4;
		public const int GAME_STATE_READY = 5;
		public const int GAME_STATE_TUTORIAL = 6;

		/* ROS robot command message command types */
		public const int ROBOT_COMMAND_SLEEP = 0;
		public const int ROBOT_COMMAND_WAKEUP = 1;
		public const int ROBOT_COMMAND_DO = 2;

	}

}