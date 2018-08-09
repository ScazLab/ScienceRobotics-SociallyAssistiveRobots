using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace TrainGame {

    /* Information that makes levels different from one another */
    public class LevelInfo
    {
        public List<string> frontPieceOptionNames;
        public List<string> bodyPieceOptionNames;
        public List<string> cabinPieceOptionNames;
        public List<string> backPieceOptionNames;
        public List<string> smokePieceOptionNames;
        public List<string> wheelPieceOptionNames;

        public LevelInfo(List<string> newFrontPieceOptionNames, List<string> newBodyPieceOptionNames, List<string> newCabinPieceOptionNames, 
                         List<string> newBackPieceOptionNames,  List<string> newSmokePieceOptionNames,List<string> newWheelPieceOptionNames)
        {
            frontPieceOptionNames = newFrontPieceOptionNames;
            bodyPieceOptionNames = newBodyPieceOptionNames;
            cabinPieceOptionNames = newCabinPieceOptionNames;
            backPieceOptionNames = newBackPieceOptionNames;
            smokePieceOptionNames = newSmokePieceOptionNames;
            wheelPieceOptionNames = newWheelPieceOptionNames;
        }
    }

    public struct GameConfig
    {
        public string server;
        public string port;
        public bool logDebugToROS;
    }


    public static class Constants {

        /* Different types of house pieces */
        public const int NONE_SELECTED = 0;
        public const int FRONT = 1;
        public const int BODY = 2;
        public const int CABIN = 3;
        public const int BACK = 4;
        public const int SMOKE = 5;
        public const int WHEEL = 6;
        public const int WHEEL1 = 7;
        public const int WHEEL2 = 8;
        public const int WHEEL3 = 9;
        public const int WHEEL4 = 10;
        public const int SMOKE1 = 11;
        public const int SMOKE2 = 12;
        public const int SMOKE3 = 13;

        /* Scenes */
        public const int BUILDER_SCENE = 0;
        public const int EXPLAINER_SCENE = 1;
        public const int END_GAME_COMPARISON_SCENE = 2;
        public const int START_SCENE = 4;
        public const int ROBOT_GUESSING_SCENE = 5;
        public const int CHILD_GUESSING_SCENE = 6;

        /* Types of parents */
        public const int PARENT_PIECE_PANEL = 0;
        public const int PARENT_TRAIN_OUTLINE = 1;

        /* Explainer modes */
        public const int EXPLAINER_BUILDING = 0;
        public const int EXPLAINER_EXPLAINING = 1;

        /* Builder modes */
        public const int BUILDER_FIRST_BUILDING = 0;
        public const int BUILDER_CONTINUED_BUILDING = 1;

        /* Internal Unity game state variables */
        public const int WAITING_FOR_GAME_START = 0;
        public const int START_GAME = 1;
        public const int STARTED_GAME = 2;
        public const int END_GAME = 3;
        public const int ENDED_GAME = 4;

        /* Variables related to the result type */
        public const int CHILD_EXPLAINER_LOCATION_ACCURACY = 0;
        public const int CHILD_EXPLAINER_CHOICE_ACCURACY = 1;
        public const int CHILD_BUILDER_LOCATION_ACCURACY = 2;
        public const int CHILD_BUILDER_CHOICE_ACCURACY = 3;

        /* House piece colors */
        /*public const int BLUE = 0;
        public const int RED = 1;
        public const int WOOD = 2;
        public const int BRICK = 3;
        public const int GREEN = 4;

		// HOUSE PIECE WALL TYPES
		public const int TYPE_WALL_NONE = -1;
		public const int TYPE_WALL_BLUE = 0;
		public const int TYPE_WALL_BRICK = 1;
		public const int TYPE_WALL_LEGO = 2;
		public const int TYPE_WALL_STRAW = 3;
		public const int TYPE_WALL_WINDOW = 4;
		public const int TYPE_WALL_WOOD = 5;

		// HOUSE ROOF TYPES
		public const int TYPE_ROOF_NONE = -1;
		public const int TYPE_ROOF_PANEL = 0;
		public const int TYPE_ROOF_PALMTREE = 1;
		public const int TYPE_ROOF_PLAIN = 2;
		public const int TYPE_ROOF_ROUNDED = 3;
		public const int TYPE_ROOF_TYPICAL = 4;
		public const int TYPE_ROOF_GARDEN = 5;

		// HOUSE DOOR TYPES
		public const int TYPE_DOOR_NONE = -1;
		public const int TYPE_DOOR_GLASS = 0;
		public const int TYPE_DOOR_PLAIN = 1;
		public const int TYPE_DOOR_RECTANGELS = 2;
		public const int TYPE_DOOR_SHUTTERS = 3;
		public const int TYPE_DOOR_TWO = 4;
		public const int TYPE_DOOR_WINDOW = 5;

		// HOUSE PLANT TYPES
		public const int TYPE_PLANT_NONE = -1;
		public const int TYPE_PLANT_BIRD = 0;
		public const int TYPE_PLANT_FLOWER = 1;
		public const int TYPE_PLANT_TREE = 2;
		public const int TYPE_PLANT_SLIDE = 3;
		public const int TYPE_PLANT_TREEHOUSE = 4;
		public const int TYPE_PLANT_PLAIN = 5;*/

        /* Level specific information */
        public static readonly Dictionary<int, LevelInfo> LEVEL_INFO = new Dictionary<int, LevelInfo>{
            {1, new LevelInfo(
                new List<string> (new string[] {"front_classic"}),
                new List<string> (new string[] {"body_black"}),
                new List<string> (new string[] {"cabin_modern"}),
                new List<string> (new string[] {"back_curved"}),
                new List<string> (new string[] {"smoke_one", "smoke_two"}),
                new List<string> (new string[] {"wheel_one", "wheel_two"}))},
            {2, new LevelInfo(
                new List<string> (new string[] {"front_classic"}),
                new List<string> (new string[] {"body_black"}),
                new List<string> (new string[] {"cabin_modern"}),
                new List<string> (new string[] {"back_curved"}),
                new List<string> (new string[] {"smoke_one", "smoke_two", "smoke_three", "smoke_four"}),
                new List<string> (new string[] {"wheel_one", "wheel_two", "wheel_three"}))},
            {3, new LevelInfo(

                new List<string> (new string[] {"front_classic", "front_simple"}),
                new List<string> (new string[] {"body_black", "body_metal"}),
                new List<string> (new string[] {"cabin_modern", "cabin_classic"}),
                new List<string> (new string[] {"back_curved", "back_flat"}),
                new List<string> (new string[] {"smoke_one", "smoke_two", "smoke_three", "smoke_four", "smoke_five"}),
                new List<string> (new string[] {"wheel_one", "wheel_two", "wheel_three", "wheel_concave"}))},
            {4, new LevelInfo(
				new List<string> (new string[] {"front_classic", "front_simple"}),
                new List<string> (new string[] {"body_black", "body_metal"}),
                new List<string> (new string[] {"cabin_modern", "cabin_classic"}),
                new List<string> (new string[] {"back_curved", "back_flat"}),
                new List<string> (new string[] {"smoke_one", "smoke_two", "smoke_three", "smoke_four", "smoke_five", "smoke_six", "smoke_seven"}),
                new List<string> (new string[] {"wheel_one", "wheel_two", "wheel_three", "wheel_concave", "wheel_convex"}))},
        };

		/* SAR game identity */
		public const int SAR_GAME_IDENTITY = 6;

		/* config file name and paths */
		public const string CONFIG_FILE = "yurp_config.txt";
		public const string CONFIG_PATH_OSX = @"/Resources/";
		public const string CONFIG_PATH_LINUX = "/Resources/";

		/* robot utterances name and paths */
		public const string ROBOT_UTTERANCES_FILE = "robot_behavior_train_game.json";
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
        public const int TUTORIAL = 6;

        /* ROS robot command message command types */
        public const int ROBOT_COMMAND_SLEEP = 0;
		public const int ROBOT_COMMAND_WAKEUP = 1;
		public const int ROBOT_COMMAND_DO = 2;

	}

}