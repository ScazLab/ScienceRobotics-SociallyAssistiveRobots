using System.Collections.Generic;
using System.Collections;
using System.IO;
using System;
using MiniJSON;
using UnityEngine;

namespace HouseGame
{

    public static class Utilities
    {

        public static bool ParseConfig(string path, out GameConfig gameConfig)
        {
            gameConfig.server = "";
            gameConfig.port = "";
            gameConfig.logDebugToROS = false;

            string config = "";
            try
            {
                config = File.ReadAllText(path);
                Logger.Log("got config: " + config);
                config.Replace("\n", "");

                Dictionary<string, object> data = null;
                data = Json.Deserialize(config) as Dictionary<string, object>;
                if (data == null)
                {
                    Logger.LogError("Could not parse JSON config file!");
                    return false;
                }
                Logger.Log("deserialized " + data.Count + " objects from JSON!");

                // if the config file doesn't have all parts, consider it invalid
                if (!(data.ContainsKey("server") && data.ContainsKey("port") && data.ContainsKey("log_debug_to_ros")))
                {
                    Logger.LogError("Did not get a valid config file!");
                    return false;
                }

                // get configuration options
                gameConfig.server = (string)data["server"];
                gameConfig.port = (string)data["port"];
                gameConfig.logDebugToROS = (bool)data["log_debug_to_ros"];

                Logger.Log("server: " + gameConfig.server + "  port: " + gameConfig.port
                    + "  log_debug_to_ros: " + gameConfig.logDebugToROS);
                return true;

            }
            catch (Exception e)
            {
                Logger.LogError("Could not read config file! File path given was "
                    + path + "\nError: " + e);
                return false;
            }

        }

        public static bool ParseRobotUtterancesFile(string path, out Dictionary<string, List<List<string>>> robotUtterances)
        {

            Dictionary<string, object> tempRobotUtterances = new Dictionary<string, object>();
            robotUtterances = new Dictionary<string, List<List<string>>>();

            string robotUtterancesStringContents = "";
            try
            {
                robotUtterancesStringContents = File.ReadAllText(path);
                robotUtterancesStringContents.Replace("\n", "");

                tempRobotUtterances = Json.Deserialize(robotUtterancesStringContents) as Dictionary<string, object>;
                if (tempRobotUtterances == null)
                {
                    Logger.LogError("Could not parse JSON robot utterances file!");
                    return false;
                }

                List<List<string>> utterancesInCategory = new List<List<string>>();
                List<string> singleUtteranceToAdd = new List<string>();
                foreach (KeyValuePair<string, object> tempRobotUtterancesEntry in tempRobotUtterances)
                {
                    utterancesInCategory = new List<List<string>>();
                    foreach (object tempRobotUtterancesValueListItem in (List<object>)tempRobotUtterancesEntry.Value)
                    {
                        singleUtteranceToAdd = new List<string>();
                        foreach (object tempRobotUtteranceSegment in (List<object>)tempRobotUtterancesValueListItem)
                        {
                            singleUtteranceToAdd.Add((string)tempRobotUtteranceSegment);
                        }
                        utterancesInCategory.Add(singleUtteranceToAdd);
                    }
                    robotUtterances.Add(tempRobotUtterancesEntry.Key, utterancesInCategory);
                }

                Logger.Log("deserialized " + robotUtterances.Count + " objects from JSON!");
                return true;
            }
            catch (Exception e)
            {
                Logger.LogError("Could not read robot utterances file! File path given was " + path + "\nError: " + e);
                return false;
            }

        }

        public static string PrepareUtteranceForSending(string utteranceString, bool childExplainer)
        {

            string preparedString = String.Copy(utteranceString);

            // for the active player
            GameObject gameManager = GameObject.Find("GameManager");
            if (gameManager != null)
            {
                if (gameManager.GetComponent<MainGameController>().currentScene == Constants.BUILDER_SCENE)
                {
                    preparedString = preparedString.Replace("{active-player-name}", "{builder-name}");
                }
                else
                {
                    preparedString = preparedString.Replace("{active-player-name}", "{explainer-name}");
                }
            }

            // for role specific substitutions
            if (childExplainer)
            {
                // explainer 
                preparedString = preparedString.Replace("{explainer-name}", "[child-name]");
                preparedString = preparedString.Replace("{look-at-explainer}", "<lookat_child>");
                preparedString = preparedString.Replace("{look-at-explainer, nb}", "<lookat_child,nb>");
                preparedString = preparedString.Replace("{look-at-explainer,nb}", "<lookat_child,nb>");
                preparedString = preparedString.Replace("{look-at-explainer, b}", "<lookat_child,b>");
                preparedString = preparedString.Replace("{look-at-explainer,b}", "<lookat_child,b>");
                // builder
                preparedString = preparedString.Replace("{builder-name}", "[guardian-name]");
                preparedString = preparedString.Replace("{look-at-builder}", "<lookat_guardian>");
                preparedString = preparedString.Replace("{look-at-builder, nb}", "<lookat_guardian,nb>");
                preparedString = preparedString.Replace("{look-at-builder,nb}", "<lookat_guardian,nb>");
                preparedString = preparedString.Replace("{look-at-builder, b}", "<lookat_guardian,b>");
                preparedString = preparedString.Replace("{look-at-builder,b}", "<lookat_guardian,b>");
            }
            else
            {
                // explainer 
                preparedString = preparedString.Replace("{explainer-name}", "[guardian-name]");
                preparedString = preparedString.Replace("{look-at-explainer}", "<lookat_guardian>");
                preparedString = preparedString.Replace("{look-at-explainer, nb}", "<lookat_guardian,nb>");
                preparedString = preparedString.Replace("{look-at-explainer,nb}", "<lookat_guardian,nb>");
                preparedString = preparedString.Replace("{look-at-explainer, b}", "<lookat_guardian,b>");
                preparedString = preparedString.Replace("{look-at-explainer,b}", "<lookat_guardian,b>");
                // builder
                preparedString = preparedString.Replace("{builder-name}", "[child-name]");
                preparedString = preparedString.Replace("{look-at-builder}", "<lookat_child>");
                preparedString = preparedString.Replace("{look-at-builder, nb}", "<lookat_child,nb>");
                preparedString = preparedString.Replace("{look-at-builder,nb}", "<lookat_child,nb>");
                preparedString = preparedString.Replace("{look-at-builder, b}", "<lookat_child,b>");
                preparedString = preparedString.Replace("{look-at-builder,b}", "<lookat_child,b>");
            }

            return preparedString;
        }

        public static string replaceTypeUtterances(string utteranceString, int pieceType)
        {
            string preparedString = String.Copy(utteranceString);

            if (pieceType == Constants.WALL)
            {
                preparedString = preparedString.Replace("{piece-type}", "wall pieces");
            }
            if (pieceType == Constants.DOOR)
            {
                preparedString = preparedString.Replace("{piece-type}", "door pieces");
            }
            if (pieceType == Constants.PLANT)
            {
                preparedString = preparedString.Replace("{piece-type}", "plant pieces");
            }
            if (pieceType == Constants.ROOF)
            {
                preparedString = preparedString.Replace("{piece-type}", "roof pieces");
            }
            if (pieceType == Constants.RIGHT_DOOR)
            {
                preparedString = preparedString.Replace("{piece-type}", "right door piece");
            }
            if (pieceType == Constants.LEFT_DOOR)
            {
                preparedString = preparedString.Replace("{piece-type}", "left door piece");
            }
            if (pieceType == Constants.RIGHT_PLANT)
            {
                preparedString = preparedString.Replace("{piece-type}", "right plant piece");
            }
            if (pieceType == Constants.LEFT_PLANT)
            {
                preparedString = preparedString.Replace("{piece-type}", "left plant piece");
            }
            return preparedString;
        }

        public static string replaceColorUtterances(string utteranceString, int color)
        {
            string preparedString = String.Copy(utteranceString);

            if (color == Constants.RED)
            {
                preparedString = preparedString.Replace("{color}", "red");
            }

            if (color == Constants.GREEN)
            {
                preparedString = preparedString.Replace("{color}", "dark green or light green");
            }

            if (color == Constants.BLUE)
            {
                preparedString = preparedString.Replace("{color}", "dark blue or light blue");
            }

            if (color == Constants.WOOD)
            {
                preparedString = preparedString.Replace("{color}", "light wooden");
            }

            if (color == Constants.BRICK)
            {
                preparedString = preparedString.Replace("{color}", "brick");
            }
            return preparedString;
        }
        public static string replacePiecesUtterances(string utteranceString, int pieceType, int pieceId)
        {
            string preparedString = String.Copy(utteranceString);

            // wall pieces
            if ((pieceType == Constants.WALL) && (pieceId == Constants.TYPE_WALL_BLUE))
            {
                preparedString = preparedString.Replace("{piece-name}", "blue and grey walls");
            }
            if ((pieceType == Constants.WALL) && (pieceId == Constants.TYPE_WALL_BRICK))
            {
                preparedString = preparedString.Replace("{piece-name}", "orange brick walls");
            }
            if ((pieceType == Constants.WALL) && (pieceId == Constants.TYPE_WALL_LEGO))
            {
                preparedString = preparedString.Replace("{piece-name}", "lego brick walls");
            }
            if ((pieceType == Constants.WALL) && (pieceId == Constants.TYPE_WALL_STRAW))
            {
                preparedString = preparedString.Replace("{piece-name}", "straw walls");
            }
            if ((pieceType == Constants.WALL) && (pieceId == Constants.TYPE_WALL_WINDOW))
            {
                preparedString = preparedString.Replace("{piece-name}", "windows on all the walls");
            }
            if ((pieceType == Constants.WALL) && (pieceId == Constants.TYPE_WALL_WOOD))
            {
                preparedString = preparedString.Replace("{piece-name}", "wooden walls");
            }


            // roof pieces
            if ((pieceType == Constants.ROOF) && (pieceId == Constants.TYPE_ROOF_PANEL))
            {
                preparedString = preparedString.Replace("{piece-name}", "blue panel roof");
            }
            if ((pieceType == Constants.ROOF) && (pieceId == Constants.TYPE_ROOF_PALMTREE))
            {
                preparedString = preparedString.Replace("{piece-name}", "palmtree roof");
            }
            if ((pieceType == Constants.ROOF) && (pieceId == Constants.TYPE_ROOF_PLAIN))
            {
                preparedString = preparedString.Replace("{piece-name}", "plain green roof");
            }
            if ((pieceType == Constants.ROOF) && (pieceId == Constants.TYPE_ROOF_ROUNDED))
            {
                preparedString = preparedString.Replace("{piece-name}", "red roof");
            }
            if ((pieceType == Constants.ROOF) && (pieceId == Constants.TYPE_ROOF_TYPICAL))
            {
                preparedString = preparedString.Replace("{piece-name}", "blocked grey roof");
            }
            if ((pieceType == Constants.ROOF) && (pieceId == Constants.TYPE_ROOF_GARDEN))
            {
                preparedString = preparedString.Replace("{piece-name}", "garden with flowers for a roof");
            }

            // plant pieces
            if ((pieceType == Constants.PLANT) && (pieceId == Constants.TYPE_PLANT_BIRD))
            {
                preparedString = preparedString.Replace("{piece-name}", "fountain with a bird");
            }
            if ((pieceType == Constants.PLANT) && (pieceId == Constants.TYPE_PLANT_FLOWER))
            {
                preparedString = preparedString.Replace("{piece-name}", "bush with flowers");
            }
            if ((pieceType == Constants.PLANT) && (pieceId == Constants.TYPE_PLANT_TREE))
            {
                preparedString = preparedString.Replace("{piece-name}", "tree");
            }
            if ((pieceType == Constants.PLANT) && (pieceId == Constants.TYPE_PLANT_SLIDE))
            {
                preparedString = preparedString.Replace("{piece-name}", "slide");
            }
            if ((pieceType == Constants.PLANT) && (pieceId == Constants.TYPE_PLANT_TREEHOUSE))
            {
                preparedString = preparedString.Replace("{piece-name}", "treehouse");
            }
            if ((pieceType == Constants.PLANT) && (pieceId == Constants.TYPE_PLANT_PLAIN))
            {
                preparedString = preparedString.Replace("{piece-name}", "bush");
            }



            //door pieces
            if ((pieceType == Constants.DOOR) && (pieceId == Constants.TYPE_DOOR_GLASS))
            {
                preparedString = preparedString.Replace("{piece-name}", "light blue door");
            }
            if ((pieceType == Constants.DOOR) && (pieceId == Constants.TYPE_DOOR_PLAIN))
            {
                preparedString = preparedString.Replace("{piece-name}", "yellow door");
            }
            if ((pieceType == Constants.DOOR) && (pieceId == Constants.TYPE_DOOR_RECTANGELS))
            {
                preparedString = preparedString.Replace("{piece-name}", "red door");
            }
            if ((pieceType == Constants.DOOR) && (pieceId == Constants.TYPE_DOOR_SHUTTERS))
            {
                preparedString = preparedString.Replace("{piece-name}", "white door");
            }
            if ((pieceType == Constants.DOOR) && (pieceId == Constants.TYPE_DOOR_TWO))
            {
                preparedString = preparedString.Replace("{piece-name}", "wooden door");
            }
            if ((pieceType == Constants.DOOR) && (pieceId == Constants.TYPE_DOOR_WINDOW))
            {
                preparedString = preparedString.Replace("{piece-name}", "dark blue door");
            }


            // right door pieces
            if ((pieceType == Constants.RIGHT_DOOR) && (pieceId == Constants.TYPE_DOOR_GLASS))
            {
                preparedString = preparedString.Replace("{piece-name}", "light blue door on the right");
            }
            if ((pieceType == Constants.RIGHT_DOOR) && (pieceId == Constants.TYPE_DOOR_PLAIN))
            {
                preparedString = preparedString.Replace("{piece-name}", "yellow door on the right");
            }
            if ((pieceType == Constants.RIGHT_DOOR) && (pieceId == Constants.TYPE_DOOR_RECTANGELS))
            {
                preparedString = preparedString.Replace("{piece-name}", "red door on the right");
            }
            if ((pieceType == Constants.RIGHT_DOOR) && (pieceId == Constants.TYPE_DOOR_SHUTTERS))
            {
                preparedString = preparedString.Replace("{piece-name}", "white door on the right");
            }
            if ((pieceType == Constants.RIGHT_DOOR) && (pieceId == Constants.TYPE_DOOR_TWO))
            {
                preparedString = preparedString.Replace("{piece-name}", "wooden door on the right");
            }
            if ((pieceType == Constants.RIGHT_DOOR) && (pieceId == Constants.TYPE_DOOR_WINDOW))
            {
                preparedString = preparedString.Replace("{piece-name}", "dark blue door on the right");
            }

            if ((pieceType == Constants.LEFT_DOOR) && (pieceId == Constants.TYPE_DOOR_GLASS))
            {
                preparedString = preparedString.Replace("{piece-name}", "light blue door on the left");
            }
            if ((pieceType == Constants.LEFT_DOOR) && (pieceId == Constants.TYPE_DOOR_PLAIN))
            {
                preparedString = preparedString.Replace("{piece-name}", "yellow door on the left");
            }
            if ((pieceType == Constants.LEFT_DOOR) && (pieceId == Constants.TYPE_DOOR_RECTANGELS))
            {
                preparedString = preparedString.Replace("{piece-name}", "red door on the left");
            }
            if ((pieceType == Constants.LEFT_DOOR) && (pieceId == Constants.TYPE_DOOR_SHUTTERS))
            {
                preparedString = preparedString.Replace("{piece-name}", "left white door");
            }
            if ((pieceType == Constants.LEFT_DOOR) && (pieceId == Constants.TYPE_DOOR_TWO))
            {
                preparedString = preparedString.Replace("{piece-name}", "left wooden door");
            }
            if ((pieceType == Constants.LEFT_DOOR) && (pieceId == Constants.TYPE_DOOR_WINDOW))
            {
                preparedString = preparedString.Replace("{piece-name}", "left dark blue door");
            }

            // right plant pieces
            if ((pieceType == Constants.RIGHT_PLANT) && (pieceId == Constants.TYPE_PLANT_BIRD))
            {
                preparedString = preparedString.Replace("{piece-name}", "fountain with a bird on the right");
            }
            if ((pieceType == Constants.RIGHT_PLANT) && (pieceId == Constants.TYPE_PLANT_FLOWER))
            {
                preparedString = preparedString.Replace("{piece-name}", "flower bush on the right");
            }
            if ((pieceType == Constants.RIGHT_PLANT) && (pieceId == Constants.TYPE_PLANT_TREE))
            {
                preparedString = preparedString.Replace("{piece-name}", "tree on the right");
            }
            if ((pieceType == Constants.RIGHT_PLANT) && (pieceId == Constants.TYPE_PLANT_SLIDE))
            {
                preparedString = preparedString.Replace("{piece-name}", "slide on the right");
            }
            if ((pieceType == Constants.RIGHT_PLANT) && (pieceId == Constants.TYPE_PLANT_TREEHOUSE))
            {
                preparedString = preparedString.Replace("{piece-name}", "treehouse on the right");
            }
            if ((pieceType == Constants.RIGHT_PLANT) && (pieceId == Constants.TYPE_PLANT_PLAIN))
            {
                preparedString = preparedString.Replace("{piece-name}", "bush on the right");
            }

            // left plant pieces
            if ((pieceType == Constants.LEFT_PLANT) && (pieceId == Constants.TYPE_PLANT_BIRD))
            {
                preparedString = preparedString.Replace("{piece-name}", "bird on a fountain on the left");
            }
            if ((pieceType == Constants.LEFT_PLANT) && (pieceId == Constants.TYPE_PLANT_FLOWER))
            {
                preparedString = preparedString.Replace("{piece-name}", "flower bush on the left");
            }
            if ((pieceType == Constants.LEFT_PLANT) && (pieceId == Constants.TYPE_PLANT_TREE))
            {
                preparedString = preparedString.Replace("{piece-name}", "tree on the left");
            }
            if ((pieceType == Constants.LEFT_PLANT) && (pieceId == Constants.TYPE_PLANT_SLIDE))
            {
                preparedString = preparedString.Replace("{piece-name}", "slide on the left");
            }
            if ((pieceType == Constants.LEFT_PLANT) && (pieceId == Constants.TYPE_PLANT_TREEHOUSE))
            {
                preparedString = preparedString.Replace("{piece-name}", "treehouse on the left");
            }
            if ((pieceType == Constants.LEFT_PLANT) && (pieceId == Constants.TYPE_PLANT_PLAIN))
            {
                preparedString = preparedString.Replace("{piece-name}", "bush on the left");
            }
            return preparedString;

        }
    }
}
 
 