using System.Collections.Generic;
using System.Collections;
using System.IO;
using System;
using MiniJSON;
using UnityEngine;

namespace TrainGame
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

        public static string replaceTypeUtterances(string utteranceString, int pieceType1, int pieceType2, int pieceType3, int pieceType4)
        {
            string preparedString = String.Copy(utteranceString);

            if (pieceType1 == Constants.FRONT)
            {
                preparedString = preparedString.Replace("{piece-type1}", "front piece");
            }
            if (pieceType1 == Constants.BODY)
            {
                preparedString = preparedString.Replace("{piece-type1}", "body piece");
            }
            if (pieceType1 == Constants.CABIN)
            {
                preparedString = preparedString.Replace("{piece-type1}", "cabin piece");
            }
            if (pieceType1 == Constants.BACK)
            {
                preparedString = preparedString.Replace("{piece-type1}", "back piece");
            }
            if (pieceType1 == Constants.WHEEL1)
            {
                preparedString = preparedString.Replace("{piece-type1}", "front wheel");
            }
            if (pieceType1 == Constants.WHEEL2)
            {
                preparedString = preparedString.Replace("{piece-type1}", "middle front wheel");
            }
            if (pieceType1 == Constants.WHEEL3)
            {
                preparedString = preparedString.Replace("{piece-type1}", "middle back wheel");
            }
            if (pieceType1 == Constants.WHEEL4)
            {
                preparedString = preparedString.Replace("{piece-type1}", "back wheel");
            }
            if (pieceType1 == Constants.SMOKE1)
            {
                preparedString = preparedString.Replace("{piece-type1}", "front chimney piece");
            }
            if (pieceType1 == Constants.SMOKE2)
            {
                preparedString = preparedString.Replace("{piece-type1}", "middle chimney piece");
            }
            if (pieceType1 == Constants.SMOKE3)
            {
                preparedString = preparedString.Replace("{piece-type1}", "back chimney piece");
            }

            if (pieceType2 == Constants.FRONT)
            {
                preparedString = preparedString.Replace("{piece-type2}", "front piece");
            }
            if (pieceType2 == Constants.BODY)
            {
                preparedString = preparedString.Replace("{piece-type2}", "body piece");
            }
            if (pieceType2 == Constants.CABIN)
            {
                preparedString = preparedString.Replace("{piece-type2}", "cabin piece");
            }
            if (pieceType2 == Constants.BACK)
            {
                preparedString = preparedString.Replace("{piece-type2}", "back piece");
            }
            if (pieceType2 == Constants.WHEEL1)
            {
                preparedString = preparedString.Replace("{piece-type2}", "front wheel");
            }
            if (pieceType2 == Constants.WHEEL2)
            {
                preparedString = preparedString.Replace("{piece-type2}", "middle front wheel");
            }
            if (pieceType2 == Constants.WHEEL3)
            {
                preparedString = preparedString.Replace("{piece-type2}", "middle back wheel");
            }
            if (pieceType2 == Constants.WHEEL4)
            {
                preparedString = preparedString.Replace("{piece-type2}", "back wheel");
            }
            if (pieceType2 == Constants.SMOKE1)
            {
                preparedString = preparedString.Replace("{piece-type2}", "front chimney piece");
            }
            if (pieceType2 == Constants.SMOKE2)
            {
                preparedString = preparedString.Replace("{piece-type2}", "middle chimney piece");
            }
            if (pieceType2 == Constants.SMOKE3)
            {
                preparedString = preparedString.Replace("{piece-type2}", "back chimney piece");
            }

            if (pieceType3 == Constants.FRONT)
            {
                preparedString = preparedString.Replace("{piece-type3}", "front piece");
            }
            if (pieceType3 == Constants.BODY)
            {
                preparedString = preparedString.Replace("{piece-type3}", "body piece");
            }
            if (pieceType3 == Constants.CABIN)
            {
                preparedString = preparedString.Replace("{piece-type3}", "cabin piece");
            }
            if (pieceType3 == Constants.BACK)
            {
                preparedString = preparedString.Replace("{piece-type3}", "back piece");
            }
            if (pieceType3 == Constants.WHEEL1)
            {
                preparedString = preparedString.Replace("{piece-type3}", "front wheel");
            }
            if (pieceType3 == Constants.WHEEL2)
            {
                preparedString = preparedString.Replace("{piece-type3}", "middle front wheel");
            }
            if (pieceType3 == Constants.WHEEL3)
            {
                preparedString = preparedString.Replace("{piece-type3}", "middle back wheel");
            }
            if (pieceType3 == Constants.WHEEL4)
            {
                preparedString = preparedString.Replace("{piece-type3}", "back wheel");
            }
            if (pieceType3 == Constants.SMOKE1)
            {
                preparedString = preparedString.Replace("{piece-type3}", "front chimney piece");
            }
            if (pieceType3 == Constants.SMOKE2)
            {
                preparedString = preparedString.Replace("{piece-type3}", "middle chimney piece");
            }
            if (pieceType3 == Constants.SMOKE3)
            {
                preparedString = preparedString.Replace("{piece-type3}", "back chimney piece");
            }

            if (pieceType4 == Constants.FRONT)
            {
                preparedString = preparedString.Replace("{piece-type4}", "front piece");
            }
            if (pieceType4 == Constants.BODY)
            {
                preparedString = preparedString.Replace("{piece-type4}", "body piece");
            }
            if (pieceType4 == Constants.CABIN)
            {
                preparedString = preparedString.Replace("{piece-type4}", "cabin piece");
            }
            if (pieceType4 == Constants.BACK)
            {
                preparedString = preparedString.Replace("{piece-type4}", "back piece");
            }
            if (pieceType4 == Constants.WHEEL1)
            {
                preparedString = preparedString.Replace("{piece-type4}", "front wheel");
            }
            if (pieceType4 == Constants.WHEEL2)
            {
                preparedString = preparedString.Replace("{piece-type4}", "middle front wheel");
            }
            if (pieceType4 == Constants.WHEEL3)
            {
                preparedString = preparedString.Replace("{piece-type4}", "middle back wheel");
            }
            if (pieceType4 == Constants.WHEEL4)
            {
                preparedString = preparedString.Replace("{piece-type4}", "back wheel");
            }
            if (pieceType4 == Constants.SMOKE1)
            {
                preparedString = preparedString.Replace("{piece-type4}", "front chimney piece");
            }
            if (pieceType4 == Constants.SMOKE2)
            {
                preparedString = preparedString.Replace("{piece-type4}", "middle chimney piece");
            }
            if (pieceType4 == Constants.SMOKE3)
            {
                preparedString = preparedString.Replace("{piece-type4}", "back chimney piece");
            }
            return preparedString;
        }
    }
}
 
 