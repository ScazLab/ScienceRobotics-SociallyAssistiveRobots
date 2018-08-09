using System.Collections.Generic;
using System.Collections;
using System.IO;
using System;
using MiniJSON;
using UnityEngine;

namespace RocketBarrierGame {
	
	public static class Utilities {

		public static bool ParseConfig (string path, out GameConfig gameConfig) {
			gameConfig.server = "";
			gameConfig.port = "";
			gameConfig.logDebugToROS = false;

			string config = "";
			try {
				config = File.ReadAllText(path);
				Logger.Log("got config: " + config);
				config.Replace("\n", "");

				Dictionary<string, object> data = null;
				data = Json.Deserialize(config) as Dictionary<string, object>;
				if(data == null) {   
					Logger.LogError("Could not parse JSON config file!");
					return false;
				}
				Logger.Log("deserialized " + data.Count + " objects from JSON!");

				// if the config file doesn't have all parts, consider it invalid
				if(!(data.ContainsKey("server") && data.ContainsKey("port") && data.ContainsKey("log_debug_to_ros"))) {
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

			} catch(Exception e) {
				Logger.LogError("Could not read config file! File path given was " 
					+ path + "\nError: " + e);
				return false;
			}

		}

		public static bool ParseRobotUtterancesFile (string path, out Dictionary<string, List<List<string>>> robotUtterances) {

			Dictionary<string, object> tempRobotUtterances = new Dictionary<string, object> ();
			robotUtterances = new Dictionary<string, List<List<string>>> ();

			string robotUtterancesStringContents = "";
			try {
				robotUtterancesStringContents = File.ReadAllText(path);
				robotUtterancesStringContents.Replace("\n", "");

				tempRobotUtterances = Json.Deserialize(robotUtterancesStringContents) as Dictionary<string, object>;
				if (tempRobotUtterances == null) {
					Logger.LogError("Could not parse JSON robot utterances file!");
					return false;
				}

				List<List<string>> utterancesInCategory = new List<List<string>> ();
				List<string> singleUtteranceToAdd = new List<string> ();
				foreach (KeyValuePair<string, object> tempRobotUtterancesEntry in tempRobotUtterances) {
					utterancesInCategory = new List<List<string>> ();
					foreach (object tempRobotUtterancesValueListItem in (List<object>)tempRobotUtterancesEntry.Value) {
						singleUtteranceToAdd = new List<string> ();
						foreach (object tempRobotUtteranceSegment in (List<object>) tempRobotUtterancesValueListItem) {
							singleUtteranceToAdd.Add ((string)tempRobotUtteranceSegment);
						}
						utterancesInCategory.Add (singleUtteranceToAdd);
					}
					robotUtterances.Add(tempRobotUtterancesEntry.Key, utterancesInCategory);
				}

				Logger.Log ("deserialized " + robotUtterances.Count + " objects from JSON!");
				return true;
			} catch (Exception e) {
				Logger.LogError("Could not read robot utterances file! File path given was " + path + "\nError: " + e);
				return false;
			}

		}

		public static string PrepareUtteranceForSending (string utteranceString, bool childExplainer) {

			string preparedString = String.Copy (utteranceString);

			// for the active player
			GameObject gameManager = GameObject.Find ("GameManager");
			if (gameManager != null) {
				if (gameManager.GetComponent<MainGameController> ().currentScene == Constants.BUILDER_SCENE) {
					preparedString = preparedString.Replace ("{active-player-name}", "{builder-name}");
				} else {
					preparedString = preparedString.Replace ("{active-player-name}", "{explainer-name}");
				} 
			}

			// for role specific substitutions
			if (childExplainer) {
				// explainer 
				preparedString = preparedString.Replace ("{explainer-name}", "[child-name]");
				preparedString = preparedString.Replace ("{look-at-explainer}", "<lookat_child>");
				preparedString = preparedString.Replace ("{look-at-explainer, nb}", "<lookat_child,nb>");
				preparedString = preparedString.Replace ("{look-at-explainer,nb}", "<lookat_child,nb>");
				preparedString = preparedString.Replace ("{look-at-explainer, b}", "<lookat_child,b>");
				preparedString = preparedString.Replace ("{look-at-explainer,b}", "<lookat_child,b>");
				// builder
				preparedString = preparedString.Replace ("{builder-name}", "[guardian-name]");
				preparedString = preparedString.Replace ("{look-at-builder}", "<lookat_guardian>");
				preparedString = preparedString.Replace ("{look-at-builder, nb}", "<lookat_guardian,nb>");
				preparedString = preparedString.Replace ("{look-at-builder,nb}", "<lookat_guardian,nb>");
				preparedString = preparedString.Replace ("{look-at-builder, b}", "<lookat_guardian,b>");
				preparedString = preparedString.Replace ("{look-at-builder,b}", "<lookat_guardian,b>");
			} else {
				// explainer 
				preparedString = preparedString.Replace ("{explainer-name}", "[guardian-name]");
				preparedString = preparedString.Replace ("{look-at-explainer}", "<lookat_guardian>");
				preparedString = preparedString.Replace ("{look-at-explainer, nb}", "<lookat_guardian,nb>");
				preparedString = preparedString.Replace ("{look-at-explainer,nb}", "<lookat_guardian,nb>");
				preparedString = preparedString.Replace ("{look-at-explainer, b}", "<lookat_guardian,b>");
				preparedString = preparedString.Replace ("{look-at-explainer,b}", "<lookat_guardian,b>");
				// builder
				preparedString = preparedString.Replace ("{builder-name}", "[child-name]");
				preparedString = preparedString.Replace ("{look-at-builder}", "<lookat_child>");
				preparedString = preparedString.Replace ("{look-at-builder, nb}", "<lookat_child,nb>");
				preparedString = preparedString.Replace ("{look-at-builder,nb}", "<lookat_child,nb>");
				preparedString = preparedString.Replace ("{look-at-builder, b}", "<lookat_child,b>");
				preparedString = preparedString.Replace ("{look-at-builder,b}", "<lookat_child,b>");
			}

			return preparedString;
		}


	}
}