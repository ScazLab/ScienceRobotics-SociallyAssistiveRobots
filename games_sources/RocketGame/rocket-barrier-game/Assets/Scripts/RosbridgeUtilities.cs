using System.Collections.Generic;
using System.Collections;
using System.IO;
using System;
using MiniJSON;
using UnityEngine;

namespace RocketBarrierGame {
	
	public static class RosbridgeUtilities {

		public static string GetROSJsonPublishGameStateMsg (string topic, int gameState, Dictionary<string,float> performance) {
			// build a dictionary of things to include in the message
			Dictionary<string,object> rosPublish = new Dictionary<string, object> ();
			rosPublish.Add ("op", "publish");
			rosPublish.Add ("topic", topic);
			Dictionary<string, object> rosMessage = new Dictionary<string, object>();
			rosMessage.Add ("game", Constants.SAR_GAME_IDENTITY);
			rosMessage.Add ("state", gameState);
			rosMessage.Add ("performance", Json.Serialize(performance));
			rosPublish.Add ("msg", rosMessage);

			return Json.Serialize (rosPublish);
		}

		public static string GetROSJsonPublishRobotCommandMsg (string topic, int command, string id, bool interrupt, string robotActionAndSpeech) {
			// build a dictionary of things to include in the message
			Dictionary<string,object> rosPublish = new Dictionary<string, object> ();
			rosPublish.Add ("op", "publish");
			rosPublish.Add ("topic", topic);
			Dictionary<string, object> rosMessage = new Dictionary<string, object>();
			rosMessage.Add ("command", command);
			rosMessage.Add ("id", id);
			rosMessage.Add ("interrupt", interrupt);
			rosMessage.Add ("properties", robotActionAndSpeech);
			rosPublish.Add ("msg", rosMessage);

			return Json.Serialize (rosPublish);
		}
			
		public static string GetROSJsonPublishStringMsg (string topic, string message) {
			// build a dictionary of things to include in the message
			Dictionary<string,object> rosPublish = new Dictionary<string, object>();
			rosPublish.Add("op", "publish");
			rosPublish.Add("topic", topic);
			Dictionary<string,object> rosMessage = new Dictionary<string, object>();
			rosMessage.Add("data", message);
			rosPublish.Add("msg", rosMessage);

			return Json.Serialize(rosPublish);
		}
			
		public static string GetROSJsonSubscribeMsg (string topic, string messageType) {
			// build a dictionary of things to include in the message
			Dictionary<string,object> rosSubscribe = new Dictionary<string, object>();
			rosSubscribe.Add("op", "subscribe");
			rosSubscribe.Add("topic", topic);
			rosSubscribe.Add("type", messageType);

			return Json.Serialize(rosSubscribe);
		}
			
		public static string GetROSJsonAdvertiseMsg (string topic, string messageType) {
			// build a dictionary of things to include in the message
			Dictionary<string,object> rosAdvertise = new Dictionary<string, object>();
			rosAdvertise.Add("op", "advertise");
			rosAdvertise.Add("topic", topic);
			rosAdvertise.Add("type", messageType); 

			return Json.Serialize(rosAdvertise);
		}

		public static void DecodeROSJsonCommand (string rosmsg, out int game, out int command, out int level) {

			// set up output variables
			game = -1;
			command = -1;
			level = -1;

			// there is also a header in the command message, but we aren't
			// using it for anything
			// 
			// parse data, see if it's valid
			//
			// messages might look like:
			// {"topic": "/yurp_command", "msg": {"header":{"stamp:{"secs": 1465502871, 
			// "nsecs":940923929}, "frame_id":", "seq":1}, "game":1, "command":0, "level":2}, 
			// "op": "publish"}
			//
			// or:
			// "topic": "/yurp_command", "msg": {"game":0, "command":2, "level":""}, "op": "publish"
			//
			// should be valid json, so we try parsing the json

			Dictionary<string, object> data = null;
			data = Json.Deserialize(rosmsg) as Dictionary<string, object>;
			if(data == null) {   
				Logger.LogWarning("[decode ROS msg] Could not parse JSON message!");
				return;
			}
			Logger.Log("[decode ROS msg] deserialized " + data.Count + " objects from JSON!");

			// message sent over rosbridge comes with the topic name and what the
			// operation was
			//
			// TODO should we check that the topic matches one that we're subscribed
			// to before parsing further? Would need to keep a list of subscriptions. 
			//
			// if the message doesn't have all three parts, consider it invalid
			if(!data.ContainsKey("msg") && !data.ContainsKey("topic") 
				&& !data.ContainsKey("op")) 
			{
				Logger.LogWarning("[decode ROS msg] Did not get a valid message!");
				return;
			}

			Logger.Log("[decode ROS msg] Got " + data["op"] + " message on topic " + data["topic"]);

			// parse the actual message
			Logger.Log("[decode ROS msg] Parsing message: " + data["msg"]);
			Dictionary<string, object> msg = data["msg"] as Dictionary<string, object>;

			// print header for debugging
			if(msg.ContainsKey("header"))
			{
				Logger.Log("[decode ROS msg]" + msg["header"]);
			}

			// get the game
			if(msg.ContainsKey("game")) {
				Logger.Log("[decode ROS msg] game: " + msg["game"]);
				try {
					game = Convert.ToInt32(msg["game"]);
				} catch(Exception ex) {
					Logger.LogError("[decode ROS msg] Error! Could not get game: " + ex);
				}
			}

			// get the command
			if(msg.ContainsKey("command")) {
				Logger.Log("[decode ROS msg] command: " + msg["command"]);
				try {
					command = Convert.ToInt32(msg["command"]);
				} catch(Exception ex) {
					Logger.LogError("[decode ROS msg] Error! Could not get command: " + ex);
				}
			}

			// get the level
			if(msg.ContainsKey("level")) {
				Logger.Log("[decode ROS msg] level: " + msg["level"]);
				try {
					level = Convert.ToInt32(msg["level"]);
				} catch(Exception ex) {
					Logger.LogError("[decode ROS msg] Error! Could not get level: " + ex);
				}
			}

		}

		private static int[] ObjectToIntArray (IEnumerable en)
		{
			// C# is weird about conversions from object to arrays
			// so this is a hack way of converting an object into an
			// IEnumerable so we can then convert each element of the
			// array to a number, so we can then make an array.....
			int[] posn = {0,0,0};
			if(en != null) 
			{
				int count = 0;
				foreach(object el in en) 
				{
					posn[count] = Convert.ToInt32(el);
					count++;
				}
			}
			return posn;
		}


		private static string[] ObjectToStringArray (IEnumerable en)
		{
			// C# is weird about conversions from object to arrays
			// so this is a hack way of converting an object into an
			// IEnumerable so we can then convert each element of the
			// array to a string, so we can then make an array.....
			string[] s;
			if (en != null)
			{
				// get length of array
				int count = 0;
				foreach(object el in en) 
				{
					count++;
				}
				// make a destination array of the right size 
				s = new string[count];

				// reset counter
				count = 0;

				// convert each element to a string
				foreach(object el in en) 
				{
					s[count] = Convert.ToString(el);
					count++;
				}
				return s;
			}
			return null;
		}
			
		public static Dictionary<string, object> GetROSHeader()
		{
			Dictionary<string,object> header = new Dictionary<string, object>();
			// header sequence number (ROS overrides this)
			header.Add("seq", 0);
			// header frame (no frame)
			header.Add("frame_id", "");
			// time for header
			Dictionary<string, Int32> time = new Dictionary<string, Int32>();
			TimeSpan unixtime = DateTime.UtcNow.Subtract(new DateTime(1970,1,1));
			time.Add("sec", (Int32)(unixtime.TotalSeconds));
			time.Add("nsec", (Int32)(unixtime.Milliseconds * 1000));
			// add time to header
			header.Add("stamp", time);
			return header;
		}
	}
}