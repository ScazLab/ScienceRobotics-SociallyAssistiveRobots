# SAR Year 5 Rocket Barrier Game

This game is designed to be deployed in the SAR Year 5 deployment. The Rocket Barrier Game serves to promote the skill of perspective taking in children with ASD. 

## Contributors

[Sarah Strohkorb](mailto:sarah.strohkorb@yale.edu) developed the game play and code. [Laura Boccanfuso](mailto:laura.boccanfuso@yale.edu) and her intern [Olivia (Liv) Pruett](mailto:olivia.pruett.yale.edu@gmail.com) developed the behaviors of the robot during gameplay (the content found in the [robot_behavior_rocket_barrier_game.json](https://github.com/sociallyassistiverobotics/sar-yale-rocket-barrier-game/blob/master/rocket-barrier-game/Assets/Resources/robot_behavior_rocket_barrier_game.json) file).

## Gameplay

The players of the rocket barrier game include the child with ASD , their parent (or other care provider), and the robot. The child and the parent take turns being the 'explainer' and the 'builder'. The robot has more of a third-party outside role. 

The game begins with the robot reminding the child and parent of the rules of the game. Then, the eplainer constructs a rocket from the pieces avaliable. When they are complete they will press a button indicating that they have finished construction. Following the completion of construction, the explainer starts to explain how to build the rocket they've constructed to the builder. The builder then starts to build a rocket based on the explainer's instructions. The screen of the game has buttons that allow the builder and explainer access to their separate rockets (the explainer - to reference the rocket they built to give hints to the builder; and the builder - to construct their rocket). The robot doesn't interject too much during the gameplay, other than to encourage the players and keep them engaged. 

After the builder has completed their rocket, they'll press a button indicating they've finished. Then, the screen will show both of the players' rockets side-by-side, highlighting the differences between the two. The robot will make an utterance designed to help the child with ASD learn from any mistakes made in the game.  

The game has four levels:

1. Small rocket size (2 booster slots, 4 body slots, 2 fin slots, and 1 cone slot), minimum number of piece options

2. Medium rocket size (3 booster slots, 6 body slots, 2 fin slots, and 1 cone slot), more piece options

3. Large rocket size (3 booster slots, 9 body slots, 2 fin slots, and 1 cone slot), even more piece options

4. Extra-large rocket size (4 booster slots, 12 body slots, 2 fin slots, and 1 cone slot), maximum number of piece options


There are two outcome measures for this game: 

1. How many pieces were accurately placed on the rocket (correct piece in correct slot) out of the total number of piece slots. 

2. How many pieces were placed on the rocket that were similar between the two rockets (regardless of whether or not the pieces were placed in the correct slots) out of the total number of piece slots. 

## Unity Platform

This game was developed in Unity 5.4.0f3. It has the following dependencies: 
- [MiniJSON](https://gist.github.com/darktable/1411710): a pretty basic C# JSON encoder and decoder. It can serialize and deserialize JSON strings, which are sent to and received from the ROS rosbridge_server. I've used the code mostly as-is, with one or two of the bug fixes listed in the comments on the github gist page added in.
- [websocket-sharp](https://github.com/sta/websocket-sharp): a .Net implementation of websockets, and is used to communicate with the ROS rosbridge_server. (Note that if you try to build this project, the Newtonsoft.Json dll appears to be missing. However, a dll is built and placed in bin/Debug anyway. Some functionality may be missing as a result, but it doesn't seem to be necessary for this project.)

## Configuration

This Unity game sends messages to a rosbridge server through a rosbridge websocket. We used the [SAR-opal-base](https://github.com/sociallyassistiverobotics/SAR-opal-base) code as a template for making this connection to enable ROS message sending from this game. 

**The game will not run unless it connects properly to a rosbridge server.** The connection configuration information should be put in the `yurp_config.txt` file in either the `Assets/Resources` directory if running the game from the Unity Editor, or within the `executable_name_Data/Resources` directory that's on the same directory level as the executable for the game. You will have to manually add the configuration file if you're running the game as a standalone app (not within the Unity editor). This is because when the app is packaged up, all the game assets are packaged up by Unity and cannot be easily edited after the game is installed. If that file doesn't exist, or if connecting fails with the values listed in that file, the game will try default values set in the packaged game.

Within the `yurp_config.txt` file, you'll have three configuration options: 
- server: [string] the IP address or hostname of the ROS server
- port: [string] port number to use
- log_debug_to_ros: [boolean] whether or not to log Unity's Debug.Log calls to the ROS topic "/yurp_log".

To start up the rosbridge (which should be done before trying to run the game), execute the following command: 
```
roslaunch rosbridge_server rosbridge_websocket.launch 
```

If the specified server address does not exist on the network, there is a 90s timeout before it'll give up trying (hardcoded in the websocket library, so one could patch the library to change the timeout length if one so desires). This will manifest as the application hanging, unresponsive, for the duration of the timeout.

If the server address does exist but if you've forgotten to start rosbridge_server, the connection will be refused.

## ROS Messages

The game subscribes to the ROS topic "/sar/game_command" to receive messages of type [sar_game_command_msgs](https://github.com/sociallyassistiverobotics/sar_game_command_msgs)/GameCommand.

The game publishes [sar_game_command_msgs](https://github.com/sociallyassistiverobotics/sar_game_command_msgs)/GameState messages to the ROS topic "/sar/game_state". These messages are used to communicate what's going on in the game (start, in progress, paused, user timeout, end) as well as pass on game performance information at the end of the game.

The game publishes [sar_robot_command_msgs](https://github.com/sociallyassistiverobotics/sar_robot_command_msgs)/RobotCommand messages to the ROS topic "/sar/robot_command". These messages are used to send the robot commands on what the robot should do and say. 

The game publishes std_msgs/String messages to the ROS topic "/yurp_log". These messages are used for logging purposes.

### Robot Utterances and Actions

At various points in the game, ROS messages will be sent from the game to the robot to tell the robot what to say and what animations to enact. These utterances and animations can be found in [robot_behavior_rocket_barrier_game.json](https://github.com/sociallyassistiverobotics/sar-yale-rocket-barrier-game/blob/master/rocket-barrier-game/Assets/Resources/robot_behavior_rocket_barrier_game.json). **This file, like the `yurp.config` file must be placed in the `Assets/Resources` or `executable_name_Data/Resources` directory (for running the game in Unity Editor and standalone app respectively) in order for the program to run successfully.** In the game, when the game determines it's time for the robot to make an action, it randomly selects one of utterance/action strings in the appropriate category. This file can be edited independently. Utterances can easily be edited or added to pre-existing lists. 

During gameplay, the robot makes the following utterances: 
- "game-first-time": (CURRENTLY NOT IMPLEMENTED - WAITING FOR DEMO MODE TO BE SENT VIA ROS MESSAGE) The first time this game is ever played, the robot introduces the game and the rules. 
- "game-start": When the game is opened each time, the robot will remind the players of the rules.
- "explainer-start-building": At the beginning of the round, the robot prompts the players to take their specific roles (builder, explainer) and prompts the explainer to start building their rocket.
- "builder-start-building": After the explainer has finished building their rocket, the robot prompts the builder to start building (and the explainer to start explaining to the builder how to build the rocket).
- "encouragement": An encouragment is made by the robot every 45 seconds if the game is on the builder or explainer rocket constructing screens.
- "prompt-inactivity": If there hasn't been a screen interaction for the past 30 seconds, the robot will prompt the users to engage in the game.
- "prompt-hurry-up": If the round has lasted longer than 10 minutes, the robot encourages the players to finish up at 10 minutes and every 60 seconds after that. 
- "round-over": After the builder has finished construction of their rocket, the robot announces the end of the round.
- "feedback-[]": The robot offers one point of feedback for the child and caregiver. This point of feedback is informed by whether the pair was successful or not, and what kind of mistake was made. If incorrect pieces had the same color, flame-color, or shape/design; the robot recognizes this and makes tailored feedback. 
- "restart-prompt": 20 seconds after the builder presses the done button and the two rockets are revealed side-by-side the 'Play Again' button appears and the robot prompts the child and caregiver to play again.
- "graceful-exit[]": Once the game end message has been received, the robot urges the players to end the game soon, dependent on where they are in the game. 
- "goodbye": When the robot shuts down the game, says goodbye.
- "tutorial-[]": Utterances guiding the players through the tutorial, numbered in order. 

## Unity Editor Game Development Notes

If you want to edit the game without having to first have a rosbridge server set up and the correct configuration details ready-to-go, you can uncheck the `Send Receive ROS Messages` boolean in the `Main Game Controller Script (Script)` component of the `GameManager` gameobject in the StartScene. 
