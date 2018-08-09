# USC Expedition Games

This repository contains the games USC developed for the Expeditions grant. There are three games, each with five different activities. The games are web-based, written in Javascript using the Phaser JS library.

## Setup

Clone the repo:

    $ git clone git@github.com:interaction-lab/expedition-games.git

[Install and setup ROS Indigo](http://wiki.ros.org/indigo/installation/Ubuntu) (Desktop-Full Install recommended)

Run the basic setup script in the root of this repository:

    $ ./basic_setup.sh

This script installs Node.js and npm.

Install NodeJS dependencies (run in expedition-games repo):

    $ npm install

Generate `main.js` (again, in the root of the expedition-games repo):

    $ gulp build

**Warning: Be sure to rerun `gulp build` anytime the JS files change (say after a `git pull`). Gulp compiles all the JS games into a single JS file.**

## Running Games

**For USC, these steps will slightly differ. See the [Expedition Backend](https://github.com/interaction-lab/expedition-year-5-backend/) repo for running the games with our ROS backend**

To run the games, launch a rosbridge server:

    $ roslaunch rosbridge_server rosbridge_websocket.launch

In the expedition-games repo, run an http-sever:

    $ http-server -p 8080

Open a browser and head to localhost:8080 to see the default loading screen for the games.


## Starting Games

Our JS games begin when the appropriate [GameCommand messages](https://github.com/sociallyassistiverobotics/sar_game_command_msgs) are received.

To start a game, run the following in a new terminal:

    $ source /path/to/catkin_ws/devel/setup.*sh
    $ rosrun sar_game_command_sender sar_game_command_sender.py --game <game> -l <level> start

The options for `game` are:

* `galactic_traveler` or `g`
* `spaceship_tidyup` or `t`
* `alien_codes` or `a`

The options for level are 1, 2, or 3.

You can end the game by running the same `rosrun` command from above, but pass in `end` instead of `start`.

## Generating speech file (for USC/CoRDial only)
To create the scripts.txt format of the speech to pregenerate audio for cordial,
open developer console on the index page and refresh the page - the console
should display the formatted text to copy to the scripts.txt file.

## Version and Dependency Notes

The games were built and tested with:

* Python 2.7
* ROS Indigo
* Ubuntu 14.04 LTS (64-bit)
* http-server 0.9.0
* Phaser.js 2.5.0

## Bugs and Issues

To report any bugs or issues, go to the [issue tracker page on GitHub](https://github.com/interaction-lab/expedition-games/issues)
