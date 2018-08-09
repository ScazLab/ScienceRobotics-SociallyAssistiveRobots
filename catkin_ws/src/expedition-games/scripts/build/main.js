(function(f){if(typeof exports==="object"&&typeof module!=="undefined"){module.exports=f()}else if(typeof define==="function"&&define.amd){define([],f)}else{var g;if(typeof window!=="undefined"){g=window}else if(typeof global!=="undefined"){g=global}else if(typeof self!=="undefined"){g=self}else{g=this}g.SARLIB = f()}})(function(){var define,module,exports;return (function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
/* depends on Phaser, ROS*/

var CONST = require('./constants.js');
var SPEECH = require('./speech.js');

var path_to_assets='../../assets/';
var path_to_sprites=path_to_assets+'sprites/';
var path_to_images=path_to_assets+'images/';

/*********************************************************************************************************************
 * Parent class for all SARGames
 *
 */
function SARGame(ros, level, gameid, completionCallBack, activityID) {
    // ID of Game (e.g. Galactic Traveler = 2)
    this.gameid = gameid;

    this.activityid = activityID;

    // Ros Handler
    this.ros = ros;

    // Difficulty level of activity
    this.level = level;

    this.attempts = 1;

    this.stat = false;

    this.start = performance.now(); //start time in milliseconds
    this.duration = 0; //duration in seconds
    this.tutorialStringIndex = 0;

    // Callback function called when activity is successfully completed
    this.completionCallBack = completionCallBack;

    /* Phaser Game */
    this.game = new Phaser.Game(window.innerWidth, window.innerHeight, Phaser.AUTO, '',
                    {
                        preload: this.preload.bind(this), // bind sets 'this' in callback functions to current 'this' obj
                        create: this.create.bind(this),
                        update: this.update.bind(this),
                        render: this.render.bind(this)
                    });


    /* ROS Topics */

    // Game State (Publishing to)
    this.game_state = new ROSLIB.Topic({
        ros: this.ros,
        name: '/sar/game_state',
        messageType: 'sar_game_command_msgs/GameState'
    });

    // Robot Commands (Publishing to)
    this.robot_command = new ROSLIB.Topic({
        ros: ros,
        name: '/sar/robot_command',
        messageType: 'sar_robot_command_msgs/RobotCommand'
    });

    // Game Commands (Subscribed to)
    this.game_command = new ROSLIB.Topic({
        ros: this.ros,
        name: '/sar/game_command',
        messageType: 'sar_game_command_msgs/GameCommand'
    });
    this.game_command.subscribe( message => {
        switch(message.command){

            case CONST.GameCommands.PAUSE:
                console.log('Received game PAUSE message.');
                this.game.paused = true;
                this.publishGameState(CONST.GameStates.PAUSED, null);
                break;

            case CONST.GameCommands.CONTINUE:
                console.log('Received game CONTINUE message.');
                this.game.paused = false;
                this.publishGameState(CONST.GameStates.IN_PROGRESS, null);
                break;

            default:
                break;
        }
    });
}


/**
 * Phaser Game callbacks
 */
SARGame.prototype.init = function() {};
SARGame.prototype.preload = function() {};
SARGame.prototype.create = function() {};
SARGame.prototype.update = function() {};
SARGame.prototype.render = function() {};

SARGame.prototype.resize = function() {
    if (this.game.scale) {
        this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
        this.game.scale.updateLayout();
    }
};

/**
 * Kills the current Phaser game
 */
SARGame.prototype.end = function() {
    this.duration = (performance.now() - this.start) / 1000.;
    this.game.destroy();
    this.game_command.unsubscribe();
};


/**
 * Function to run when an activity is successfully completed.
 * Calls this.end to destroy Phaser game and unsubscribe topics. Then calls the completion call back which passes
 * control back to the GameWrapper
 */
SARGame.prototype.gameComplete = function() {
    // window.alert('Yay! You did it!!');
    this.end();
    this.completionCallBack();
};


/**
 * Publishes a message to the /sar/game_state Topic
 * @param {Int} game
 * @param {Int} state
 * @param {String} performance
 */
SARGame.prototype.publishGameState = function(state, performance) {
    var msg = new ROSLIB.Message({
        header: {
            frame_id: "JS" //TODO: figure out proper value for this
        },
        game: this.gameid,
        state: state,
    });
    if (performance !== null)
        msg.performance =  performance;
    this.game_state.publish(msg);
};


/**
 * Publishes a message to the /sar/robot_command Topic
 * @param {String} id
 * @param {Int} command
 * @param {Int} interrupt
 * @param {String} properties
 */
SARGame.prototype.publishRobotCommand = function(id, command, interrupt, properties) {
    var msg = new ROSLIB.Message({
        header: {
            frame_id: "JS" //TODO: figure out proper value for this
        },
        id: id,
        command: command,
        interrupt: interrupt,
        properties: properties
    });
    this.robot_command.publish(msg);
};

/*
*   Helper function to access the game-start instructions for the current game and activity.
*/
SARGame.prototype.getInstructions = function() {
    return SPEECH.GAMES[Object.getOwnPropertyNames(CONST.Games)[this.gameid]][this.activityid].instructions;
}

/*
*   Helper function to access the mistake feedback strings for the current game and activity.
*/
SARGame.prototype.getFeedback = function() {
    return SPEECH.GAMES[Object.getOwnPropertyNames(CONST.Games)[this.gameid]][this.activityid].feedback.concat(SPEECH.GENERAL_FEEDBACK);
}
/**
 * Publishes a random success message chosen from the
 * available ones in speech.js to the /sar/robot_command Topic
 */
SARGame.prototype.publishRobotSuccessMessage = function() {
    // select a random success message
    var idx = this.game.rnd.integerInRange(0, SPEECH.SUCCESS.length - 1);
    this.publishRobotCommand(SPEECH.SUCCESS[idx].id, CONST.RobotCommands.DO, false, SPEECH.SUCCESS[idx].msg);
}

/**
 * Select a random string from the provided messageArr, with string format options for %i %s and %d.
 * The placeholders are replaced with the respective function arguments,
 * or the replacement can be omitted by specifiying an argument as null.
 * @param {Array{String}} messageArr - messages to choose from
 * @param {Array} arguments for string format
 */
SARGame.prototype.formatAndPublishRobotCommand = function() {
    var args = arguments;
    var messageArr = args[0];
    var formatArgs = args[1];

    var idx = this.game.rnd.integerInRange(0, messageArr.length - 1);
    // format using our custom String format function
    var msg_id = String.prototype.format.apply(messageArr[idx].id, formatArgs);
    var msg_str = String.prototype.format.apply(messageArr[idx].msg, formatArgs);

    this.publishRobotCommand(msg_id, CONST.RobotCommands.DO, false, msg_str);
}

/**
 * Function that is called from activities when activity instructions are requested.
 * Selects a random string from the activity's instructions and formats it.
 * @param {Array} arguments for string format
 */
SARGame.prototype.publishRobotGameIntro = function() {
    // slice and sort needed to convert arguments object to array
    this.formatAndPublishRobotCommand.apply(this, [this.getInstructions(), Array.prototype.slice.call(arguments).sort()]);
}

/**
 * Function that is called from activities when feedback is requested.
 * Selects a random string from the activity's feedback and formats it.
 * @param {Array} arguments for string format
 */
SARGame.prototype.publishRobotGameFeedback = function() {
   this.formatAndPublishRobotCommand.apply(this, [this.getFeedback(), Array.prototype.slice.call(arguments).sort()]);
}

SARGame.prototype.publishTutorialSequenceBlocking = function(next) {
    var msg = SPEECH.TUTORIALS[Object.getOwnPropertyNames(CONST.Games)[this.gameid]][this.tutorialStringIndex]
    if (!msg)
        return;
    var msg_id = msg.id;
    var msg_str = msg.msg;
    var msg_dur = msg.duration;

    this.publishRobotCommand(msg_id, CONST.RobotCommands.DO, false, msg_str);
    this.tutorialStringIndex++;
    console.log(msg_str);
    setTimeout(next, msg_dur * 1000);
}

/*
 * Randomly shuffle an array
 * @param {Array} array
 */
SARGame.prototype.shuffle = function(array) {
    var currentIndex = array.length, temporaryValue, randomIndex;
    // While there remain elements to shuffle...
    while (0 !== currentIndex) {
        // Pick a remaining element...
        randomIndex = Math.floor(Math.random() * currentIndex);
        currentIndex -= 1;
        // And swap it with the current element.
        temporaryValue = array[currentIndex];
        array[currentIndex] = array[randomIndex];
        array[randomIndex] = temporaryValue;
    }
    return array;
}


/*********************************************************************************************************************
 * A Phaser game that renders a loading screen to transition between different games
 *
 */
function LoadingScreen() {
    this.game = new Phaser.Game(window.innerWidth, window.innerHeight, Phaser.AUTO, '',
                    {
                        preload: this.preload.bind(this), // bind sets 'this' in callback functions to current 'this' obj
                        create: this.create.bind(this),
                    });
}


/*
 * Load static assets
 */
LoadingScreen.prototype.preload = function() {
    console.log('Loading space game and waiting on SAR game command...');

    //window.addEventListener('resize', event => { this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;  this.game.scale.updateLayout();};

    this.game.load.image('splash', path_to_images+'Backgrounds/HomeScreen.png');
    this.game.load.atlasJSONHash('bot', path_to_sprites+'running_bot.png', path_to_sprites+'running_bot.json');
};


/**
 * Create the assets on screen
 */
LoadingScreen.prototype.create = function() {
    // LOADING SPRITE
    var splash = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'splash');
    splash.anchor.setTo(0.5, 0.5);

    var bot = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'bot');
    bot.anchor.setTo(0.5, 0.5);
    bot.animations.add('run');
    bot.animations.play('run', 15, true); // 15 fps, true = loops
};


LoadingScreen.prototype.end = function() {
    this.game.destroy();
};

function Menu(ros, gameID, next_game, tutorial, level, completionCallBack) {
    var gameWidth = 1920;
    var gameHeight = 1080;
    this.ros = ros;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.level = level;

    this.game = new Phaser.Game(window.innerWidth, window.innerHeight, Phaser.AUTO, '',
                    {
                        preload: this.preload.bind(this), // bind sets 'this' in callback functions to current 'this' obj
                        create: this.create.bind(this),
                    });

    this.gameID = gameID;
    this.next_game = next_game;
    this.tutorial = tutorial;
    this.completionCallBack = completionCallBack;

    this.game_command = new ROSLIB.Topic({
        ros: this.ros,
        name: '/sar/game_command',
        messageType: 'sar_game_command_msgs/GameCommand'
    });

    this.game_state = new ROSLIB.Topic({
        ros: this.ros,
        name: '/sar/game_state',
        messageType: 'sar_game_command_msgs/GameState'
    });

    console.log("Created new menu");
}

/*
 * Load static assets
 */
Menu.prototype.preload = function() {
    this.game.load.spritesheet('buttonTutorial', CONST.ASSETS_PATH+'Items/QuestionButton_Sheet.png', 278, 278, 2, 0, 10);
    this.game.load.spritesheet('buttonGame', CONST.ASSETS_PATH+'Items/PlayButton_Sheet.png', 278, 278, 2, 0, 10);
    this.game.load.image('splash', path_to_images+'Backgrounds/HomeScreen.png');
};


/**
 * Create the assets on screen
 */
Menu.prototype.create = function() {
    var splash = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'splash');
    splash.anchor.setTo(0.5, 0.5);

    var tutButton = this.game.add.button(0, 0, 'buttonTutorial', this.onButtonReleaseTutorial, this, 0, 0, 1, 0);
    tutButton.scale.setTo(1.0*this.sx);
    tutButton.x = (this.game.width - tutButton.width) * 0.7;
    tutButton.y = (this.game.height - tutButton.height) / 2;

    var gameButton = this.game.add.button(0, 0, 'buttonGame', this.onButtonReleaseGame, this, 0, 0, 1, 0);
    gameButton.scale.setTo(1.0*this.sx);
    gameButton.x = (this.game.width - gameButton.width) / 3;
    gameButton.y = (this.game.height - gameButton.height) / 2;
};

Menu.prototype.onButtonReleaseTutorial = function() {
    this.game.destroy();
    // start tutorial
    //new this.tutorial(this.ros, () => new this.next_game(this.ros, this.level, this.completionCallBack));
    this.publishGameState(CONST.GameStates.TUTORIAL, null);
    this.game = new this.tutorial(this.ros, () => {
        new Menu(this.ros, this.gameID, this.next_game, this.tutorial, this.level, this.completionCallBack)
    });
}

Menu.prototype.onButtonReleaseGame = function() {
    this.game.destroy();
    // start game
    this.publishGameState(CONST.GameStates.START, null);
    this.game = new this.next_game(this.ros, this.level, this.completionCallBack);
}

Menu.prototype.end = function() {
    if (this.game) {
        if (this.game.end)
            this.game.end();
        if (this.game.destroy)
            this.game.destroy();
    }
    this.completionCallBack();
}

Menu.prototype.publishGameState = function(state) {
    var msg = new ROSLIB.Message({
        header: {
            frame_id: "JS"
        },
        game: this.gameID,
        state: state,
    });

    this.game_state.publish(msg);
};


/*********************************************************************************************************************
 * A wrapper around the 5 activities for a given game.
 * Each of the three games will have a sublcass of GameWrapper in charge of running the game's respective activities.
 * Takes care of switching between activities when one is completed.
 *
 * param {ROS object} ros - a ROS handler
 * param {Int} level - difficulty
 * param {Func} exitCallBack - call back function to run when we exit this game (i.e. all activities are completed)
 */
function GameWrapper(ros, level, exitCallBack) {
    this.ros = ros;
    this.level = level;
    this.activities, this.gameid, this.currentGame;
    this.exitCallBack = exitCallBack;
    this.completed = [];

    // Game State (Publishing to)
    this.game_state = new ROSLIB.Topic({
        ros: this.ros,
        name: '/sar/game_state',
        messageType: 'sar_game_command_msgs/GameState'
    });
}

/*
 * Product the game performance object based on statistics of activities for this game
 */
GameWrapper.prototype.getGamePerformance = function() {
    var performance = {
        attempts_mean: 0,
        attempts_variance: 0,
        time_spent_mean: 0,
        time_spent_variance: 0,
        activity_with_most_attempts: 0,
        activity_child_spent_longest: 0,
        child_success_percentage: 0,
    }

    var totalAttempts = 0;
    var totalDuration = 0;

    var mostAttempts = 1;
    var longest = -1;

    var perfectActivities = 0;

    for (let activity of this.completed) {
        totalAttempts += activity.attempts;
        totalDuration += activity.duration;

        if (activity.attempts > mostAttempts) {
            mostAttempts = activity.attempts;
            performance.activity_with_most_attempts = activity.activityid;
        }
        if (activity.duration > longest) {
            longest = activity.duration;
            performance.activity_child_spent_longest = activity.activityid;
        }

        if (activity.attempts == 1) {
            perfectActivities++;
        }
    }

    performance.attempts_mean = totalAttempts / this.completed.length;
    performance.time_spent_mean = totalDuration / this.completed.length;

    for (let activity of this.completed) {
        performance.attempts_variance += Math.pow(activity.attempts - performance.attempts_mean, 2);
        performance.time_spent_variance += Math.pow(activity.duration - performance.time_spent_mean, 2);
    }
    performance.attempts_variance /= this.completed.length;
    performance.time_spent_variance /= this.completed.length;

    performance.child_success_percentage = perfectActivities / this.completed.length;

    // if (performance == null)
    //     performance =  {
    //     attempts_mean: 0,
    //     attempts_variance: 0,
    //     time_spent_mean: 0,
    //     time_spent_variance: 0,
    //     activity_with_most_attempts: 0,
    //     activity_child_spent_longest: 0,
    //     child_success_percentage: 0,
    // };
    // if (isNaN(performance.attempts_mean) == true){
    //     performance.attempts_mean = 0;
    // }
    // if (performance.attempts_variance == null){
    //     performance.attempts_variance = 0;
    // }
    // if (performance.time_spent_mean == null){
    //     performance.time_spent_mean = 0;
    // }
    // if (performance.time_spent_variance == null){
    //     performance.time_spent_variance = 0;
    // }
    // if (performance.child_success_percentage == null){
    //     performance.child_success_percentage = 0;
    // }

    return performance;
}


/**
 * Start the next activity
 *  If no activities remaining, call the exitCallBack function
 */
GameWrapper.prototype.start = function() {
    // keep track of completed games if we were running a game before this function call
    if (this.currentGame)
        this.completed.push(this.currentGame);

    if (this.activities.length > 0) {
        var nextGame = this.activities.shift();
        this.currentGame = new nextGame(this.ros, this.level, this.gameid, this.start.bind(this));
    } else {
        var performance = this.getGamePerformance();
        this.publishGameState(CONST.GameStates.END, performance);
        this.exitCallBack();
    }
};


/**
 * Kill the current activity
 */
GameWrapper.prototype.end = function() {
    // this.publishGameState(CONST.GameStates.END, null);
    // this.currentGame.end();
    // cmhuang: also send the performance back
    var performance = this.getGamePerformance();
    this.publishGameState(CONST.GameStates.END, performance);
    this.exitCallBack();
};


GameWrapper.prototype.publishGameState = function(state, performance) {
    var msg = new ROSLIB.Message({
        header: {
            frame_id: "JS" //TODO: figure out proper value for this
        },
        game: this.gameid,
        state: state,
    });
    if (performance === null)
        performance = {};

    msg.performance = JSON.stringify(performance);
    this.game_state.publish(msg);
};


exports.SARGame = SARGame;
exports.LoadingScreen = LoadingScreen;
exports.Menu = Menu;
exports.GameWrapper = GameWrapper;

},{"./constants.js":2,"./speech.js":25}],2:[function(require,module,exports){
exports.Games = Object.freeze({STORYTELLING:0, ROCKET_BARRIER:1, GALACTIC_TRAVELER:2, SPACESHIP_TIDYUP:3, ALIEN_CODES:4, HOUSE_PERSPECTIVE_TAKING:5, TRAIN_SEQUENCING:6});
exports.GameCommands = Object.freeze({START:0, CONTINUE:1, PAUSE:2, END:3, WAIT_FOR_RESPONSE:4, SKIP_RESPONSE:5});
exports.GameStates = Object.freeze({START: 0, IN_PROGRESS: 1, PAUSED: 2, USER_TIMEOUT: 3, END: 4, READY: 5, TUTORIAL: 6});

exports.RobotCommands = Object.freeze({SLEEP:0, WAKEUP:1, DO:2});
exports.RobotActions = Object.freeze({SOME_ACTION:0}); // TODO(Vadim): figure out what controls what actions we can perform

exports.ASSETS_PATH = '../../assets/'; // Relative to main.js in scripts/build/ dir

exports.UNEXPECTED_LEVEL = "Unexpected Error";

/*
 * Custom string format function
 */
if (!String.prototype.format) {
  String.prototype.format = function() {
    var args = arguments;
    return this.replace(/{(\d+)}/g, function(match, number) {
      return typeof args[number] != 'undefined'
        ? args[number]
        : match
      ;
    });
  };
}

},{}],3:[function(require,module,exports){
/**
  * Alien Codes Activity 1
  * -----------------------------
  *      
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 1;

/**
 * Constructor for Activity 1
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity1(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.NUM_IMAGE_OPTIONS = 8;
    this.allButtons = [];
    this.requestImagesCount;
    this.requestImages = [];
    this.responseImages = [];
    this.boxes = [];
    this.shouldRespawn = false;
    this.difficulty = level;
    this.debug = true;

    this.container, this.alien, this.bubble;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.centerX = window.innerWidth / 2;
    this.centerY = window.innerHeight / 2;
}
Activity1.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity1.prototype.preload = function() {
    this.game.load.image('background', CONST.ASSETS_PATH + 'Background/ShipInterior.png', window.innerWidth, window.innerHeight);
    this.game.load.image('alien', CONST.ASSETS_PATH + 'Items/Aliens/Alien7.png');

    this.game.load.image('option1', CONST.ASSETS_PATH + 'Items/Crystals/Crystal1.png');
    this.game.load.image('option2', CONST.ASSETS_PATH + 'Items/Crystals/Crystal2.png');
    this.game.load.image('option3', CONST.ASSETS_PATH + 'Items/Crystals/Crystal4.png');
    this.game.load.image('option4', CONST.ASSETS_PATH + 'Items/Crystals/Crystal5.png');
    this.game.load.image('option5', CONST.ASSETS_PATH + 'Items/Planets/Jupiter.png');
    this.game.load.image('option6', CONST.ASSETS_PATH + 'Items/Planets/Neptune.png');
    this.game.load.image('option7', CONST.ASSETS_PATH + 'Items/Rocks/Moonrock4.png');
    this.game.load.image('option8', CONST.ASSETS_PATH + 'Items/Rocks/Moonrock3.png');

    this.game.load.image('container', CONST.ASSETS_PATH + 'Items/BoxLong.png');
    this.game.load.image('display', CONST.ASSETS_PATH + 'Items/ControlPanel.png');
    this.game.load.image('box', CONST.ASSETS_PATH + 'Items/Box.png');

    this.game.load.image('bubble', CONST.ASSETS_PATH + 'Items/TextBubble.png');
    this.game.load.image('button', CONST.ASSETS_PATH + 'Items/Button.png');
    this.game.load.image('clear', CONST.ASSETS_PATH + 'Items/Clear.png');
    this.game.load.image('right', CONST.ASSETS_PATH + 'Items/Right.png');

    this.game.load.image('ship', CONST.ASSETS_PATH + 'Items/Spaceship.png');

    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity1.prototype.create  = function() {
    if (this.difficulty == 1)
    {
        this.requestImagesCount = Math.floor(Math.random() * 2 + 2);
    }
    else if (this.difficulty == 2)
    {
        this.requestImagesCount = Math.floor(Math.random() * 3 + 3);
    }
    else if (this.difficulty == 3)
    {
        this.requestImagesCount = Math.floor(Math.random()* 3 + 5);
    }

    if (this.debug)
    {
        console.log("Difficulty is: " + this.difficulty);
        console.log("Number of images to match is: " + this.requestImagesCount);
    }

    var spacing = (this.NUM_IMAGE_OPTIONS - this.requestImagesCount) / 2;
    this.leftSpacing = Math.ceil(spacing);
    this.rightSpacing = Math.floor(spacing);

    this.requestRandomImages();

    this.showConstantImages();
    this.showOptions();

    this.showAlienRequest();

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.requestImagesCount);
};

Activity1.prototype.requestRandomImages = function() {
    // for (var i = 0; i < this.leftSpacing; ++i)
    // {
    //     this.requestImages.push(-1);
    // }

    for (var i = 0; i < this.requestImagesCount; ++i)
    {
        this.requestImages.push(Math.floor(Math.random() * this.NUM_IMAGE_OPTIONS));
    }

    // for (var i = 0; i < this.rightSpacing; ++i)
    // {
    //     this.requestImages.push(-1);
    // }

    if (this.debug)
    {
        for (var i = 0; i < this.requestImages.length; ++i)
        {
            console.log("We want: " + this.requestImages[i]);
        }
    }
};

Activity1.prototype.showConstantImages = function() {
    this.background = this.game.add.sprite(0, 0, 'background');
    this.background.scale.setTo(this.sx, this.sy);

    // Reset button
    var resetButton = this.game.add.button(this.centerX - 550 * this.sx, this.centerY + 260 * this.sy, 'button', this.resetImages, this, 2, 1, 0);
    resetButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    var clear = this.game.add.sprite(this.centerX - 535 * this.sx, this.centerY + 285 * this.sy, 'clear');
    clear.scale.setTo(0.25 * this.sx, 0.25 * this.sy);

    // Verify button
    var verifyButton = this.game.add.button(this.centerX + 375 * this.sx, this.centerY + 260 * this.sy, 'button', this.checkImages, this, 2, 1, 0);
    verifyButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    var right = this.game.add.sprite(this.centerX + 400 * this.sx, this.centerY + 295 * this.sy, 'right');
    right.scale.setTo(0.1 * this.sx, 0.1 * this.sy);

    // Below is the box for feedback to the user.
    var display = this.game.add.sprite(this.centerX - 1075 * this.sx, this.centerY - 250 * this.sy, 'display');
    display.scale.setTo(1.125 * this.sx, 0.37 * this.sy);

    // Below are the options for the user to click on.
    this.container = this.game.add.sprite(this.centerX - 375 * this.sx, this.centerY + 200 * this.sy, 'container');
    this.container.scale.setTo(0.5 * this.sx, 0.6 * this.sy);

    // Below is the alien which prompts action from user.
    this.alien = this.game.add.sprite(this.centerX - this.centerX, this.centerY - 500 * this.sy, 'alien');
    this.alien.scale.setTo(0.45 * this.sx, 0.45 * this.sy);

    // Below is the bubble to signify the alien speaking.
    this.bubble = this.game.add.sprite(this.centerX - 760 * this.sx, this.centerY - 525 * this.sy, 'bubble');
    this.bubble.scale.setTo(2.4 * this.sx, 0.9 * this.sy);
};

Activity1.prototype.showOptions = function() {
    for (var i = 1; i <= this.NUM_IMAGE_OPTIONS; ++i)
    {
        this.spawnOption(i);
    }
};
Activity1.prototype.spawnOption = function(optionNumber) {
    var button;
    var width, height;
    if (optionNumber <= 4)
    {
        width = this.centerX - ((160 * optionNumber) * this.sx) + 360 * this.sx;
        height = this.centerY + 240 * this.sy;
    }
    else
    {
        width = this.centerX - (160 * (optionNumber - 4) * this.sx) + 360 * this.sx;
        height = this.centerY + 350 * this.sy;
    }

    button = this.game.add.button(width, height, 'option' + optionNumber, null, this, 2, 1, 0);
    if (optionNumber == 8)
    {
        button.scale.setTo(0.22 * this.sx, 0.22 * this.sy);
    }
    else
    {
        button.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
    }
    button.input.enableDrag(true);
    button.events.onDragStop.add(this.onDragStop, this);
    button.events.onDragStart.add(this.onDragStart, this);
    this.allButtons.push(button);
};

Activity1.prototype.showAlienRequest = function() {
    this.responseImages = [];
    // Below are the Alien's requests.
    for (var i = 0; i < this.requestImages.length; ++i)
    {
        var imageToDisplay = this.requestImages[i] + 1;
        if (imageToDisplay > 0)
        {
            var image = this.game.add.sprite(this.centerX - (600 * this.sx) + (150 * i * this.sx), this.centerY - 420 * this.sy, 'option' + imageToDisplay);
            image.scale.setTo(0.25 * this.sx, 0.25 * this.sy);
            if (imageToDisplay == 8)
            {
                image.scale.setTo(0.22 * this.sx, 0.22 * this.sy);
            }
            else
            {
                image.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
            }


            var box = this.game.add.sprite(this.centerX - (700 * this.sx) + (215 * i * this.sx), this.centerY - 90 * this.sy, 'box');
            box.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
            this.boxes.push(box);

            // var box = this.game.add.group();
            // box.create(this.centerX - (1100 * this.sx) + (215 * i * this.sx), this.centerY - 90 * this.sy, 'box');
            // this.boxes.push(box);
            this.responseImages.push(-1);
        }
    }
};



Activity1.prototype.showMismatch = function()
{
    
}

Activity1.prototype.showMatch = function()
{
    if (this.debug)
    {
        console.log('Show match');
    }

    // Menu button
    // var menuButton = this.game.add.button(window.innerWidth / 2 - 150 * this.sx, window.innerHeight / 2 - 70 * this.sy, 'button', this.toNext, this, 2, 1, 0);
    // menuButton.scale.setTo(0.9 * this.sx, 0.9 * this.sy);
    // var ship = this.game.add.sprite(menuButton.position.x + 35 * this.sx, menuButton.position.y + 75 * this.sy, 'ship');
    // ship.scale.setTo(0.15 * this.sx, 0.15 * this.sy);

    this.allButtons = [];
    this.requestImages = [];
    this.responseImages = [];
    this.boxes = [];
    this.shouldRespawn = false;
    
    this.gameComplete();
}

Activity1.prototype.isInside = function(object,box){
    if (object.x>box.x && object.y>box.y && object.x<(box.x + box.width) && object.y<(box.y + box.height))        return true;
    else return false;
}

/*
 * Callback Functions
 */

Activity1.prototype.resetImages = function()
{
    if (this.debug)
    {
        console.log("Reset images");
    }

    if (this.allButtons.length == 0)
    {
        return;
    }

    //TODO(Vadim)
    //publishRobotCommand(ROBOT_COMMAND_DO, true, "A1Encouragement", 
    //    "This string is spoken if the third paremter is an empty string or undefined");

    while (this.allButtons.length > 0)
    {
        this.allButtons[0].destroy();
        this.allButtons.splice(0, 1);
    }

    for (var i = 0; i < this.responseImages.length; ++i)
    {
        console.log(this.responseImages[i] + "-" + this.requestImages[i]);
        if (this.responseImages[i] != this.requestImages[i]) {
            this.responseImages[i] = -1;
            this.boxes[i].children = [];
        }
    }

    this.showOptions();
}

Activity1.prototype.checkImages = function()
{
    if (this.debug)
    {
        console.log('Check images');
        console.log('Images in bay: ' + this.responseImages.length);
        console.log('We want this many images: ' + this.requestImages.length);
    }

    var requestCopy = this.requestImages;

    var check = 0;
    while (check < requestCopy.length)
    {
        if (requestCopy[check] == -1)
        {
            requestCopy.splice(check, 1);
        }
        else
        {
            ++check;
        }
    }

    if (this.responseImages.length < requestCopy)
    {
        this.publishRobotGameFeedback();
        this.showMismatch();
        return;
    }

    for (var i = 0; i < this.responseImages.length; ++i)
    {
        if (this.responseImages[i] != requestCopy[i])
        {
            this.publishRobotGameFeedback();
            this.showMismatch();
            return;
        }
    }

    this.publishRobotSuccessMessage();
    this.showMatch();
}

Activity1.prototype.onDragStart = function(view)
{
    view.bringToTop();
    if (this.debug)
    {
        console.log("Drag started for view: " + view.key);
    }

    this.shouldRespawn = this.isInside(view, this.container);

    if (this.debug)
    {
        console.log('Started drag inside container: ' + this.shouldRespawn);
    }
}

Activity1.prototype.onDragStop = function(view)
{
    for (var i = 0; i < this.boxes.length; ++i)
    {
        if (this.isInside(view, this.boxes[i]) && this.responseImages[i] == -1)
        {
            this.responseImages[i] = view.key.slice(-1) - 1;
            this.boxes[i].children = [];
            this.boxes[i].addChild(this.game.make.sprite(this.boxes[i].width, this.boxes[i].height, view.key));
        }
    }

    view.destroy();
    this.spawnOption(view.key.slice(-1));
}


module.exports = Activity1;

},{"../../SARGame.js":1,"../../constants.js":2}],4:[function(require,module,exports){
/**
  * Alien Codes Activity 2
  * -----------------------------
  *      
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 2;

/**
 * Constructor for Activity 1
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity2(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.NUM_IMAGE_OPTIONS = 8;
    this.requestImagesCount;
    this.requestImages = [];
    this.requestBare = [];
    this.showingCount;
    this.showingImages = [];
    this.missingCount;
    this.missingImages = [];
    this.missingSpotBoxes = [];
    this.missingSpotGroups = [];
    this.response = [];
    this.allButtons = [];
    this.shouldRespawn = false;

    this.background;
    this.alien;
    this.bubble;
    this.container;
    this.leftSpacing;
    this.rightSpacing;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.centerX = window.innerWidth / 2;
    this.centerY = window.innerHeight / 2;
}
Activity2.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity2.prototype.preload = function() {
    this.game.load.image('background', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('alien', CONST.ASSETS_PATH + 'Items/Aliens/Alien2.png');

    this.game.load.image('option1', CONST.ASSETS_PATH + 'Items/Crystals/Crystal1.png');
    this.game.load.image('option2', CONST.ASSETS_PATH + 'Items/Crystals/Crystal2.png');
    this.game.load.image('option3', CONST.ASSETS_PATH + 'Items/Crystals/Crystal4.png');
    this.game.load.image('option4', CONST.ASSETS_PATH + 'Items/Crystals/Crystal5.png');
    this.game.load.image('option5', CONST.ASSETS_PATH + 'Items/Planets/Jupiter.png');
    this.game.load.image('option6', CONST.ASSETS_PATH + 'Items/Planets/Neptune.png');
    this.game.load.image('option7', CONST.ASSETS_PATH + 'Items/Rocks/Moonrock4.png');
    this.game.load.image('option8', CONST.ASSETS_PATH + 'Items/Rocks/Moonrock3.png');

    this.game.load.image('container', CONST.ASSETS_PATH + 'Items/BoxLong.png');
    this.game.load.image('display', CONST.ASSETS_PATH + 'Items/ControlPanel.png');
    this.game.load.image('box', CONST.ASSETS_PATH + 'Items/Box.png');

    this.game.load.image('bubble', CONST.ASSETS_PATH + 'Items/TextBubble.png');
    this.game.load.image('button', CONST.ASSETS_PATH + 'Items/Button.png');
    this.game.load.image('clear', CONST.ASSETS_PATH + 'Items/Clear.png');
    this.game.load.image('right', CONST.ASSETS_PATH + 'Items/Right.png');

    this.game.load.image('ship', CONST.ASSETS_PATH + 'Items/Spaceship.png');

    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity2.prototype.create  = function() {
    if (this.level == 1)
    {
        this.requestImagesCount = Math.floor(Math.random() * 3 + 2);
        this.missingCount = 1;
    }
    else if (this.level == 2)
    {
        this.requestImagesCount = Math.floor(Math.random() * 2 + 4);
        this.missingCount = Math.floor(Math.random() * 2 + 1);
    }
    else if (this.level == 3)
    {
        this.requestImagesCount = Math.ceil(Math.random() * 2 + 6);
        this.missingCount = Math.floor(Math.random() * 2 + 3);
    }

    for (var i = 0; i < this.missingCount; ++i)
    {
        this.response.push(-1);
    }

    this.showingCount = this.requestImagesCount - this.missingCount;

    var spacing = (this.NUM_IMAGE_OPTIONS - this.requestImagesCount) / 2;
    this.leftSpacing = Math.ceil(spacing);
    this.rightSpacing = Math.floor(spacing);

    this.showConstantImages();
    this.showChoices();

    this.requestRandomImages();
    this.showAlienRequest();
    this.requestShowingImages();
    this.showKnownImages();
    this.showMissingSpots();

    this.resize();

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.missingCount);
};


Activity2.prototype.requestRandomImages = function()
{
    for (var i = 0; i < this.leftSpacing; ++i)
    {
        this.requestImages.push(-1);
    }

    var allOptions = [];
    for (var i = 0; i < this.NUM_IMAGE_OPTIONS; ++i)
    {
        allOptions.push(i);
    }

    this.shuffle(allOptions);

    for (var i = 0; i < this.requestImagesCount; ++i)
    {
        this.requestImages.push(allOptions[i]);
        var value = {
            "Number": allOptions[i],
            "Position": i
        };
        this.requestBare.push(value);
        this.missingImages.push(value);
    }

    for (var i = 0; i < this.rightSpacing; ++i)
    {
        this.requestImages.push(-1);
    }
}

Activity2.prototype.requestShowingImages = function()
{
    var requestImagesCopy = this.requestBare;
    var count = 0;


    while (count < this.showingCount)
    {
        var randomImagePos = Math.floor(Math.random() * requestImagesCopy.length);
        var imageVal = requestImagesCopy[randomImagePos];

        this.showingImages.push(imageVal);
        requestImagesCopy.splice(randomImagePos, 1);
        this.missingImages.splice(randomImagePos, 1);
        ++count;
    }
}

Activity2.prototype.showConstantImages = function()
{
    this.background = this.game.add.sprite(0, 0, 'background');
    this.background.scale.setTo(this.sx, this.sy);

    // Reset button
    var resetButton = this.game.add.button(this.centerX - 550 * this.sx, this.centerY + 260 * this.sy, 'button', this.resetImages, this, 2, 1, 0);
    resetButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    var clear = this.game.add.sprite(this.centerX - 535 * this.sx, this.centerY + 285 * this.sy, 'clear');
    clear.scale.setTo(0.25 * this.sx, 0.25 * this.sy);

    // Verify button
    var verifyButton = this.game.add.button(this.centerX + 375 * this.sx, this.centerY + 260 * this.sy, 'button', this.checkImages, this, 2, 1, 0);
    verifyButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    var right = this.game.add.sprite(this.centerX + 400 * this.sx, this.centerY + 295 * this.sy, 'right');
    right.scale.setTo(0.1 * this.sx, 0.1 * this.sy);

    // Below is the box for feedback to the user.
    var display = this.game.add.sprite(this.centerX - 1075 * this.sx, this.centerY - 250 * this.sy, 'display');
    display.scale.setTo(1.125 * this.sx, 0.37 * this.sy);

    // Below are the options for the user to click on.
    this.container = this.game.add.sprite(this.centerX - 375 * this.sx, this.centerY + 200 * this.sy, 'container');
    this.container.scale.setTo(0.5 * this.sx, 0.6 * this.sy);

    // Below is the alien which prompts action from user.
    this.alien = this.game.add.sprite(this.centerX - this.centerX, this.centerY - 500 * this.sy, 'alien');
    this.alien.scale.setTo(0.45 * this.sx, 0.45 * this.sy);

    // Below is the bubble to signify the alien speaking.
    this.bubble = this.game.add.sprite(this.centerX - 760 * this.sx, this.centerY - 525 * this.sy, 'bubble');
    this.bubble.scale.setTo(2.4 * this.sx, 0.9 * this.sy);
}


 Activity2.prototype.showChoices = function()
 {
    for (var i = 1; i <= this.NUM_IMAGE_OPTIONS; ++i)
    {
        this.spawnOption(i);
    }
 }

 Activity2.prototype.showAlienRequest = function()
 {
    // Below are the Alien's requests.
    for (var i = 0; i < this.requestImages.length; ++i)
    {
        var imageToDisplay = this.requestImages[i] + 1;
        if (imageToDisplay > 0)
        {
            var image = this.game.add.sprite(this.centerX - (600 * this.sx) + (150 * i * this.sx), this.centerY - 420 * this.sy, 'option' + imageToDisplay);
            image.scale.setTo(0.25 * this.sx, 0.25 * this.sy);
            if (this.imageToDisplay == 8)
            {
                image.scale.setTo(0.22 * this.sx, 0.22 * this.sy);
            }
            else
            {
                image.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
            }
        }
    }
 }

 Activity2.prototype.showKnownImages = function()
 {
    for (var i = 0; i < this.showingImages.length; ++i)
    {
        var imageToDisplay = 'option' + (this.showingImages[i].Number + 1);
        var image = this.game.add.sprite(this.centerX - (850 * this.sx) + (215 * this.showingImages[i].Position * this.sx), window.innerHeight / 2 - 40, imageToDisplay);
        if (imageToDisplay == 8)
        {
            image.scale.setTo(0.22 * this.sx, 0.22 * this.sy);
        }
        else
        {
            image.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
        }
    }
 }

 Activity2.prototype.showMissingSpots = function()
 {
    for (var i = 0; i < this.missingImages.length; ++i)
    {
        var image = this.game.add.sprite(this.centerX - (850 * this.sx) + (215 * this.missingImages[i].Position * this.sx), this.centerY - 90 * this.sy, 'box');
        image.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
        this.missingSpotBoxes.push(image);
        var group = this.game.add.group();
        this.missingSpotGroups.push(group);
    }
 }

 Activity2.prototype.spawnOption = function(optionNumber)
 {
    var button;
    var width, height;
    if (optionNumber <= 4)
    {
        width = this.centerX - ((160 * optionNumber) * this.sx) + 360 * this.sx;
        height = this.centerY + 240 * this.sy;
    }
    else
    {
        width = this.centerX - (160 * (optionNumber - 4) * this.sx) + 360 * this.sx;
        height = this.centerY + 350 * this.sy;
    }

    button = this.game.add.button(width, height, 'option' + optionNumber, null, this, 2, 1, 0);
    if (optionNumber == 8)
    {
        button.scale.setTo(0.22 * this.sx, 0.22 * this.sy);
    }
    else
    {
        button.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
    }
    button.input.enableDrag(true);
    button.events.onDragStop.add(this.onDragStop, this);
    button.events.onDragStart.add(this.onDragStart, this);
    this.allButtons.push(button);
 }

Activity2.prototype.isInside = function(object,box){
    if (object.x>box.x && object.y>box.y && object.x<(box.x + box.width) && object.y<(box.y + box.height))        return true;
    else return false;
}

Activity2.prototype.showMismatch = function()
{

}

Activity2.prototype.showMatch = function()
{
    // Menu button
    // var menuButton = this.game.add.button(window.innerWidth / 2 - 150 * this.sx, window.innerHeight / 2 - 70 * this.sy, 'button', this.toNext, this, 2, 1, 0);
    // menuButton.scale.setTo(0.9 * this.sx, 0.9 * this.sy);
    // var ship = this.game.add.sprite(menuButton.position.x + 35 * this.sx, menuButton.position.y + 75 * this.sy, 'ship');
    // ship.scale.setTo(0.15 * this.sx, 0.15 * this.sy);

    this.requestImages = [];
    this.requestBare = [];
    this.showingCount;
    this.showingImages = [];
    this.missingCount;
    this.missingImages = [];
    this.missingSpotBoxes = [];
    this.missingSpotGroups = [];
    this.response = [];
    this.allButtons = [];
    this.shouldRespawn = false;

    this.gameComplete();
}

 /*
  * Callback Functions
  */

Activity2.prototype.resetImages = function()
{
    if (this.allButtons.length == 0)
    {
        return;
    }

    //TODO(Vadim) encouragement string
    // publishRobotCommand(ROBOT_COMMAND_DO, true, "A2Encouragement", 
    //     "This string is spoken if the third paremter is an empty string or undefined");

    while (this.allButtons.length > 0)
    {
        this.allButtons[0].destroy();
        this.allButtons.splice(0, 1);
    }

    for (var i = 0; i < this.response.length; ++i)
    {
        if (this.response[i] != this.missingImages[i].Number) {
            this.response[i] = -1;
            this.missingSpotBoxes[i].children = [];
        }
    }

    this.showChoices();
}

Activity2.prototype.checkImages = function()
{
    for (var i = 0; i < this.response.length; ++i)
    {
        if (this.response[i] != this.missingImages[i].Number)
        {
            this.publishRobotGameFeedback();
            this.showMismatch();
            return;
        }
    }

    this.publishRobotSuccessMessage();
    this.showMatch();
}

Activity2.prototype.onDragStart = function(view)
{
    view.bringToTop();

    this.shouldRespawn = this.isInside(view, this.container);
}

Activity2.prototype.onDragStop = function(view)
{
    for (var i = 0; i < this.missingSpotBoxes.length; ++i)
    {
        if (this.isInside(view, this.missingSpotBoxes[i]) && this.response[i] == -1)
        {
            this.missingSpotGroups[i].add(view);
            this.response[i] = view.key.slice(-1) - 1;

            this.missingSpotBoxes[i].children = [];
            this.missingSpotBoxes[i].addChild(this.game.make.sprite(this.missingSpotBoxes[i].width, this.missingSpotBoxes[i].height, view.key));
        }
    }

    view.destroy();
    this.spawnOption(view.key.slice(-1));
}

module.exports = Activity2;

},{"../../SARGame.js":1,"../../constants.js":2}],5:[function(require,module,exports){
/**
  * Alien Codes Activity 3
  * -----------------------------
  *      
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 3;

/**
 * Constructor for Activity 3
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity3(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.logoBackground;
    this.onPanel;
    this.bottomPanel;
    this.text;
    this.boxes = [];
    this.boxes_in_group = [];

    this.objectname = [];
    this.yanaSequenceObj = [];
    this.yukiSequenceObj = [];
    this.yanaHealthLevel = 0;
    this.yukiHealthLevel = 0;
    this.numGoodPills = 0;
    this.totalPills = 0;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.centerX = window.innerWidth / 2;
    this.centerY = window.innerHeight / 2;
}
Activity3.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity3.prototype.preload = function() {
    this.game.load.image('background', CONST.ASSETS_PATH + "Background/ShipInterior.png");
    this.game.load.image('controlPanel', CONST.ASSETS_PATH + "Items/ControlPanelcrop.png");
    this.game.load.image('box_long', CONST.ASSETS_PATH + "Items/BoxLong.png");
    this.game.load.image('alien', CONST.ASSETS_PATH + "Items/Aliens/Alien5.png");
    this.game.load.image('health_bar', CONST.ASSETS_PATH + "Items/ColorBoxes/BoxGreen.png");
    this.game.load.image('ship', CONST.ASSETS_PATH + 'Items/Spaceship.png');
    this.game.load.spritesheet('yana', CONST.ASSETS_PATH + "Items/Aliens/Yana_sheet.png", 730, 881, 2);
    this.game.load.spritesheet('yuki', CONST.ASSETS_PATH + "Items/Aliens/Yuki_sheet.png", 730, 881, 2);
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH + "Items/Rocks/Moonrock1_sheet.png", 610, 400, 2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH + "Items/Rocks/Moonrock2_sheet.png", 460, 360, 2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH + "Items/Rocks/Moonrock3_sheet.png", 660, 360, 2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH + "Items/Crystals/Crystal3_sheet.png", 410, 360, 2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH + "Items/Crystals/Crystal4_sheet.png", 410, 360, 2);
    this.game.load.spritesheet('object6', CONST.ASSETS_PATH + "Items/Crystals/Crystal5_sheet.png",410, 360, 2);
    this.game.load.spritesheet('object7', CONST.ASSETS_PATH + "Items/Stars/Star1_sheet.png", 410,410,2);
    this.game.load.spritesheet('object8', CONST.ASSETS_PATH + "Items/Stars/Star2_sheet.png", 410,410,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH + "Items/Button_sheet.png",288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH + "Items/Box_sheet.png", 881, 730, 2);

    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity3.prototype.create  = function() {
    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.missingCount);
    
    // mapping difficulty
    this.numGoodPills = 3;
    this.totalPills = this.level + this.numGoodPills;

    for (var i=0; i< this.totalPills; i++) {
        // set up all the object names
        this.objectname[i] = "object" + (i+1);
        this.yanaSequenceObj[i] = "object" + (i+1);
        this.yukiSequenceObj[i] = "object" + (i+1);
    }

    this.shuffle(this.objectname);

    // shuffle the sequences to random which pills are good.
    // only the first "this.numGoodPills" (i.e 3) pills are good for each alien sequence.
    this.shuffle(this.yanaSequenceObj);
    this.shuffle(this.yukiSequenceObj);

    this.logoBackground = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'background');
    this.logoBackground.anchor.setTo(0.5, 0.5);
                    
    var style = { font: "30px Arial", fill: "#ffffff", align: "center" };
    this.text = this.game.add.text(this.game.world.centerX, this.game.world.centerY, "Help me heal my space pets!", style);
    this.text.x = this.centerX - this.text.width * 0.5;
    this.text.y = this.centerY - this.text.height * 10 * this.sy;
                    
    this.bottomPanel = this.game.add.sprite(0,0, 'box_long');
    this.bottomPanel.scale.setTo(0.8*this.sx, 0.45*this.sy);
    this.bottomPanel.x = this.centerX - this.bottomPanel.width/2;
    this.bottomPanel.y = this.centerY + this.bottomPanel.height * 1.5 * this.sy;

    this.health_bar_yana = this.game.add.sprite(0,0, 'health_bar');
    this.health_bar_yuki = this.game.add.sprite(0,0, 'health_bar');
                        
    this.box1 = this.game.add.sprite(0,0, 'yana');
    this.box1.scale.setTo(0.48*this.sx,0.4*this.sy);
    this.box1.x = this.centerX - this.box1.width * 2.5 * this.sx;
    this.box1.y = this.centerY - this.box1.height * this.sy;
    this.box1.frame = 0;
    this.box1.visible = true;

    this.health_bar_yana.width = this.box1.width;

    this.health_bar_yana.max_height = this.box1.height;
    this.health_bar_yana.height = 0;
    this.health_bar_yana.x = this.box1.x;
    this.health_bar_yana.y = this.box1.y + this.box1.height;

    this.box2 = this.game.add.sprite(0,0, 'yuki');
    this.box2.scale.setTo(0.48*this.sx,0.4*this.sy);
    this.box2.x = this.centerX + this.box2.width * 0.75 * this.sx;
    this.box2.y = this.box1.y;
    this.box2.frame = 0;
    this.box2.visible = true;

    this.health_bar_yuki.width = this.box1.width;
    this.health_bar_yuki.max_height = this.box2.height;
    this.health_bar_yuki.height = 0;
    this.health_bar_yuki.x = this.box2.x;
    this.health_bar_yuki.y = this.box2.y + this.box2.height;
                    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
                        
    //generate each object on panel
    for (var i=0;i< this.totalPills ;i++){
            var tempx = this.bottomPanel.x + this.bottomPanel.width/2 - 400*this.sx +(i%this.totalPills) * 180*this.sx;
            var tempy = this.bottomPanel.y + this.bottomPanel.height/2  + Math.floor(i/this.totalPills) * 130*this.sy;
                            
            this.onPanel.create(tempx, tempy, this.objectname[i]);
    }
                    
    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
            object.frame=0;
            object.scale.setTo(0.25*this.sx);      
            object.inputEnabled=true;
            object.input.enableDrag();
            object.events.onDragStart.add(this.onDragStart,this);
            object.events.onDragStop.add(this.onDragStop,this);
    });
};

Activity3.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkCode();
    console.log("Button Pressed!");        
}
        
Activity3.prototype.onDragStart = function(sprite, pointer) {
    // we should create a new sprite, if they are dragging 
    //from inside the control panel
    if (this.isInside(sprite, this.bottomPanel)) {   
        var object = this.game.add.sprite(sprite.x, sprite.y, sprite.key);
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        object.inputEnabled=true;
        object.input.enableDrag();
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        this.onPanel.add(object);
    }
}

Activity3.prototype.onDragStop = function(sprite, pointer){
    sprite.frame = 0;
    if (this.isInside(sprite,this.box1)){
        var correct = false;
        for (var i = 0; i < this.numGoodPills; i++) {
            if (sprite.key == this.yanaSequenceObj[i]) {
                correct = true;
                console.log("Correct");
                // increase the health bar
                this.yanaHealthLevel = Math.min(100, this.yanaHealthLevel + 20);
                break;
            }
        }
        if (!correct) {
            console.log("False");
            this.yanaHealthLevel = Math.max(0, this.yanaHealthLevel - 10);
            this.publishRobotGameFeedback();
        }
        // update the health bar
        this.health_bar_yana.height = (this.yanaHealthLevel/100.0) * this.health_bar_yana.max_height;
        this.health_bar_yana.y = this.box1.y + this.box1.height - this.health_bar_yana.height;
                            
    }
    else if (this.isInside(sprite,this.box2)){
            var correct = false;
            for (var i = 0; i < this.numGoodPills; i++) {
                if (sprite.key == this.yukiSequenceObj[i]) {
                    correct = true;
                    this.yukiHealthLevel = Math.min(100, this.yukiHealthLevel+20); 
                    break;
                }
        }
            if (!correct) {
                this.yukiHealthLevel = Math.max(0, this.yukiHealthLevel-10);
                this.publishRobotGameFeedback();
            }
        // update the health bar
        this.health_bar_yuki.height = (this.yukiHealthLevel/100.0) * this.health_bar_yuki.max_height;
        this.health_bar_yuki.y = this.box2.y + this.box2.height - this.health_bar_yuki.height;
                            
    }
    sprite.destroy();
    if (this.yukiHealthLevel >= 100 && this.yanaHealthLevel >= 100) {
        this.text.setText("Hurray, you healed Yana and Yuki!");
        this.publishRobotSuccessMessage();
        this.showMatch();
    }
    else {
        //TODO(Vadim): encouragement message
    }
}

Activity3.prototype.isInside = function(object,box){
    if (object.x>box.x+0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height)    return true;
    else return false;
}

Activity3.prototype.showMatch = function()
{
    console.log("Pets all healed!");

    // Dynamically show "go to next activity" button
    // var menuButton = this.game.add.button(window.innerWidth / 2 - 50, window.innerHeight / 2 - 70, 'button', this.toNext, this, 2, 1, 0);
    // menuButton.scale.setTo(0.9 * sx, 0.9 * sy);
    // menuButton.x = centerX - menuButton.width/2;
    // menuButton.y = centerY - menuButton.height/2;
    // var ship = game.add.sprite(menuButton.position.x + 35 * sx, menuButton.position.y + 75 * sy, 'ship');
    // ship.scale.setTo(0.15 * sx, 0.15 * sy);
    this.gameComplete();
}

module.exports = Activity3;

},{"../../SARGame.js":1,"../../constants.js":2}],6:[function(require,module,exports){
/**
  * Alien Codes Activity 4
  * -----------------------------
  *      
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 4;

/**
 * Constructor for Activity 1
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity4(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.logoBackground;
    this.onPanel;
    this.bottomPanel;
    this.text;
    this.boxes = [];

    this.objectname = [];
    this.sequenceObj = [];
    this.totalObjects = 0;
    this.numCodes = 0;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.centerX = window.innerWidth / 2;
    this.centerY = window.innerHeight / 2;
}
Activity4.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity4.prototype.preload = function() {
    this.game.load.image('background', CONST.ASSETS_PATH + 'Background/HomeScreen.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH + "Items/ControlPanelcrop.png");
    this.game.load.image('box_long', CONST.ASSETS_PATH + "Items/BoxLong.png");
    this.game.load.image('speech_bubble', CONST.ASSETS_PATH + "Items/TextBubble.png");
    this.game.load.image('alien', CONST.ASSETS_PATH + "Items/Aliens/Alien5.png");
    this.game.load.image('ship', CONST.ASSETS_PATH + 'Items/Spaceship.png');
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH + "Items/Rocks/Moonrock1_sheet.png", 610, 400, 2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH + "Items/Rocks/Moonrock2_sheet.png", 460, 360, 2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH + "Items/Crystals/Crystal3_sheet.png", 410, 360, 2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH + "Items/Crystals/Crystal4_sheet.png", 410, 360, 2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH + "Items/Crystals/Crystal5_sheet.png",410, 360, 2);
    this.game.load.spritesheet('object6', CONST.ASSETS_PATH + "Items/Stars/Star1_sheet.png", 410,410,2);
    this.game.load.spritesheet('object7', CONST.ASSETS_PATH + "Items/Stars/Star2_sheet.png", 410,410,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH + "Items/Box_sheet.png", 881, 730, 2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH + "Items/Button_sheet.png",288,288,2);
    this.game.load.spritesheet('bigBox', CONST.ASSETS_PATH + "Items/Box_sheet.png",1510,510,2);

    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity4.prototype.create  = function() {
    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.missingCount);
    
    // mapping difficulty [1-3] to totalObjects [3-5]
    this.totalObjects = this.level + 2;
    this.numCodes = this.level;
    console.log("this.numCodes2 before:" + this.numCodes);

    for (var i=0; i< this.totalObjects; i++) {
        this.objectname[i] = "object" + (i+1);
    }
    this.shuffle(this.objectname);

    // creating the code sequence
    for (var i=0; i< this.numCodes; i++) {
        this.sequenceObj[i] = "object" + this.game.rnd.integerInRange(1,this.totalObjects);
    }
    this.shuffle(this.sequenceObj);

    var logoBackground = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'background');
    logoBackground.anchor.setTo(0.5, 0.5);

    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.8*this.sx,0.3*this.sy);
    this.controlPanel.x = this.centerX - this.controlPanel.width/2;
    this.controlPanel.y =  this.centerY - this.controlPanel.height;

    var textbubble = this.game.add.sprite(0,0, 'speech_bubble');
    textbubble.scale.setTo(1.8 * this.sx, 0.6 * this.sy);
    textbubble.x = this.centerX - textbubble.width * 0.5;
    textbubble.y = this.controlPanel.y - textbubble.height;

    var style = { font: "30px Arial", fill: "#000000", align: "center" };
    this.text = this.game.add.text(this.centerX, this.centerY, "Help me crack the code to get in the spaceship!", style);
    this.text.x = this.centerX - this.text.width/2;
    this.text.y = textbubble.y + textbubble.height/2.5 * this.sy;

    var alien = this.game.add.sprite(0,0, 'alien');
    alien.scale.setTo(0.3, 0.3);
    alien.x = textbubble.x - alien.width;
    alien.y = textbubble.y;

    this.bottomPanel = this.game.add.sprite(0,0, 'box_long');
    this.bottomPanel.scale.setTo(0.8*this.sx, 0.45*this.sy);
    this.bottomPanel.x = this.game.world.centerX - this.bottomPanel.width/2;
    //bottomPanel.y = controlPanel.y + controlPanel.height + bottomPanel.height * (0.25 * sy);
    this.bottomPanel.y = this.game.world.centerY + this.bottomPanel.height/2;
    this.button = this.game.add.sprite(0,0,'button');
    this.button.scale.setTo(0.5*this.sx, 0.5*this.sy);
    this.button.x = this.centerX - this.button.width/2;
    this.button.y = this.bottomPanel.y + this.bottomPanel.height + this.button.height * (0.25 * this.sy);
    this.button.frame=0;
    this.button.inputEnabled=true;
    this.button.events.onInputDown.add(this.onButtonPress,this);

    // the following chunk of code is to center the boxes onto the control panel based on how many boxes there are. 

    // the offset is the small wire stuff at the two ends of the control panel image.
    // we want to take that away from our calculations of where the control panel
    // actually starts to position the boxes correctly.
    var offset = this.controlPanel.width / 12;

    // the actual width of the controlPanel for positioning purposes.
    var width = this.controlPanel.width - offset * 2;

    // how much room is given for each box based on how many boxes there are.
    var ratio = width / this.numCodes;

    for (var i=0; i<this.numCodes; i++) {
        this.boxes[i] = this.game.add.sprite(0,0,'box');                
        this.boxes[i].scale.setTo(0.3*this.sx,0.3*this.sy);
        // to center the boxes on the control panel
        this.boxes[i].x = this.controlPanel.x + offset + ((i+1) * ratio) -  (ratio * 0.5) - this.boxes[i].width/2;
        this.boxes[i].y = this.controlPanel.centerY - this.boxes[i].height/2;
        this.boxes[i].frame = 0;
        this.boxes[i].visible = true;
    }

    this.onPanel = this.game.add.group();

    //generate each object on panel
    for (var i=0;i< this.totalObjects ;i++){
        var tempx = this.bottomPanel.x + this.bottomPanel.width/2 - 400*this.sx +(i%5) * 180*this.sx;
        var tempy = this.bottomPanel.y + this.bottomPanel.height/4  + Math.floor(i/5) * 130*this.sy;
        console.log("creating " + this.objectname[i]);
        this.onPanel.create(tempx, tempy, this.objectname[i]);
    }

    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        object.inputEnabled=true;
        object.input.enableDrag();
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
    });
};

Activity4.prototype.isInside = function(object,box){
    if (object.x>box.x+0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height)        return true;
    else return false;
}

// Check whether the correct code is in each box, will turn the box green if correct.
Activity4.prototype.checkCode = function() {
    console.log("In check code");
    var count = 0;
    for (var i=0; i< this.boxes.length; i++) {
        console.log("target " + this.sequenceObj[i]);
        if (this.boxes[i].children[0] != null)
            console.log("actual " + this.boxes[i].children[0].key);
        if (this.boxes[i].children[0] != null && this.boxes[i].children[0].key == this.sequenceObj[i]) {
            // turn the box green and disable that box/item.
            this.boxes[i].tint = 0x50D050;
            count++;
        } else {
            this.boxes[i].children = [];
        }
    }

    if (count == this.numCodes) {
        // The player correctly guessed the code!
        this.text.setText("Hurray, you opened the space ship!");
        this.publishRobotSuccessMessage();
        this.showMatch();
    }
    else {
        // Player incorrecly guessed the code, send an encouragement message.
        this.publishRobotGameFeedback();
    }
}

Activity4.prototype.showMatch = function()
{
    console.log("Spaceship code cracked!");
    // Dynamically show "go to next activity" button
    // var menuButton = game.add.button(window.innerWidth / 2 - 50, window.innerHeight / 2 - 70, 'button', this.toNext, this, 2, 1, 0);
    // menuButton.scale.setTo(0.9 * sx, 0.9 * sy);
    // menuButton.x = centerX - menuButton.width/2;
    // menuButton.y = centerY - menuButton.height/2;
    // var ship = game.add.sprite(menuButton.position.x + 35 * sx, menuButton.position.y + 75 * sy, 'ship');
    // ship.scale.setTo(0.15 * sx, 0.15 * sy);
    this.gameComplete();
}

/*
* Callback functions
*/

Activity4.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkCode();
    console.log("Button Pressed!");
}

Activity4.prototype.onDragStart = function(sprite, pointer) {
    // we should create a new sprite, if they are dragging 
    //from inside the control panel
    if (this.isInside(sprite, this.bottomPanel)) {     
        var object = this.game.add.sprite(sprite.x, sprite.y, sprite.key);
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        object.inputEnabled=true;
        object.input.enableDrag();
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        this.onPanel.add(object);
    }
}

Activity4.prototype.onDragStop = function(sprite, pointer){
    sprite.frame = 0;
    var in_a_box = false;
    for (var i=0; i < this.boxes.length; i++) {
        // only add the sprite to the box if there wasn't a sprite already in that box.
        if (this.boxes[i].children.length == 0 && this.isInside(sprite,this.boxes[i])){
                this.onPanel.removeChild(sprite);
                this.boxes[i].addChild(this.game.make.sprite(this.boxes[i].width, this.boxes[i].height, sprite.key));
                sprite.destroy();
                in_a_box = true;
                break;
            } 
    }
    if (!in_a_box) {
    // if the sprite did not end up in a box, destroy it.
        console.log("Removing sprite");
        sprite.destroy();
    }
}

module.exports = Activity4;

},{"../../SARGame.js":1,"../../constants.js":2}],7:[function(require,module,exports){
/**
  * Alien Codes Activity 5
  * -----------------------------
  *      
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 5;

/**
 * Constructor for Activity 1
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity5(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    
    this.box;
    this.container;
    this.background;
    this.alien;
    this.bubble;
    this.leftSpacing;
    this.rightSpacing;

    this.NUM_IMAGE_OPTIONS = 6;
    this.requestImagesCount = 0;
    this.imageOptions = [];
    this.showImageNum = [];
    this.imagesShown = 0;
    this.allBoxes = [];
    this.valueInBoxes = [];
    this.wordsDisplayed = [];
    this.mapping = {
        "option1":"Crystal", 
        "option2":"Moon",
        "option3":"Saturn",
        "option4":"Rock",
        "option5":"Star",
        "option6":"Alien"
    };

    this.style = { font: "30px Arial", fill: "#000000", align: "center" };

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.centerX = window.innerWidth / 2;
    this.centerY = window.innerHeight / 2;
}
Activity5.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity5.prototype.preload = function() {
    this.game.load.image('background', CONST.ASSETS_PATH + 'Background/ShipInterior.png', window.innerWidth, window.innerHeight);
    this.game.load.image('alien', CONST.ASSETS_PATH + 'Items/Aliens/Alien6.png');

    this.game.load.image('option1', CONST.ASSETS_PATH + 'Items/Crystals/Crystal3Glow.png');
    this.game.load.image('option2', CONST.ASSETS_PATH + 'Items/Planets/Moon.png');
    this.game.load.image('option3', CONST.ASSETS_PATH + 'Items/Planets/Saturn.png');
    this.game.load.image('option4', CONST.ASSETS_PATH + 'Items/Rocks/Moonrock5.png');
    this.game.load.image('option5', CONST.ASSETS_PATH + 'Items/Stars/Star4Glow.png');
    this.game.load.image('option6', CONST.ASSETS_PATH + 'Items/Aliens/Alien8.png');

    this.game.load.image('container', CONST.ASSETS_PATH + 'Items/BoxLong.png');
    this.game.load.image('box', CONST.ASSETS_PATH + 'Items/Box.png');
    this.game.load.image('display', CONST.ASSETS_PATH + 'Items/ControlPanel.png');

    this.game.load.image('bubble', CONST.ASSETS_PATH + 'Items/TextBubble.png');
    this.game.load.image('button', CONST.ASSETS_PATH + 'Items/Button.png');
    this.game.load.image('reset', CONST.ASSETS_PATH + 'Items/Clear.png');
    this.game.load.image('right', CONST.ASSETS_PATH + 'Items/Right.png');

    this.game.load.image('galaxy', CONST.ASSETS_PATH + 'Items/Galaxies/Galaxy1.png');
    this.game.load.image('ship', CONST.ASSETS_PATH + 'Items/Spaceship.png');

    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity5.prototype.create  = function() {
    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.missingCount);
    
    if (this.level == 1)
    {
        this.requestImagesCount = Math.floor(Math.random() * 2 + 1);
    }
    else if (this.level == 2)
    {
        this.requestImagesCount = Math.floor(Math.random() * 3 + 2);
    }
    else if (this.level == 3)
    {
        this.requestImagesCount = Math.floor(Math.random()* 2 + 4);
    }

    var spacing = (this.NUM_IMAGE_OPTIONS - this.requestImagesCount) / 2;
    this.leftSpacing = Math.ceil(spacing);
    this.rightSpacing = Math.floor(spacing);

    this.showConstantImages();
    this.requestRandomOrder();
    this.showAlienRequest();
    this.showOptions();

    this.showBoxes();
    this.showText();
};

/*
 * Helper Functions
 */

Activity5.prototype.showConstantImages = function()
{
    this.background = this.game.add.sprite(0, 0, 'background');
    this.background.scale.setTo(this.sx, this.sy);

    // Reset button
    var resetButton = this.game.add.button(this.centerX - 550 * this.sx, this.centerY + 260 * this.sy, 'button', this.resetImages, this, 2, 1, 0);
    resetButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    var clear = this.game.add.sprite(this.centerX - 535 * this.sx, this.centerY + 285 * this.sy, 'reset');
    clear.scale.setTo(0.25 * this.sx, 0.25 * this.sy);

    // Verify button
    var verifyButton = this.game.add.button(this.centerX + 375 * this.sx, this.centerY + 260 * this.sy, 'button', this.checkImages, this, 2, 1, 0);
    verifyButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    var right = this.game.add.sprite(this.centerX + 400 * this.sx, this.centerY + 295 * this.sy, 'right');
    right.scale.setTo(0.1 * this.sx, 0.1 * this.sy);

    // Below is the box for feedback to the user.
    var display = this.game.add.sprite(this.centerX - 1075 * this.sx, this.centerY - 250 * this.sy, 'display');
    display.scale.setTo(1.125 * this.sx, 0.37 * this.sy);

    // Below are the options for the user to click on.
    this.container = this.game.add.sprite(this.centerX - 375 * this.sx, this.centerY + 200 * this.sy, 'container');
    this.container.scale.setTo(0.5 * this.sx, 0.6 * this.sy);

    // Below is the alien which prompts action from user.
    this.alien = this.game.add.sprite(this.centerX - this.centerX, this.centerY - 500 * this.sy, 'alien');
    this.alien.scale.setTo(0.45 * this.sx, 0.45 * this.sy);

    // Below is the bubble to signify the alien speaking.
    this.bubble = this.game.add.sprite(this.centerX - 760 * this.sx, this.centerY - 525 * this.sy, 'bubble');
    this.bubble.scale.setTo(2.4 * this.sx, 0.9 * this.sy);
}

Activity5.prototype.showAlienRequest = function()
{
    var galaxy = this.game.add.sprite(this.centerX - 400 * this.sx, this.centerY - 460 * this.sy, 'galaxy');
    galaxy.scale.setTo(0.3 * this.sx, 0.3 * this.sy);

    var rightDemo = this.game.add.sprite(this.centerX - 100 * this.sx, this.centerY - 420 * this.sy, 'right');
    rightDemo.scale.setTo(0.2 * this.sx, 0.1 * this.sy);

    var box = this.game.add.sprite(this.centerX + 150 * this.sx, this.centerY - 490 * this.sy, 'box');
    box.scale.setTo(0.3 * this.sx, 0.3 * this.sy);
    this.game.add.text(box.position.x + 25 * this.sx, box.position.y + 150 * this.sy, "Galaxy", this.style);

    var smallGalaxy = this.game.add.sprite(box.position.x + 45 * this.sx, this.centerY - 440 * this.sy, 'galaxy');
    smallGalaxy.scale.setTo(0.15 * this.sx, 0.15 * this.sy);
}

Activity5.prototype.requestRandomOrder = function()
{
    for (var i = 0; i < this.NUM_IMAGE_OPTIONS; ++i)
    {
        this.showImageNum.push(false);
    }

    var showArray = [];
    for (var i = 0; i < this.NUM_IMAGE_OPTIONS; ++i)
    {
        showArray.push(i);
    }

    this.shuffle(showArray);

    for (var i = 0; i < this.requestImagesCount; ++i)
    {
        this.showImageNum[showArray[i]] = true;
        this.wordsDisplayed.push(this.mapping['option' + (showArray[i] + 1)]);
    }
}

Activity5.prototype.showOptions = function()
{
    for (var i = 0; i < this.showImageNum.length; ++i)
    {
        if (this.showImageNum[i])
        {
            this.spawnOption(i + 1);
        }
    }
}

Activity5.prototype.spawnOption = function(optionNumber)
{
    var button;
    var width, height;

    width = this.centerX - 475 * this.sx + 120 * (this.imagesShown + this.leftSpacing) * this.sx;
    height = this.centerY + 300 * this.sy;

    button = this.game.add.button(width, height, 'option' + optionNumber, null, this, 2, 1, 0);
    button.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
    button.input.enableDrag(true);
    button.events.onDragStart.add(this.onDragStart, this);
    button.events.onDragStop.add(this.onDragStop, this);

    this.imageOptions.push(button);
    ++this.imagesShown;
}

Activity5.prototype.showBoxes = function()
{
    for (var i = 0; i < this.requestImagesCount; ++i)
    {
        var box = this.game.add.sprite(this.centerX - 800 * this.sx + 300 * i * this.sx, this.centerY - 100 * this.sy, 'box');
        box.scale.setTo(0.3 * this.sx, 0.3 * this.sy);
        this.allBoxes.push(box);
        this.valueInBoxes.push("EMPTY");
    }
}

Activity5.prototype.showText = function()
{
    var wordsDisplayedCopy = this.wordsDisplayed;
    this.shuffle(wordsDisplayedCopy);

    for (var i = 0; i < this.allBoxes.length; ++i)
    {
        var boxPosition = this.allBoxes[i].position;
        var text = this.game.add.text(boxPosition.x + 25 * this.sx, boxPosition.y + 150 * this.sy, wordsDisplayedCopy[i], this.style);
    }

    this.wordsDisplayed = wordsDisplayedCopy;
}

Activity5.prototype.showMismatch = function()
{
    console.log('Mismatch!');
}

Activity5.prototype.showMatch = function()
{
    // Menu button
    this.requestImagesCount = 0;
    this.imageOptions = [];
    this.showImageNum = [];
    this.imagesShown = 0;
    this.allBoxes = [];
    this.valueInBoxes = [];
    this.wordsDisplayed = [];

    this.gameComplete();
}

Activity5.prototype.isInside = function(object,box)
{
    return Phaser.Rectangle.containsRect(object, box);
}

/*
 * Callback Functions
 */

Activity5.prototype.resetOptions = function()
{
    if (this.imageOptions.length == 0)
    {
        return;
    }

    //TODO(Vadim): encouragement string

    while (this.imageOptions.length > 0)
    {
        this.imageOptions[0].destroy();
        this.imageOptions.splice(0, 1);
    }

    this.imagesShown = 0;
    this.showOptions();
}

Activity5.prototype.resetImages = function() {
    this.resetOptions();
    for (var i = 0; i < this.wordsDisplayed.length; ++i)
    {
        if (this.mapping[this.valueInBoxes[i]] != this.wordsDisplayed[i])
        {
            this.allBoxes[i].children = [];
        }
    }
}

Activity5.prototype.checkImages = function()
{
    var incorrect = false;
    for (var i = 0; i < this.wordsDisplayed.length; ++i)
    {
        if (this.mapping[this.valueInBoxes[i]] != this.wordsDisplayed[i])
        {
            console.log('Incorrect match');
            this.allBoxes[i].children = [];
            incorrect = true;
        }
    }
    if (incorrect) {
        this.publishRobotGameFeedback();
        this.showMismatch();
        this.resetOptions();
    } else {
        this.publishRobotSuccessMessage();
        this.showMatch();
    }
}

Activity5.prototype.onDragStart = function(view)
{
    view.bringToTop();
}

Activity5.prototype.onDragStop = function(view)
{
    var keyValue = view.key;
    for (var i = 0; i < this.allBoxes.length; ++i)
    {
        if (this.isInside(view, this.allBoxes[i]))
        {
            this.valueInBoxes[i] = keyValue;
            this.allBoxes[i].addChild(this.game.make.sprite(this.allBoxes[i].width, this.allBoxes[i].height, view.key));
        } else if (this.valueInBoxes[i] == keyValue) {
            this.valueInBoxes[i] = "EMPTY";
        } 
        view.destroy();
        this.resetOptions();
    }
}

module.exports = Activity5;

},{"../../SARGame.js":1,"../../constants.js":2}],8:[function(require,module,exports){
'use strict';

var Activity1 = require('./activity1.js');
var Activity2 = require('./activity2.js');
var Activity3 = require('./activity3.js');
var Activity4 = require('./activity4.js');
var Activity5 = require('./activity5.js');
var Activity6 = require('./activity1.js');
var Activity7 = require('./activity2.js');
var Activity8 = require('./activity3.js');
var Activity9 = require('./activity4.js');
var Activity10 = require('./activity5.js');

var GameWrapper = require('../../SARGame.js').GameWrapper;
var CONST = require('../../constants.js');


function AlienCodes(ros, level, exitCallBack) {
    GameWrapper.call(this, ros, level, exitCallBack);
    this.gameid = CONST.Games.ALIEN_CODES;
    this.activities = [Activity1, Activity2, Activity3, Activity4, Activity5, Activity6, Activity7, Activity8, Activity9, Activity10];
    // this.activities = [Activity1, Activity2];
    this.start();
}
AlienCodes.prototype.__proto__ = GameWrapper.prototype;


module.exports = AlienCodes;

},{"../../SARGame.js":1,"../../constants.js":2,"./activity1.js":3,"./activity2.js":4,"./activity3.js":5,"./activity4.js":6,"./activity5.js":7}],9:[function(require,module,exports){
/**
 * Galactic Traveler Activity 2
 * -----------------------------
 *      Charge spaceship with [1-3, 4-6, 7-9] energy crystals. The level determines the number of crystals needed to
 *      charge the spaceship.
 *
 */

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');


const ROCK_AMOUNT = 10;
const COLUMNS_PANEL = 4;
const COLUMNS_BOX = 4;
const ACTIVITY_ID = 1;

/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity1(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920,
        gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onBox;
    this.box;
    this.logo;
    this.button;
    this.controlPanel;
    this.scoreText;
    this.promptText;
    this.target;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.logo;
    this.controlPanel;
    this.button;
    this.dragging = false;
    this.draggingSecond = false;

    //create variable panel with two coordinates properties
    this.panel = {
      CoordX : new Array(),
      CoordY : new Array()
    };
}
Activity1.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity1.prototype.preload = function() {
    // Load static image assets
    this.game.load.image('logo', '../../assets/Background/ShipInterior.png'); // logo means background image
    this.game.load.image('controlPanel', CONST.ASSETS_PATH + 'Items/ControlPanelcrop.png'); // frame box where crystals located

    // Load sprites
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Rocks/Moonrock1_sheet.png',610,400,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Rocks/Moonrock2_sheet.png',460,360,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Rocks/Moonrock3_sheet.png',660,360,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Rocks/Moonrock4_sheet.png',510,410,2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH+'Items/Rocks/Moonrock5_sheet.png',660,360,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);

    // Number of moonrocks needed to be in box randomly generated
    this.target = this.generateTarget();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets
 */
Activity1.prototype.create = function() {
    // background image is displayed
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    //  Moves the image anchor to the middle, so it centers inside the game properly
    this.logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+ this.game.width);
    console.log('Game height is'+ this.game.height);

    // controlPanel where moonrock located image is created on screen
    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;


    //button image is loaded
    this.button = this.game.add.sprite(this.controlPanel.x + this.controlPanel.width/2,
                                       this.controlPanel.y + this.controlPanel.height-200, 'button');
    this.button.scale.setTo(0.3);
    this.button.x = this.controlPanel.x + this.controlPanel.width / 2 - this.button.width / 2;
    this.button.y = this.controlPanel.y + this.controlPanel.height - this.button.height-0.05 * this.game.height;
    //button frame 
    this.button.frame=0;
    //Enables all kind of input actions on this image (click, etc)
    this.button.inputEnabled=true;
    //onButtonPress function
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);

    this.box = this.game.add.sprite(0,0, 'box');
    this.box.scale.setTo(0.6 * this.sx, 0.8 * this.sy);
    this.box.x = this.controlPanel.x + this.controlPanel.width - this.box.width - 130 * this.sx;
    this.box.y = this.controlPanel.y + 80 * this.sx;
    this.box.inputEnabled = true;
    this.box.frame = 0;

    //Custom properties, CoordX and CoordY, added to object box1
    this.box.CoordX = new Array();
    this.box.CoordY = new Array();

    console.log('Box height is '+ this.box.height);
    console.log('Box width is '+ this.box.width);

    console.log('cp width and height are '+ this.controlPanel.width + 'x' + this.controlPanel.height);

    this.onPanel = this.game.add.group();
    this.onBox = this.game.add.group();


    for (var i=0;i<ROCK_AMOUNT;i++){
        var objectname = "object" + this.game.rnd.integerInRange(1,5);
        this.panel.CoordX[i] = this.controlPanel.x + 220 * this.sx + (i % COLUMNS_PANEL) * 150 * this.sx;
        this.panel.CoordY[i] = this.controlPanel.y + 130 * this.sy + Math.floor(i / COLUMNS_PANEL) * 150 * this.sy;

        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], objectname);
    }

    // set the rocks coordinate/ position in the box
    this.boxCoordinates();

    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame = 0;
        object.scale.setTo(0.25 * this.sx);
        //object.originalPosition = object.position.clone();
        //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
        object.inputEnabled = true;
        object.input.enableDrag(false,false,false,255,null, this.controlPanel);
        //object.input.enableSnap(130 *sx,150 *sy,false,true);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });


    this.scoreText = this.game.add.text(this.game.width / 2, 0.1 * this.game.height, 'Count: 0', {fontSize:'24px',fill:'#000'});
    this.scoreText.x = this.game.width / 2 - this.scoreText.width / 2;

    this.promptText = this.game.add.text(this.game.width / 2, 0.03 * this.game.height,'Please move moonrocks into the box',
                                         {fontSize:'24px',fill:'#fff'});
    this.promptText.x = this.game.width / 2 - this.promptText.width / 2;
    var question = 'Please move ' + this.target + ' moonrocks into the box';
    this.promptText.setText(question);

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.target);
};

Activity1.prototype.boxCoordinates = function() {
    for (var i = 0; i < ROCK_AMOUNT; i++)
    {
        this.box.CoordX[i] = this.controlPanel.x + 960 * this.sx + i%3*150*this.sx;
        this.box.CoordY[i] = this.controlPanel.y + 160 * this.sy + Math.floor(i/3)*100*this.sy;
    }
};

Activity1.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
};

Activity1.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.stat = this.countRocks();
    if (!this.stat[0]) this.attempts++;

    if (this.stat[0]){
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
    else {
        var feedback = "That doesn't look right. ";
        if (this.stat[1] == 'less') {
            feedback += "Do we need more?";
        }
        else if (this.stat[1] == 'more') {
            feedback += "Is that too many?";
        }
        // provide specific feedback on the mistake
        this.publishRobotGameFeedback(this.target, this.stat[1] == 'less' ? 'more': 'fewer');
    }
};

Activity1.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
};

Activity1.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;

    if (this.dragging) 
        this.draggingSecond = true;
    this.dragging = true;
};

Activity1.prototype.onDragStop = function(sprite,pointer){
    sprite.frame=0;
    this.box.frame = 0;
                
        if (this.isInside(sprite) && this.onPanel.removeChild(sprite))
        {
            this.onBox.add(sprite);
        }
        else if (!this.isInside(sprite) && this.onBox.removeChild(sprite))
        {
            this.onPanel.add(sprite);
        }
        console.log("onPanel.total: " + this.onPanel.total + " onBox.total: " + this.onBox.total);
    
    if (this.draggingSecond)
        this.draggingSecond = false;
    else {
        this.dragging = false
        //Reorders the rocks on the Panel
        for (var i = 0; i < this.onPanel.total; i++)
        {
            this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
            this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
        }
        
        //Reorders the rocks on the Box
        for (var i = 0; i < this.onBox.total; i++)
        {
            this.onBox.getChildAt(i).x = this.box.CoordX[i];
            this.onBox.getChildAt(i).y = this.box.CoordY[i];
        }
    }
}

Activity1.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite)) this.box.frame = 1;
    else if (!this.isInside(sprite)){
        this.box.frame = 0;
    }
}

Activity1.prototype.isInside = function(object){
    if (object.x > this.box.x+0.01*this.game.width && 
        object.y > this.box.y +0.01*this.game.height && 
        object.x < this.box.x+this.box.width-0.01*this.game.width && 
        object.y < this.box.y+this.box.height-0.01*this.game.height) return true;
    else return false;
}

Activity1.prototype.countRocks = function(){
    console.log("Count in box! : " + this.onBox.total);

    this.scoreText.setText('Count: '+ this.onBox.total);
    if (this.onBox.total==this.target){
        //disable the drag properties
        this.onBox.forEach(function(object){object.input.draggable = false;});
        this.onPanel.forEach(function(object){object.input.draggable = false;});
        //game.input.mouse.enabled = false;
        return [true, true];
    } else 
        return [false,(this.onBox.total<this.target)?"less":"more"];
}

Activity1.prototype.generateTarget = function() {
    if (this.level == 1) {
        return this.game.rnd.integerInRange(1,3);
    } else if (this.level == 2) {
        return this.game.rnd.integerInRange(4,6);
    } else if (this.level == 3) {
        return this.game.rnd.integerInRange(7,9);
    } else {
        throw CONST.UNEXPECTED_LEVEL;
    }
}


module.exports = Activity1;

},{"../../SARGame.js":1,"../../constants.js":2}],10:[function(require,module,exports){
/**
 * Galactic Traveler Activity 2
 * -----------------------------
 *      Charge spaceship with [1-3, 4-6, 7-9] energy crystals. The level determines the number of crystals needed to
 *      charge the spaceship.
 *
 */

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');


const COLUMNS_PANEL = 4;
const COLUMNS_BATTERY = 4;

const ACTIVITY_ID = 2;

/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity2(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920,
        gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onBattery;
    this.box;
    this.battery;
    this.count = 0;
    this.target;
    this.numCrystals = 10;
    this.scoreText;
    this.promptText;
    this.dragging = false;
    this.draggingSecond = false;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.logo;
    this.controlPanel;
    this.button;

    //create variable panel with two coordinates properties
    this.panel = {
      CoordX : new Array(),
      CoordY : new Array()
    };
}
Activity2.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity2.prototype.preload = function() {
    // Load static image assets
    this.game.load.image('logo', '../../assets/Background/ShipInterior.png'); // logo means background image
    this.game.load.image('controlPanel', CONST.ASSETS_PATH + 'Items/ControlPanelcrop.png'); // frame box where crystals located

    // Load sprites
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH + 'Items/Crystals/Crystal1_sheet.png',460,560,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH + 'Items/Crystals/Crystal2_sheet.png',460,510,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH + 'Items/Crystals/Crystal3_sheet.png',410,460,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH + 'Items/Crystals/Crystal4_sheet.png',460,410,2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH + 'Items/Crystals/Crystal5_sheet.png',410,360,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH + 'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('battery', CONST.ASSETS_PATH + 'Items/Battery_sheet.png',560,810,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH + 'Items/Box_sheet.png',881,730,2);

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets
 */
Activity2.prototype.create = function() {
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    this.logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is ' + this.game.width);
    console.log('Game height is ' + this.game.height);

    // Control Panel
    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.95 * this.sx, 0.95 * this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;

    // Button
    this.button = this.game.add.sprite(this.controlPanel.x+this.controlPanel.width/2,this.controlPanel.y+this.controlPanel.height-200,'button');
    this.button.scale.setTo(0.4*this.sx);
    this.button.scaleMin = 0.3;
    this.button.x = this.controlPanel.x+this.controlPanel.width/2-this.button.width/2;
    this.button.y = this.controlPanel.y+this.controlPanel.height-this.button.height-0.1*this.game.height;
    this.button.frame=0;
    this.button.inputEnabled=true;
    this.button.events.onInputDown.add(this.onButtonPress.bind(this), this);
    this.button.events.onInputUp.add(onButtonRelease,this);

    // Battery
    this.battery = this.game.add.sprite(0,0, 'battery');
    this.battery.scale.setTo(0.8*this.sx,0.85*this.sy);
    this.battery.x = this.controlPanel.x + this.controlPanel.width - this.battery.width - 200*this.sx;
    this.battery.y = this.controlPanel.y + this.controlPanel.height/2 - this.battery.height/2;
    this.battery.inputEnabled=true;
    //Custom properties, CoordX and CoordY, added to object battery
    this.battery.CoordX = new Array();
    this.battery.CoordY = new Array();

    // Box
    this.box = this.game.add.sprite(0,0,'box');
    this.box.frame = 1;
    this.box.scale.setTo(0.2*this.sx,0.25*this.sy);
    this.box.x = this.controlPanel.x + this.controlPanel.width/2 - this.box.width/2;
    this.box.y = this.controlPanel.y + this.controlPanel.height/5 - this.box.height/2;
    
    // Set Phaser groups
    this.onPanel = this.game.add.group();
    this.onBattery = this.game.add.group();

    // this.numCrystals = this.randNumCrystals(); 
    // uncomment to randomize the total number of crystals
    
    for (var i=0;i<this.numCrystals;i++){
        var objectname = "object" + this.game.rnd.integerInRange(1,5);
        
        //var object = objects.create(controlPanel.x+220*sx+(arr[i]%4)*150*sx,controlPanel.y+150*sy+Math.floor(arr[i]/4)*180*sy,objectname);
        
        this.panel.CoordX[i] = this.controlPanel.x+220*this.sx+(i%COLUMNS_PANEL)*150*this.sx;
        this.panel.CoordY[i] = this.controlPanel.y+130*this.sy+Math.floor(i/COLUMNS_PANEL)*150*this.sy;
        //this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], objectname);
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], objectname);
    }
    
    this.batteryCoordinates();

    this.onPanel.forEach(function(object){
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.35*this.sx);
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,this.controlPanel);
        object.events.onDragStart.add(this.onDragStart.bind(this), this);
        object.events.onDragStop.add(this.onDragStop.bind(this), this);
        object.events.onDragUpdate.add(this.onDragUpdate.bind(this), this);
        //object.input.enableSnap(100*sx,100*sy,false,true);
    }, this);

    var fontSize = 100*this.sx;
    this.target = this.randNumTarget(this.numCrystals);
    this.scoreText = this.game.add.text(this.game.width/2,0.05*this.game.height,'Count: 0',{fontSize:'24px',fill:'#fff'});
    this.scoreText.x=this.game.width/2-this.scoreText.width/2;
    this.promptText = this.game.add.text(this.box.x+this.box.width/2,this.box.y+this.box.height/2,this.target,{fontSize:fontSize+'px',fill:'#fff'});
    this.promptText.anchor.setTo(0.5,0.5);

    // Publish Robot Command to give starting instructions
    // Only pluralize "crystal" if more than one crystal
    var crystalMsg = this.target > 1 ? "crystals" : "crystal";
    this.publishRobotGameIntro(this.target, crystalMsg);
};


/**
 * Computes the (x,y) coordinates of each position in the battery where a crystal can be placed
 */
Activity2.prototype.batteryCoordinates = function() {
    for (var i = 0; i < this.numCrystals; i++)
    {
        this.battery.CoordX[i] = this.controlPanel.x + this.battery.x +  20*this.sx + i%3* 120*this.sx;
        this.battery.CoordY[i] = this.controlPanel.y + this.battery.y + 100*this.sy + Math.floor(i/3)*145*this.sy;
    }
};


/**
 * Handler for when the button sprite is pressed
 *    Counts crystals in battery. If count same as target number, game is complete
 */
Activity2.prototype.onButtonPress = function(sprite,pointer) {
    sprite.frame=1;
    this.count = this.crystalCount();
    if (!this.finished(this.count))
        this.attempts++;
    else
        this.gameComplete();
};


function onButtonRelease(sprite,pointer){
    sprite.frame=0;
}


/**
 * Handler for when crystal is grabbed
 */
Activity2.prototype.onDragStart = function(sprite,pointer) {
    sprite.frame=1;
    if (this.isInside(sprite)) this.battery.frame = 1;

    if (this.dragging) 
        this.draggingSecond = true;
    this.dragging = true;
};


/**
 * Update crystal when dragged around
 */
Activity2.prototype.onDragUpdate = function(sprite,pointer) {
    if (this.isInside(sprite)) this.battery.frame = 1;
    else if (!this.isInside(sprite)) this.battery.frame = 0;
};


/**
 * Handler for when crystal is released
 */
Activity2.prototype.onDragStop = function(sprite,pointer) {
    sprite.frame=0;
    this.battery.frame = 0;
    if (this.isInside(sprite) && this.onPanel.removeChild(sprite))
    {
        this.onBattery.add(sprite);
    }
    else if (!this.isInside(sprite) && this.onBattery.removeChild(sprite))
    {
        this.onPanel.add(sprite);
    }
    console.log("onPanel.total: " + this.onPanel.total + " onBox.total: " + this.onBattery.total);

    if (this.draggingSecond)
        this.draggingSecond = false;
    else {
        this.dragging = false
        //Reorders the objects on the Panel
        for (var i = 0; i < this.onPanel.total; i++)
        {
            this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
            this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
        }
        
        //Reorders the objects on the Battery
        for (var i = 0; i < this.onBattery.total; i++)
        {
            this.onBattery.getChildAt(i).x = this.battery.CoordX[i];
            this.onBattery.getChildAt(i).y = this.battery.CoordY[i];
        }
    }
};


/**
 * Helper function to determine if an object is inside the battery sprite
 * @returns {Bool} True if object inside battery sprite, false otherwise
 */
Activity2.prototype.isInside = function(object) {
    if (object.x>this.battery.x+0.01*this.game.width && object.y>this.battery.y+0.01*this.game.height && object.x<this.battery.x+this.battery.width-0.01*this.game.width && object.y<this.battery.y+this.battery.height-0.01*this.game.height) return true;
    else return false;
}


/**
 * Count the number of crystals inside the battery
 * @returns {Int}
 */
Activity2.prototype.crystalCount = function() {
    var count = 0;
    this.onBattery.forEach(function(object){
        if (this.isInside(object)){
            console.log("Dropped inside the battery!");
            count++;
        }
    }, this);
    this.scoreText.setText('Count: '+count);
    return count;
}


/**
 * Determines if the target number of crystals dropped into the battery
 *    Publishes a robot command to provide feedback or congratulate player.
 * @return {Bool} True if game is complete, false otherwise
 */
Activity2.prototype.finished = function(crystals) {
    if (crystals==this.target){ 
        //alert('Yayy!');
        this.onBattery.forEach(function(object){ object.input.draggable = false;} );
        this.onPanel.forEach(function(object){ object.input.draggable = false;} );
        
        this.publishRobotSuccessMessage();
        return true;
    }
    else {
        // provide specific feedback on the mistake
        this.publishRobotGameFeedback(this.target, crystals > this.target ? 'fewer': 'more');
        return false;
    }
};


Activity2.prototype.randNumTarget = function(numCrystals) {
    switch(this.level) {
        case 1:
            return this.game.rnd.integerInRange(1, 3);
        case 2:
            return this.game.rnd.integerInRange(4, 6);
        case 3:
            return this.game.rnd.integerInRange(7, 9);
        default: 
            throw CONST.UNEXPECTED_LEVEL;
    }   
};

Activity2.prototype.randNumCrystals = function() {
    switch(this.level) {
        case 1:
            return this.game.rnd.integerInRange(8, 12);
        case 2:
            return this.game.rnd.integerInRange(13, 15);
        case 3:
            return this.game.rnd.integerInRange(16, 19);
        default:
            throw CONST.UNEXPECTED_LEVEL;
    }
}


module.exports = Activity2;

},{"../../SARGame.js":1,"../../constants.js":2}],11:[function(require,module,exports){
/**
  * Galactic Traveler Activity 3
  * -----------------------------
  *      Charge spaceship with [1-3, 4-6, 7-9] energy crystals. The level determines the number of crystals needed to
  *      charge the spaceship.
  *
  */

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const BOX_MIN_STARS = 1;
const ACTIVITY_ID = 3;

/**
 * Constructor for Activity 3
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity3(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.objects;
    this.box1, this.box2;
    this.count1, this.count2;
    this.promptText, this.question, this.lessmore;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.offset, this.smallBoxMax, this.largeBoxMax;

}
Activity3.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity3.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH +'Items/ControlPanelcrop.png');
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH +'Items/Stars/Star1_sheet.png',410,410,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH +'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH +'Items/Stars/Star3_sheet.png',410,410,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH +'Items/Stars/Star4_sheet.png',410,410,2);
    this.game.load.image('galaxy1', CONST.ASSETS_PATH +'Items/Galaxies/Galaxy1.png');
    this.game.load.image('galaxy2', CONST.ASSETS_PATH +'Items/Galaxies/Galaxy2.png');
    this.game.load.spritesheet('box', CONST.ASSETS_PATH +'Items/Box_sheet.png',881,730,2);

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity3.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    var controlPanel = this.game.add.sprite(0,0,'controlPanel');
    controlPanel.scale.setTo(0.95*this.sx,0.95*this.sy);
    controlPanel.x = this.game.world.centerX - controlPanel.width/2;
    controlPanel.y = this.game.world.centerY - controlPanel.height/2;

    this.box1 = this.game.add.sprite(0,0, 'box');
    this.box1.scale.setTo(0.7*this.sx,0.8*this.sy);
    this.box1.x = controlPanel.x + 200*this.sx ;
    this.box1.y = controlPanel.y + controlPanel.height/2 - this.box1.height/2;
    this.box1.inputEnabled=true;
    this.box1.frame = 0;
    this.box1.events.onInputDown.add(this.onInputDown.bind(this), this);
    this.box1.events.onInputUp.add(onInputUp,this);

    this.box2 = this.game.add.sprite(0,0,'box');
    this.box2.scale.setTo(0.7*this.sx,0.8*this.sy);
    this.box2.x = controlPanel.x + controlPanel.width - 200*this.sx - this.box2.width;
    this.box2.y = controlPanel.y + controlPanel.height/2 - this.box2.height/2;
    this.box2.inputEnabled=true;
    this.box2.frame = 0;
    this.box2.events.onInputDown.add(this.onInputDown.bind(this), this);
    this.box2.events.onInputUp.add(onInputUp,this);

    var galaxy1 = this.game.add.sprite(this.box1.x+this.box1.width/2,this.box1.y+this.box1.height/2,'galaxy1');
    galaxy1.anchor.setTo(0.5,0.5);
    galaxy1.scale.setTo(0.7*this.sx);

    var galaxy2 = this.game.add.sprite(this.box2.x+this.box2.width/2,this.box2.y+this.box2.height/2,'galaxy2');
    galaxy2.anchor.setTo(0.5,0.5);
    galaxy2.scale.setTo(0.7*this.sx);

    //console.log('cp width and height are'+controlPanel.width+'x'+controlPanel.height);

    // get a 1 or 2 to select which is bigger, left or right box
    this.lessmore = this.game.rnd.integerInRange(1, 2); //1 is for less, 2 is for more

    this.offset = this.generateBoxNumberOffset();
    console.log("off: " + this.offset);
    this.largeBoxMax = this.generateLargeBoxMax();
    this.smallBoxMax = this.generateSmallBoxMax(this.largeBoxMax, this.offset);

    this.count1 = this.game.rnd.integerInRange(this.smallBoxMax - this.offset, this.smallBoxMax);
    this.count2 = this.count1 + this.offset;
    // do {
    //     this.count1 = this.game.rnd.integerInRange(this.smallBoxMax - this.offset, this.smallBoxMax);
    //     this.count2 = this.count1 - this.offset;
    // } while (this.count1 == this.count2);

    var arr = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16];
    arr = this.shuffle(arr);
    this.objects=this.game.add.group();
    for (var i = 1; i <= this.count1; i++){
        var objectname = 'object'+this.game.rnd.integerInRange(1,4);
        this.objects.create(this.box1.x+110*this.sx+((arr[i]-1)%4)*120*this.sx,this.box1.y+100*this.sy+Math.floor((arr[i]-1)/4)*120*this.sy, objectname);
    }
    arr = this.shuffle(arr);
    for (i = 1; i <= this.count2; i++){
        var objectname = 'object'+this.game.rnd.integerInRange(1,4);
        this.objects.create(this.box2.x+110*this.sx+((arr[i]-1)%4)*120*this.sx,this.box2.y+100*this.sy+Math.floor((arr[i]-1)/4)*120*this.sy, objectname);
    }
    this.objects.forEach(function(object){
        object.scale.setTo(0.25*this.sx);
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
    }, this);

    var selection;
    if (this.lessmore == 1) {selection = 'fewer';}
    else if (this.lessmore == 2) {selection = 'more';}
    this.promptText = this.game.add.text(this.game.width/2,0.03*this.game.height,'',{fontSize:'24px',fill:'#fff'});
    this.question = 'Please choose the box with ' + selection + ' stars in it';
    this.promptText.setText(this.question);
    this.promptText.x = this.game.width/2-this.promptText.width/2;

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.lessmore == 1 ? 2 : 1, selection);
};


/**
 * Handler for when button is pressed
 */
Activity3.prototype.onInputDown = function(sprite) {
    sprite.frame = 1;
    if (this.activityCompleted(sprite)) {
        this.stat = true;
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
    else{
        this.stat = false;
        this.attempts++;

        // provide specific feedback on the mistake
        this.publishRobotGameFeedback(this.lessmore == 1 ? 2 : 1, this.lessmore == 1 ? "fewer" : "more");
    }
};

Activity3.prototype.activityCompleted = function (sprite) {
    return (this.count1>this.count2 && this.lessmore == 2 && sprite == this.box1) ||
           (this.count1>this.count2 && this.lessmore == 1 && sprite == this.box2) ||
           (this.count1<this.count2 && this.lessmore == 2 && sprite == this.box2) ||
           (this.count1<this.count2 && this.lessmore == 1 && sprite == this.box1);
}

Activity3.prototype.generateBoxNumberOffset = function() {
    switch (this.level) {
        case 1:
            return this.game.rnd.integerInRange(4, 6);
        case 2:
            return this.game.rnd.integerInRange(2, 4);
        case 3:
            return this.game.rnd.integerInRange(1, 3);
        default:
            throw CONST.UNEXPECTED_LEVEL;
    }
}

Activity3.prototype.generateLargeBoxMax = function() {
    switch (this.level) {
        case 1:
            return this.game.rnd.integerInRange(12, 16);
        case 2:
            return this.game.rnd.integerInRange(10, 15);
        case 3:
            return this.game.rnd.integerInRange(8, 12);
        default:
            throw CONST.UNEXPECTED_LEVEL;
    }
}

Activity3.prototype.generateSmallBoxMax = function(largeBoxVal, offset) {
    return largeBoxVal - offset;
}
    

function onInputUp(sprite){

};


module.exports = Activity3;

},{"../../SARGame.js":1,"../../constants.js":2}],12:[function(require,module,exports){
/**
  * Galactic Traveler Activity 4
  * -----------------------------
  *      Select a planet with numbers [1 - 10, 1 - 25, 1 - 100]. 
  *
  */

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 4;

/**
 * Constructor for Activity 4
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity4(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.objects;
    this.box;
    this.target;
    this.promptText, this.question;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity4.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity4.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');

    this.game.load.image('logo', 'assets/Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Planets/Jupiter_sheet.png',260,262,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Planets/Moon_sheet.png',428,428,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Planets/Saturn_sheet.png',826,289,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Planets/Neptune_sheet.png',428,428,2);
    this.game.load.image('box', CONST.ASSETS_PATH+'Items/BoxLong.png');

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity4.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    var controlPanel = this.game.add.sprite(0,0,'controlPanel');
    controlPanel.scale.setTo(0.95*this.sx,0.95*this.sy);
    controlPanel.x = this.game.world.centerX - controlPanel.width/2;
    controlPanel.y = this.game.world.centerY - controlPanel.height/2;

    this.box = this.game.add.sprite(0,0, 'box');
    this.box.scale.setTo(0.95*this.sx,0.95*this.sy);
    this.box.x = controlPanel.x + controlPanel.width/2 - this.box.width/2;
    this.box.y = controlPanel.y + controlPanel.height/2 - this.box.height/2;
    this.box.inputEnabled=true;

    this.objects = this.game.add.group();
    var object1 = this.objects.create(this.box.x+20*this.sx+this.box.width/9*1,this.box.y+this.box.height/2,'object1');
    var object2 = this.objects.create(this.box.x+20*this.sx+this.box.width/9*2.8,this.box.y+this.box.height/2,'object2');
    var object3 = this.objects.create(this.box.x+20*this.sx+this.box.width/9*5.3,this.box.y+this.box.height/2,'object3');
    var object4 = this.objects.create(this.box.x+20*this.sx+this.box.width/9*7.7,this.box.y+this.box.height/2,'object4');
    object1.scale.setTo(0.96*this.sx);
    object2.scale.setTo(0.576*this.sx);
    object3.scale.setTo(0.78*this.sx);
    object4.scale.setTo(0.576*this.sx);

    this.objects.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.inputEnabled=true;
        object.events.onInputDown.add(this.onInputDown.bind(this), this);
        object.events.onInputUp.add(this.onInputUp.bind(this), this);
    });

    var maxValue = this.generateMaxValue();

    this.target = this.generateTarget();
    this.answer = this.game.rnd.integerInRange(1, 4);
    var fontSize = 120*this.sx;

    var alreadyPicked = [this.target];
    var num;
    for (var i=1;i<=4;i++){
        if (i==this.answer) {
            num = this.game.add.text(this.objects.children[i-1].x,
                                     this.objects.children[i-1].y,
                                     this.target,
                                     {fontSize:fontSize+'px',fill:'#fff'});
        } else {
            var rand;
            do {
                rand = this.game.rnd.integerInRange(1, maxValue);
            } while (alreadyPicked.indexOf(rand) != -1);
            alreadyPicked.push(rand);

            num = this.game.add.text(this.objects.children[i-1].x,
                                     this.objects.children[i-1].y,
                                     rand,
                                     {fontSize:fontSize+'px',fill:'#fff'});
        }
        num.anchor.setTo(0.5);
    }

    this.promptText = this.game.add.text(this.game.width/2,0.03*this.game.height,'Choose the number six',{fontSize:'24px',fill:'#fff'});
    this.question = 'Please choose the planet with number '+this.target;
    this.promptText.setText(this.question);
    this.promptText.x = this.game.width/2-this.promptText.width/2;

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.target);
};


/**
 * Handler for when button is pressed
 */
Activity4.prototype.onInputDown = function(sprite) {
    sprite.frame = 1;
    this.stat = sprite.z==this.answer-1;

    if (this.stat) {
        this.promptText.setText(this.question + ' - You win!'); 
        this.promptText.x = this.game.width/2-this.promptText.width/2;
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
    else {
        this.attempts++;
        // specific feedback
        this.publishRobotGameFeedback(this.target);
    }
};

Activity4.prototype.onInputUp = function(sprite){
    sprite.frame = 0;
};

Activity4.prototype.generateMaxValue = function(){
    return 10; // constant value

    // switch (this.level) {
    //     case 1: return 10;
    //     case 2: return 30;
    //     case 3: return 100;
    //     default: throw CONST.UNEXPECTED_LEVEL;
    // }
};

Activity4.prototype.generateTarget = function() {
    //this.game.rnd.integerInRange(1, maxValue)

    switch (this.level) {
        case 1: return this.game.rnd.integerInRange(1, 3);
        case 2: return this.game.rnd.integerInRange(4, 6);
        case 3: return this.game.rnd.integerInRange(7, 9);
        default: throw CONST.UNEXPECTED_LEVEL;
    }
};

module.exports = Activity4;

},{"../../SARGame.js":1,"../../constants.js":2}],13:[function(require,module,exports){
/**
 * Galactic Traveler Activity 2
 * -----------------------------
 *      Charge spaceship with [1-3, 4-6, 7-9] energy crystals. The level determines the number of crystals needed to
 *      charge the spaceship.
 *
 */

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 5;

/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity5(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920,
        gameHeight = 1080;

    // Declare object properties
    this.objects;
    this.boxGroupL;
    this.boxGroupR;
    
    this.box1CoordX = new Array();
    this.box1CoordY = new Array();
    this.box2CoordX = new Array();
    this.box2CoordY = new Array();

    this.target;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.logo;
}

Activity5.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity5.prototype.preload = function() {
    // Load static image assets
    this.game.load.image('logo', '../../assets/Background/Atmosphere1.png'); // logo means background image

    // Load sprites
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Stars/Star1_sheet.png',410,410,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Stars/Star3_sheet.png',410,410,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Stars/Star4_sheet.png',410,410,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('yana', CONST.ASSETS_PATH+'Items/Aliens/Yana_sheet.png',730,881,2);
    this.game.load.spritesheet('yuki', CONST.ASSETS_PATH+'Items/Aliens/Yuki_sheet.png',730,881,2);

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets
 */
Activity5.prototype.create = function() {
    // background image is displayed
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    //  Moves the image anchor to the middle, so it centers inside the game properly
    this.logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+ this.game.width);
    console.log('Game height is'+ this.game.height);

    this.target = this.game.math.snapToCeil(2 * this.generateTarget(), 2,0);
    
    this.button = this.game.add.sprite(0,0,'button');
    this.button.scale.setTo(0.4*this.sx);
    this.button.x = this.game.world.centerX-this.button.width/2;
    this.button.y = this.game.world.height*0.9-this.button.height/2;
    this.button.frame=0;
    this.button.inputEnabled=true;
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);

    this.box1 = this.game.add.sprite(0,0,'yuki');
    this.box1.scale.setTo(0.7*this.sx);
    this.box1.x = 0.05*this.game.width;
    this.box1.y = this.game.height/2-this.box1.height/2;
    this.box1.inputEnabled=true;
    this.box1.frame = 0;

    this.box2 = this.game.add.sprite(0,0,'yana');
    this.box2.scale.setTo(0.7*this.sx);
    this.box2.x = 0.95*this.game.width-this.box2.width;
    this.box2.y = this.game.height/2-this.box2.height/2;
    this.box2.inputEnabled=true;
    this.box2.frame = 0;
    
    this.objects = this.game.add.group();
    this.boxGroupL = this.game.add.group();
    this.boxGroupR = this.game.add.group();
    
    this.boxCoordinates(this.box1CoordX,this.box1CoordY, this.box1);
    this.boxCoordinates(this.box2CoordX,this.box2CoordY, this.box2);
    
    for (var i=1;i<=this.target;i++){
        var name = "object"+(i%4+1);
        console.log("Object number "+i+" object name "+name);
        this.objects.create(this.game.world.centerX + Math.pow(-1,i%2)*75,0.1*this.game.height+Math.floor((i-1)/2)*100,name);
    }

    var bounds = new Phaser.Rectangle(0,0,this.logo.width,this.logo.height);

    this.objects.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        //clone the original star position
        object.originalPosition = object.position.clone();
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,this.bounds,null);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        //object.input.enableSnap(100,100,false,true);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.target);
};

Activity5.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
    console.log("outside");
}

Activity5.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.stat = this.countStars();
    if (!this.stat) {
        this.attempts++;

        // publish specific fail message
        this.publishRobotGameFeedback();
    } else {
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
}

Activity5.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
}

Activity5.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;
}

Activity5.prototype.onDragStop = function(sprite,pointer){
    sprite.frame=0;
                
    if (this.isInside(sprite, this.box1))
    {
        this.boxGroupL.addChild(sprite);
    }
    else if (this.isInside(sprite, this.box2)  )
    {
        this.boxGroupR.addChild(sprite);
    }
    
    else if (!(this.isInside(sprite, this.box1) || this.isInside(sprite, this.box2)) )
    {
        //add to the object group if the star is dragged outside of box and the total objects is not equal to total number of stars
        if(this.objects.countLiving()!== this.target){
            this.objects.addChild(sprite);
        }
        //set it back to its original position
        sprite.position.copyFrom(sprite.originalPosition); 
    }
        
    //Rearrange all the position in box1 and box2
    for (var i = 0; i < this.boxGroupL.total; i++)
    {
        this.boxGroupL.getChildAt(i).x = this.box1CoordX[i];
        this.boxGroupL.getChildAt(i).y = this.box1CoordY[i];
    }
    for (var i = 0; i < this.boxGroupR.total; i++)
    {
        this.boxGroupR.getChildAt(i).x = this.box2CoordX[i];
        this.boxGroupR.getChildAt(i).y = this.box2CoordY[i];
    }
}

Activity5.prototype.isInside = function(object,box){
    if (object.x > box.x + 0.01*this.game.width && 
        object.y > box.y + 0.01*this.game.height && 
        object.x < box.x + box.width - 0.01*this.game.width && 
        object.y < box.y + box.height - 0.01*this.game.height) return true;
    else return false;
}

Activity5.prototype.countStars = function(){
    var count1 = this.boxGroupL.total;
    var count2 = this.boxGroupR.total;

    if (count1==count2 && count1==this.target/2.0){ 
        this.boxGroupL.forEach(function(object){object.input.draggable = false;});
        this.boxGroupR.forEach(function(object){object.input.draggable = false;});
        return true;
    }
        else return false;
}


Activity5.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< this.target; i++){
        
        boxArrayX[i] = box.x+100*this.sx+(i%3)*150*this.sx;
        boxArrayY[i] = box.y+ 70 *this.sy + Math.floor(i/3)*100*this.sy;
    }
}

Activity5.prototype.generateTarget = function() {
    switch (this.level) {
        case 1: return this.game.rnd.integerInRange(3, 4);
        case 2: return this.game.rnd.integerInRange(5, 6);
        case 3: return this.game.rnd.integerInRange(7, 8);
        default: throw CONST.UNEXPECTED_LEVEL;
    }
}

module.exports = Activity5;

},{"../../SARGame.js":1,"../../constants.js":2}],14:[function(require,module,exports){
'use strict';

var Activity1 = require('./activity1.js');
var Activity2 = require('./activity2.js');
var Activity3 = require('./activity3.js');
var Activity4 = require('./activity4.js');
var Activity5 = require('./activity5.js');
var Activity6 = require('./activity1.js');
var Activity7 = require('./activity2.js');
var Activity8 = require('./activity3.js');
var Activity9 = require('./activity4.js');
var Activity10 = require('./activity5.js');

var GameWrapper = require('../../SARGame.js').GameWrapper;
var CONST = require('../../constants.js');


function GalacticTraveler(ros, level, exitCallBack) {
    GameWrapper.call(this, ros, level, exitCallBack);
    this.gameid = CONST.Games.GALACTIC_TRAVELER;
    // this.activities = [Activity1, Activity2];
    this.activities = [Activity1, Activity2, Activity3, Activity4, Activity5, Activity6, Activity7, Activity8, Activity9, Activity10];
    this.start();
}
GalacticTraveler.prototype.__proto__ = GameWrapper.prototype;


module.exports = GalacticTraveler;

},{"../../SARGame.js":1,"../../constants.js":2,"./activity1.js":9,"./activity2.js":10,"./activity3.js":11,"./activity4.js":12,"./activity5.js":13}],15:[function(require,module,exports){
/**
  * Space Ship Tidy Up Activity 1
  * -----------------------------
  *      Board Space Pets - Arrange space pets based on their value 
  *      The level determines the numbers on the space pets
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 1;
/**
 * Constructor for Activity 1
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity1(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.objects, this.spaceship;
    this.currentCount = 0;
    this.target;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity1.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity1.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/Atmosphere1.png');

    this.game.load.spritesheet('object1',CONST.ASSETS_PATH+'Items/Aliens/Alien1Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object2',CONST.ASSETS_PATH+'Items/Aliens/Alien2Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object3',CONST.ASSETS_PATH+'Items/Aliens/Alien3Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object4',CONST.ASSETS_PATH+'Items/Aliens/Alien4Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object5',CONST.ASSETS_PATH+'Items/Aliens/Alien5Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object6',CONST.ASSETS_PATH+'Items/Aliens/Alien6Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object7',CONST.ASSETS_PATH+'Items/Aliens/Alien7Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object8',CONST.ASSETS_PATH+'Items/Aliens/Alien8Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object9',CONST.ASSETS_PATH+'Items/Aliens/Alien9Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object10',CONST.ASSETS_PATH+'Items/Aliens/Alien10Box_sheet.png',610,610,2);
    this.game.load.spritesheet('spaceship',CONST.ASSETS_PATH+'Items/Spaceship_sheet.png',1090,586,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity1.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    this.target = this.generateTarget();

    this.spaceship = this.game.add.sprite(0.55*this.game.width,0.5*this.game.height,'spaceship');
    this.spaceship.scale.setTo(0.7*this.sx);
    this.spaceship.y = 0.5*this.game.height - this.spaceship.height/2;
    this.spaceship.inputEnabled = true;
    this.spaceship.frame = 0;

    var arr = [1,2,3,4,5,6,7,8,9,10];
    arr = this.shuffle(arr);
    this.objects=this.game.add.group();

    for (var i=1;i<=this.target;i++){
        var name = "object"+arr[i-1];
        console.log("Object number "+i+" object name "+name);
        var object = this.objects.create(0.1*this.game.width+((i-1)%4)*230*this.sx,
                                         150*this.sy+Math.floor((i-1)/4)*230*this.sy,name);
        object.addChildAt(this.game.make.text(-object.width/2,
                                         -object.height/2,i,
                                         {fontSize:250*this.sx+'px',fill:'#fff'}),0);
    }

    var bounds = new Phaser.Rectangle(0,0,logo.width,logo.height);

    this.objects.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.scale.setTo(0.30*this.sx);
        object.originalPosition = object.position.clone();
        object.frame=0;
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,bounds,null);
        object.events.onDragStart.add(this.onDragStart.bind(this), this);
        object.events.onDragStop.add(this.onDragStop.bind(this), this);
        object.events.onDragUpdate.add(this.onDragUpdate.bind(this), this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.target);
};


Activity1.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
    console.log("outside");
}

Activity1.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;
}

Activity1.prototype.onDragStop = function(sprite,pointer){
    sprite.frame=0;
    this.spaceship.frame = 0;
    if (this.isInside(sprite) && this.isCorrect(sprite)) sprite.destroy();
     
    else if (this.isInside(sprite)) {
        var num = sprite.getChildAt(0).text;
        sprite.x = 0.1*this.game.width+((num-1)%4)*230*this.sx;
        sprite.y = 150*this.sy+Math.floor((num-1)/4)*230*this.sy;
        this.attempts++;

        this.publishRobotGameFeedback(this.target);
    }
    
    else if (!this.isInside(sprite)){
        this.objects.forEach(function(object){
            object.position.copyFrom(object.originalPosition);
        });
    }
        
 
    if (this.currentCount==this.target){
        this.objects.forEach(function(object){object.input.draggable = false;});
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
}

Activity1.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite)) this.spaceship.frame = 1;
    else if (!this.isInside(sprite)) this.spaceship.frame = 0;
}

Activity1.prototype.isInside = function(object){
    if (object.x>this.spaceship.x+0.01*this.game.width && 
        object.y>this.spaceship.y+0.01*this.game.height && 
        object.x<this.spaceship.x+this.spaceship.width-0.01*this.game.width && 
        object.y<this.spaceship.y+this.spaceship.height-0.01*this.game.height) return true;
    else return false;
}

Activity1.prototype.isCorrect = function(object){
    if (object.getChildAt(0).text==this.currentCount+1){this.currentCount++; return true;}
    else return false;
};

Activity1.prototype.generateTarget = function() {
    switch(this.level) {
        case 1:
            return this.game.rnd.integerInRange(2, 4);
        case 2:
            return this.game.rnd.integerInRange(5, 7);
        case 3:
            return this.game.rnd.integerInRange(8, 10);
        default: 
            throw CONST.UNEXPECTED_LEVEL;
    }
};


module.exports = Activity1;

},{"../../SARGame.js":1,"../../constants.js":2}],16:[function(require,module,exports){
/**
  * Space Ship Tidy Up Activity 2
  * -----------------------------
  *      Organize rocks by color
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 2;

const ROCK_AMOUNT = 9;
const COLUMNS_PANEL = 3;
const SAME_COLOR_AMOUNT = 3;
/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity2(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.logoBackground;
    this.onPanel;
    this.onbox1;
    this.onbox2;
    this.onbox3;
    this.box1;
    this.box2;
    this.box3;

    this.objectname = new Array();
            
    // create a color class-object array 
    // Each color has its properties, rockString and boxColor(String) that have the same color
    //shuffle the array later
    this.color = [
        { rockString: "object3", boxColor: "boxOrange"},
        { rockString: "object4", boxColor: "boxPink"},
        { rockString: "object5", boxColor: "boxBlue"}
    ]
    
    this.shuffle(this.color);
    
    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }
    
    // keep track of the color count using array 
    // index 0 ==> 1st color;
    // index 1 ==> 2nd color;
    // index 2 ==> 3nd color;
                        
    this.countBox1Color = [0,0,0];
    
    this.countBox2Color = [0,0,0];
    
    this.countBox3Color = [0,0,0];
    
    
    this.numColor1 =0;
    this.numColor2 =0;
    this.numColor3 =0;

    this.controlPanel, this.button;

    this.sx = window.innerWidth / gameWidth;
    this.sy = window.innerHeight / gameHeight;
}
Activity2.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity2.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');

    this.game.load.spritesheet('object3',CONST.ASSETS_PATH+'Items/Rocks/Moonrock3_sheet.png',660,360,2);
    this.game.load.spritesheet('object4',CONST.ASSETS_PATH+'Items/Rocks/Moonrock4_sheet.png',510,410,2);
    this.game.load.spritesheet('object5',CONST.ASSETS_PATH+'Items/Rocks/Moonrock5_sheet.png',660,360,2);
    this.game.load.spritesheet('button',CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box',CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    this.game.load.spritesheet('boxBlue',CONST.ASSETS_PATH+'Items/colorBox/BoxBlue_sheetnew.png',1510,499,2);
    this.game.load.spritesheet('boxPink',CONST.ASSETS_PATH+'Items/colorBox/BoxPink_sheetnew.png',1510,499,2);
    this.game.load.spritesheet('boxOrange',CONST.ASSETS_PATH+'Items/colorBox/BoxOrange_sheetnew.png',1510,499,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity2.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;
    
    this.button = this.game.add.sprite(0,0,'button');
    this.button.scale.setTo(0.3*this.sx);
    this.button.x = this.controlPanel.x+this.controlPanel.width/2-this.button.width/2;
    this.button.y = this.controlPanel.y+this.controlPanel.height-this.button.height-0.05*this.game.height;
    this.button.frame=0;
    this.button.inputEnabled=true;
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);
    
    
    this.box1 = this.game.add.sprite(0,0, this.color[0].boxColor);
    // long box is used instead of small box 
    this.box1.scale.setTo(0.45*this.sx,0.2*this.sy);
    //box1.anchor.setTo(0.5,0.5);
    this.box1.x = this.controlPanel.x + this.controlPanel.width  - this.box1.width - 140*this.sx;
    this.box1.y = this.controlPanel.y + this.controlPanel.height/3 - this.box1.height/3 - this.button.height - 50 *this.sx;
    this.box1.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box1
    this.box1.CoordX = new Array();
    this.box1.CoordY = new Array();
    
    this.box2 = this.game.add.sprite(0,0, this.color[1].boxColor);
    //box2.anchor.setTo(0.5,0.5);
    this.box2.scale.setTo(0.45*this.sx,0.2*this.sy);
    this.box2.x = this.controlPanel.x + this.controlPanel.width  - this.box2.width - 140*this.sx;
    this.box2.y = this.controlPanel.y + this.box1.y + 50*this.sx;
    this.box2.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box2
    this.box2.CoordX = new Array();
    this.box2.CoordY = new Array();
    
    this.box3 = this.game.add.sprite(0,0, this.color[2].boxColor);
    //box3.anchor.setTo(0.5,0.5);
    this.box3.scale.setTo(0.45*this.sx,0.2*this.sy);
    this.box3.x = this.controlPanel.x + this.controlPanel.width  - this.box3.width - 140*this.sx;
    this.box3.y = this.controlPanel.y + this.box2.y+ 50*this.sx;
    this.box3.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box3
    this.box3.CoordX = new Array();
    this.box3.CoordY = new Array();
    
    //set up the rock image at the upper left corner
    var upperLHRock1= this.game.add.sprite(this.box1.x, this.box1.y, this.color[0].rockString);
    upperLHRock1.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHRock2 = this.game.add.sprite(this.box2.x, this.box2.y, this.color[1].rockString);
    upperLHRock2.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHRock3 = this.game.add.sprite(this.box3.x, this.box3.y, this.color[2].rockString);
    upperLHRock3.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    
    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    
    
    //set a string array called objectname and shuffle them
    //it is used to generate/ create rock on panel
    for (var i=0; i< SAME_COLOR_AMOUNT ; i++){
        this.objectname[i] = "object3";
        this.objectname[i+3] = "object4";
        this.objectname[i+6] = "object5";
        
    }
    
    this.shuffle(this.objectname);
    
    //create the rock on panel
    for (var i=0;i<ROCK_AMOUNT;i++){
        //objectname = "object" + game.rnd.integerInRange(3,5);
        this.panel.CoordX[i] = this.controlPanel.x + 220*this.sx +(i%COLUMNS_PANEL) * 180 *this.sx;
        this.panel.CoordY[i] = this.controlPanel.y + this.controlPanel.height/2 - 120*this.sy + Math.floor(i/COLUMNS_PANEL) * 100*this.sy;
        
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }
    
    this.boxCoordinates(this.box1.CoordX, this.box1.CoordY,this.box1);
    this.boxCoordinates(this.box2.CoordX, this.box2.CoordY,this.box2);
    this.boxCoordinates(this.box3.CoordX, this.box3.CoordY,this.box3);
    
    for (var i=0; i<this.onPanel.total ; i++){
    
        switch (this.onPanel.getChildAt(i).key){
            
            case this.color[0].rockString: // let's say 'object3':
                this.numColor1++;
                break;
            case this.color[1].rockString:
                this.numColor2++;
                break;
            case this.color[2].rockString:
                this.numColor3++;
                break;
            
        }
    }

    
    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        //object.originalPosition = object.position.clone();
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,this.controlPanel);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.ROCK_AMOUNT);
};

Activity2.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
}

Activity2.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkColor();
}

Activity2.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
    this.resetColorCount();
}

Activity2.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
}

Activity2.prototype.onDragStop = function(sprite, pointer){
    this.box1.frame = 0;
    this.box2.frame = 0;
    this.box3.frame = 0;
    sprite.frame = 0;
    
    
    if(this.onBox1.total < 4 && this.isInside(sprite,this.box1) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox3.removeChild(sprite))){
        this.onBox1.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box1) && this.onBox1.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    if(this.onBox2.total < 4 && this.isInside(sprite,this.box2)&& (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox3.removeChild(sprite))){    
        this.onBox2.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box2) && this.onBox2.removeChild(sprite))
    {
        this.onPanel.add(sprite);
    }
    
    if(this.onBox3.total < 4 && this.isInside(sprite,this.box3) && (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox2.removeChild(sprite))){
        this.onBox3.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box3) && this.onBox3.removeChild(sprite))
    {
        this.onPanel.add(sprite);
    }
    
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
        this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox1.total; i++)
    {
        this.onBox1.getChildAt(i).x = this.box1.CoordX[i];
        this.onBox1.getChildAt(i).y = this.box1.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox2.total; i++)
    {
        this.onBox2.getChildAt(i).x = this.box2.CoordX[i];
        this.onBox2.getChildAt(i).y = this.box2.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox3.total; i++)
    {
        this.onBox3.getChildAt(i).x = this.box3.CoordX[i];
        this.onBox3.getChildAt(i).y = this.box3.CoordY[i];
    }

    
}

Activity2.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite,this.box1)){
        this.box1.frame = 1;
    }
    else if (!this.isInside(sprite,this.box1)){
        this.box1.frame = 0;
    }
    
    if (this.isInside(sprite,this.box2)){
        this.box2.frame = 1;
    }
    
    else if (!this.isInside(sprite,this.box2)){
        this.box2.frame = 0;
    }
    if (this.isInside(sprite,this.box3)){
        this.box3.frame = 1;
    }
    else if (!this.isInside(sprite,this.box3)){
        this.box3.frame = 0;
    }
                
}

Activity2.prototype.isInside = function(object,box){
    if (object.x > box.x+ 0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity2.prototype.checkColor = function(){
    
    //count the Number of the Y/P/LB color rocks in each box
    for(var i = 0 ; i <this.onBox1.total ; i++){
        
        if(this.onBox1.getChildAt(i).key == this.color[0].rockString){
            this.countBox1Color[0]++;
        }
        else if (this.onBox1.getChildAt(i).key == this.color[1].rockString){
            this.countBox1Color[1]++;
        
        }
        else if (this.onBox1.getChildAt(i).key == this.color[2].rockString){
            this.countBox1Color[2]++;
        }
    }
    
    
    for(var i=0; i<this.onBox2.total; i++){
        
        if(this.onBox2.getChildAt(i).key == this.color[0].rockString){
            this.countBox2Color[0]++;
        }
        else if (this.onBox2.getChildAt(i).key == this.color[1].rockString){
            this.countBox2Color[1]++;
        
        }
        else if (this.onBox2.getChildAt(i).key == this.color[2].rockString){
            this.countBox2Color[2]++;
        }
    }
    
    for(var i=0; i<this.onBox3.total ; i++){
        
        if(this.onBox3.getChildAt(i).key == this.color[0].rockString ){
            this.countBox3Color[0]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.color[1].rockString){
            this.countBox3Color[1]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.color[2].rockString){
            this.countBox3Color[2]++;
        }
    }
    
    
    if  (   
            (this.numColor1 == this.countBox1Color[0] ) && 
            (this.numColor2 == this.countBox2Color[1] ) &&
            (this.numColor3 == this.countBox3Color[2] ) 
        )
        {
            this.publishRobotSuccessMessage();
            this.gameComplete();
            return true;
        }
    else{
        this.attempts++;
        this.publishRobotGameFeedback();
    }
            
}

Activity2.prototype.resetColorCount = function(){
    for(var i =0; i< this.countBox1Color.length; i++)
    {
        this.countBox1Color[i] =0;
    }
    for(var i =0; i< this.countBox2Color.length; i++)
    {
        this.countBox2Color[i] =0;
    }
    for(var i =0; i< this.countBox3Color.length; i++)
    {
        this.countBox3Color[i] =0;
    }
}

Activity2.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< ROCK_AMOUNT; i++){
        boxArrayX[i] = box.x+ 130 *this.sx+(i%4)* 150 *this.sx;
        boxArrayY[i] = box.y+ box.height/2 + Math.floor(i/4)* 50 *this.sy;
    }
}


module.exports = Activity2;

},{"../../SARGame.js":1,"../../constants.js":2}],17:[function(require,module,exports){
/**
  * Space Ship Tidy Up Activity 3
  * -----------------------------
  * game6-1.html
  * Organize by item
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 3;

const OBJECT_AMOUNT = 9;
const COLUMNS_PANEL = 3;
const SAME_OBJECT_AMOUNT = 3;
/**
 * Constructor for Activity 3
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity3(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onBox1;
    this.onBox2;
    this.onBox3;
    
    this.box1;
    this.box2;
    this.box3;
    
    
    this.target;
    
    this.objectname = new Array();
    //string array with different type of item (in term of filename : 'object1')
    this.itemType = ['object1','object2', 'object3'];
    this.shuffle(this.itemType);
    
    
    this.box1CoordX = new Array();
    this.box1CoordY = new Array();
    this.box2CoordX = new Array();
    this.box2CoordY = new Array();
    this.box3CoordX = new Array();
    this.box3CoordY = new Array();
    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }
    
    // keep track of the item count using array for each box 
    // index 0 ==> 1st item;
    // index 1 ==> 2nd item;
    this.countBox1ItemType = [0,0,0];
    
    this.countBox2ItemType = [0,0,0];
    
    this.countBox3ItemType= [0,0,0];
    
    
    this.num_type1 =0;
    this.num_type2 =0;
    this.num_type3 =0;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity3.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity3.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');
                        
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Rocks/Moonrock2_sheet.png',460,360,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Crystals/Crystal3_sheet.png',410,460,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity3.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    var controlPanel = this.game.add.sprite(0,0,'controlPanel');
    controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    controlPanel.x = this.game.world.centerX - controlPanel.width/2;
    controlPanel.y = this.game.world.centerY - controlPanel.height/2;
    
    var button = this.game.add.sprite(0,0,'button');
    button.scale.setTo(0.3*this.sx);
    button.x = controlPanel.x+controlPanel.width/2-button.width/2;
    button.y = controlPanel.y+controlPanel.height-button.height-0.05*this.game.height;
    button.frame=0;
    button.inputEnabled=true;
    button.events.onInputDown.add(this.onButtonPress,this);
    button.events.onInputUp.add(this.onButtonRelease,this);
    
    this.box1 = this.game.add.sprite(0,0, 'box');
    this.box1.scale.setTo(0.8*this.sx,0.15*this.sy);
    this.box1.x = controlPanel.x + controlPanel.width  - this.box1.width - 140*this.sx;
    this.box1.y = controlPanel.y + controlPanel.height/3 - this.box1.height/3 - button.height - 50 *this.sx;
    this.box1.frame = 0;
    //Custom properties, CoordX and CoordY, added to object box1
    this.box1.CoordX = new Array();
    this.box1.CoordY = new Array();
    
    console.log('Box1 height is '+this.box1.height);
    console.log('Box1 width is '+this.box1.width);
    
    this.box2 = this.game.add.sprite(0,0, 'box');
    this.box2.scale.setTo(0.8*this.sx,0.15*this.sy);
    this.box2.x = controlPanel.x + controlPanel.width  - this.box2.width - 140*this.sx;
    this.box2.y = controlPanel.y + this.box1.y + 50*this.sx;
    this.box2.frame = 0;
    //Custom properties, CoordX and CoordY, added to object box2
    this.box2.CoordX = new Array();
    this.box2.CoordY = new Array();
    console.log('Box2 height is '+this.box2.height);
    console.log('Box2 width is '+this.box2.width);
    
    this.box3 = this.game.add.sprite(0,0, 'box');
    this.box3.scale.setTo(0.8*this.sx,0.15*this.sy);
    this.box3.x = controlPanel.x + controlPanel.width  - this.box3.width - 140*this.sx;
    this.box3.y = controlPanel.y + this.box2.y+ 50*this.sx;
    this.box3.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box3
    this.box3.CoordX = new Array();
    this.box3.CoordY = new Array();
    console.log('Box2 height is '+this.box3.height);
    console.log('Box2 width is '+this.box3.width);
    

    var upperLHItem1 = this.game.add.sprite(this.box1.x, this.box1.y, this.itemType[0]);
    upperLHItem1.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHItem2 = this.game.add.sprite(this.box2.x, this.box2.y, this.itemType[1]);
    upperLHItem2.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHItem3 = this.game.add.sprite(this.box3.x, this.box3.y, this.itemType[2]);
    upperLHItem3.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    
    
    // set up all the item/object on the panel
    for (var i=0; i< SAME_OBJECT_AMOUNT ; i++){
        this.objectname[i] = "object1";
        this.objectname[i+3] = "object2";
        this.objectname[i+6] = "object3";
    }

    this.shuffle(this.objectname);
    
    //create all the object on the panel
    for (var i=0;i<OBJECT_AMOUNT;i++){
        //objectname = "object" + game.rnd.integerInRange(3,5);
        this.panel.CoordX[i] = controlPanel.x + 220*this.sx +(i%COLUMNS_PANEL) * 180 *this.sx;
        this.panel.CoordY[i] = controlPanel.y + controlPanel.height/2 - 120*this.sy + Math.floor(i/COLUMNS_PANEL) * 100*this.sy;
        
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }
    
    //set the position in each box
    this.boxCoordinates(this.box1.CoordX, this.box1.CoordY, this.box1);
    this.boxCoordinates(this.box2.CoordX, this.box2.CoordY, this.box2);
    this.boxCoordinates(this.box3.CoordX, this.box3.CoordY, this.box3);
    
    // assign the rock/crystal/star to an array type[0/1/2]
    for (var i=0; i<this.onPanel.total ; i++){
    
        switch (this.onPanel.getChildAt(i).key){
            
            case this.itemType[0]:// let say 'object1':
                this.num_type1++;
                break;
            case this.itemType[1]:
                this.num_type2++;
                break;
            case this.itemType[2]:
                this.num_type3++;
                break;
            
        }
    }

    
    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        object.originalPosition = object.position.clone();
        //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,controlPanel);
        //game.add.tween(object).to( { y: 400 }, 3000, Phaser.Easing.Cubic.InOut, true, 0, Number.MAX_VALUE, true);
        //object.input.enableSnap(130 *sx,150 *sy,false,true);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro();
};

Activity3.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
}


Activity3.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkItemType();
}

Activity3.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
    this.resetItemCount();
}

Activity3.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
    

}

Activity3.prototype.onDragStop = function(sprite, pointer){
    this.box1.frame = 0;
    this.box2.frame = 0;
    this.box3.frame = 0;
    sprite.frame = 0;
    
    
    if(this.onBox1.total < 4 && this.isInside(sprite,this.box1) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox3.removeChild(sprite))){
        this.onBox1.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box1) && this.onBox1.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    if(this.onBox2.total < 4 && this.isInside(sprite,this.box2 )&& (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox3.removeChild(sprite))){
        this.onBox2.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box2) && this.onBox2.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    if(this.onBox3.total < 4 && this.isInside(sprite,this.box3) && (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox2.removeChild(sprite))){
        this.onBox3.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box3) && this.onBox3.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
        this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox1.total; i++)
    {
        this.onBox1.getChildAt(i).x = this.box1.CoordX[i];
        this.onBox1.getChildAt(i).y = this.box1.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox2.total; i++)
    {
        this.onBox2.getChildAt(i).x = this.box2.CoordX[i];
        this.onBox2.getChildAt(i).y = this.box2.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox3.total; i++)
    {
        this.onBox3.getChildAt(i).x = this.box3.CoordX[i];
        this.onBox3.getChildAt(i).y = this.box3.CoordY[i];
    }

    
}

Activity3.prototype.onDragUpdate = function (sprite,pointer){
    if (this.isInside(sprite,this.box1)){
        this.box1.frame = 1;
    }
    else if (!this.isInside(sprite,this.box1)){
        this.box1.frame = 0;
    }
    
    if (this.isInside(sprite,this.box2)){
        this.box2.frame = 1;
    }
    
    else if (!this.isInside(sprite,this.box2)){
        this.box2.frame = 0;
    }
    if (this.isInside(sprite,this.box3)){
        this.box3.frame = 1;
    }
    else if (!this.isInside(sprite,this.box3)){
        this.box3.frame = 0;
    }
                
}


Activity3.prototype.isInside = function(object,box){
    if (object.x > box.x+ 0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity3.prototype.checkItemType = function(){
    //count the Number of the rock/crystal/star  in each box
    for(var i = 0 ; i <this.onBox1.total ; i++){
        
        if(this.onBox1.getChildAt(i).key == this.itemType[0]){
            this.countBox1ItemType[0]++;
        }
        else if (this.onBox1.getChildAt(i).key == this.itemType[1]){
            this.countBox1ItemType[1]++;
        
        }
        else if (this.onBox1.getChildAt(i).key == this.itemType[2]){
            this.countBox1ItemType[2]++;
        }

    }
    
    for(var i=0; i<this.onBox2.total; i++){
        
        if(this.onBox2.getChildAt(i).key == this.itemType[0]){
            this.countBox2ItemType[0]++;
        }
        else if (this.onBox2.getChildAt(i).key == this.itemType[1]){
            this.countBox2ItemType[1]++;
        }
        else if (this.onBox2.getChildAt(i).key == this.itemType[2]){
            this.countBox2ItemType[2]++;
        }
    }
    
    for(var i=0; i<this.onBox3.total ; i++){
        
        if(this.onBox3.getChildAt(i).key == this.itemType[0] ){
            this.countBox3ItemType[0]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.itemType[1]){
            this.countBox3ItemType[1]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.itemType[2]){
            this.countBox3ItemType[2]++;
        }
    }
    
    if  (   (this.num_type1 == this.countBox1ItemType[0] ) && 
            (this.num_type2 == this.countBox2ItemType[1] ) &&
            (this.num_type3 == this.countBox3ItemType[2] ) 
        )
        {
            this.publishRobotSuccessMessage();
            this.gameComplete();
            return true;
        }
    else{
        attempts++;
        this.publishRobotGameFeedback();
    }
}

Activity3.prototype.resetItemCount = function(){
    for(var i =0; i< this.countBox1ItemType.length; i++)
    {
        this.countBox1ItemType[i] =0;
    }
    for(var i =0; i< this.countBox2ItemType.length; i++)
    {
        this.countBox2ItemType[i] =0;
    }
    for(var i =0; i< this.countBox3ItemType.length; i++)
    {
        this.countBox3ItemType[i] =0;
    }
}

Activity3.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< OBJECT_AMOUNT; i++){
        boxArrayX[i] = box.x + 130 *this.sx + (i%4)* 150 * this.sx;
        boxArrayY[i] = box.y + box.height/2 + Math.floor(i/4) * 50 * this.sy;
    }
}

module.exports = Activity3;

},{"../../SARGame.js":1,"../../constants.js":2}],18:[function(require,module,exports){
/**
  * Space Ship Tidy Up Activity 4
  * -----------------------------
  *      game4-1-2.html
  *      Copy the sequence
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 4;

const OBJECT_AMOUNT = 9;
const COLUMNS_PANEL = 3;
const SEQUENCE_AMOUNT =4;
/**
 * Constructor for Activity 1
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity4(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onBox1;
    this.onBox2;

    this.box1;
    this.box2;
    this.box3;
    this.box4;
    this.box5;
    this.box6;
    this.box7;
    this.box8;
    
    //set a string array called objectname and shuffle them
    // tempObj is randomize
    this.tempObj1 = "object" + this.game.rnd.integerInRange(1,2);
    this.tempObj2 = "object" + this.game.rnd.integerInRange(3,4);
    this.tempObj3 = "object" + this.game.rnd.integerInRange(5,6);
        
    this.objectname  =  ['object1', 'object2', 'object3', 'object4' , 'object5', 'object6', this.tempObj1, this.tempObj2, this.tempObj3];
    this.shuffle(this.objectname);
    
    //set up a sequence array
    this.sequenceObj = new Array();
    

    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity4.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity4.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');
                        
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Rocks/Moonrock1_sheet.png',610,400,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Rocks/Moonrock4_sheet.png',510,410,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Crystals/Crystal5_sheet.png',410,360,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Crystals/Crystal2_sheet.png',460,510,2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH+'Items/Stars/Star1_sheet.png',410,410,2);
    this.game.load.spritesheet('object6', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    this.game.load.spritesheet('bigBox', CONST.ASSETS_PATH+'Items/colorBox/BoxBlue_sheetnew.png',1510,510,2);
    this.game.load.spritesheet('test', CONST.ASSETS_PATH+'Items/BoxRotatedOver.png',720,871 ,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity4.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    var controlPanel = this.game.add.sprite(0,0,'controlPanel');
    controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    controlPanel.x = this.game.world.centerX - controlPanel.width/2;
    controlPanel.y = this.game.world.centerY - controlPanel.height/2;
    
    var button = this.game.add.sprite(0,0,'button');
    button.scale.setTo(0.3*this.sx);
    button.x = controlPanel.x+controlPanel.width/2-button.width/2;
    button.y = controlPanel.y+controlPanel.height-button.height-0.05*this.game.height;
    button.frame=0;
    button.inputEnabled=true;
    button.events.onInputDown.add(this.onButtonPress,this);
    button.events.onInputUp.add(this.onButtonRelease,this);
    
    /*
    box1/3/5/7 are used to set the sequence
    box2/4/6/8 are used to create the empty box on the blue rectangle box
    */
    
    this.box1 = this.game.add.sprite(0,0, 'box');
    this.box1.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box1.x = controlPanel.x + controlPanel.width  - box1.width - 140*sx;
    this.box1.x = controlPanel.x + controlPanel.width/2 -60*this.sx;
    //box1.y = controlPanel.y + controlPanel.height/2 - box1.height/2 - button.height - 50 *sx;
    this.box1.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box1.height/2 - 50 *this.sx;
    this.box1.frame = 0;
    this.box1.visible = false;
    console.log('Box1 height is '+ this.box1.height);
    console.log('Box1 width is '+ this.box1.width);           

    var rec0 = this.game.add.sprite(0,0,'bigBox');
    rec0.scale.setTo(0.488*this.sx,0.3*this.sy);
    rec0.x = this.box1.x  -40*this.sx ;
    rec0.y = this.box1.y -15*this.sx;
    rec0.frame = 0;
    
    //second blue rectangle box is created first so that it doesnt cover the empty box2/4/6/8
    var rec1 = this.game.add.sprite(0,0,'bigBox');
    rec1.scale.setTo(0.488*this.sx,0.3*this.sy);
    rec1.x = controlPanel.x + controlPanel.width/2 -60*this.sx  - 40*this.sx;
    rec1.y = controlPanel.y + this.box1.y + 50*this.sx - 20*this.sx;
    rec1.frame = 0;
    
    this.box2 = this.game.add.sprite(0,0, 'box');
    this.box2.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box2.x = controlPanel.x + controlPanel.width  - box2.width - 140*sx;
    this.box2.x = controlPanel.x + controlPanel.width/2 -60*this.sx;
    this.box2.y = controlPanel.y + this.box1.y + 50*this.sx;
    this.box2.frame = 0;
    
    this.box3 = this.game.add.sprite(0,0, 'box');
    this.box3.scale.setTo(0.18*this.sx,0.15*this.sy);
    this.box3.x = controlPanel.x +controlPanel.width/2-60*this.sx + this.box1.width + 10*this.sx;
    //box3.x = controlPanel.x + controlPanel.width  - 2*box3.width - 160*sx;
    //box3.y = controlPanel.y + controlPanel.height/2 - box3.height/2 - button.height - 50 *sx;
    this.box3.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box3.height/2 - 50 *this.sx;
    this.box3.frame = 0;
    this.box3.visible=false;
    
    this.box4 = this.game.add.sprite(0,0, 'box');
    this.box4.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box4.x = controlPanel.x + controlPanel.width  - 2*box4.width - 160*sx;
    this.box4.x = controlPanel.x +controlPanel.width/2-60*this.sx + this.box1.width + 10*this.sx;
    this.box4.y = controlPanel.y + this.box3.y + 50*this.sx;
    this.box4.frame = 0;

    this.box5 = this.game.add.sprite(0,0, 'box');
    this.box5.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box5.x = controlPanel.x + controlPanel.width  - 3*box5.width - 180*sx;
    this.box5.x = controlPanel.x + controlPanel.width/2 -60*this.sx + this.box1.width + this.box3.width +2*10*this.sx;
    this.box5.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box5.height/2 - 50 *this.sx;
    this.box5.frame = 0;
    this.box5.visible = false;
    
    this.box6 = this.game.add.sprite(0,0, 'box');
    this.box6.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box6.x = controlPanel.x + controlPanel.width  - 3*box6.width - 180*sx;
    this.box6.x = controlPanel.x + controlPanel.width/2 -60*this.sx + this.box1.width + this.box3.width +2*10*this.sx;
    this.box6.y = controlPanel.y + this.box5.y + 50*this.sx;
    this.box6.frame = 0;
    
    this.box7 = this.game.add.sprite(0,0, 'box');
    this.box7.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box7.x = controlPanel.x + controlPanel.width  - 4*box7.width - 200*sx;
    this.box7.x = controlPanel.x + controlPanel.width/2-60*this.sx  + this.box1.width + this.box3.width + this.box5.width + 3*10*this.sx;
    this.box7.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box5.height/2 - 50 *this.sx;
    this.box7.frame = 0;
    this.box7.visible = false;
    
    this.box8 = this.game.add.sprite(0,0, 'box');
    this.box8.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box8.x = controlPanel.x + controlPanel.width  - 4*box8.width - 200*sx;
    this.box8.x = controlPanel.x + controlPanel.width/2-60*this.sx  + this.box1.width + this.box3.width + this.box5.width + 3*10*this.sx;
    this.box8.y = controlPanel.y + this.box7.y + 50*this.sx;
    this.box8.frame = 0;

    // create groups 
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    this.onBox4 = this.game.add.group();
    this.onBox5 = this.game.add.group();
    this.onBox6 = this.game.add.group();
    this.onBox7 = this.game.add.group();
    this.onBox8 = this.game.add.group();

    //generate each object on panel
    for (var i=0;i< OBJECT_AMOUNT;i++){
        this.panel.CoordX[i] = controlPanel.x + 220*this.sx +(i%COLUMNS_PANEL) * 180*this.sx;
        this.panel.CoordY[i] = controlPanel.y + controlPanel.height/2 - 120*this.sy + Math.floor(i/COLUMNS_PANEL) * 100*this.sy;
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }

    for (var i=0;i< OBJECT_AMOUNT;i++){
        this.sequenceObj[i] = this.objectname[i];
    }

    this.shuffle(this.sequenceObj);
    
    //create the sequence/pattern on box1/3/5/7
    this.onBox1.create(this.box1.x + this.box1.width/2, this.box1.y + this.box1.height/2 , this.sequenceObj[0]);
    this.onBox3.create(this.box3.x + this.box3.width/2, this.box3.y + this.box3.height/2 , this.sequenceObj[1]);
    this.onBox5.create(this.box5.x +this.box5.width/2, this.box5.y + this.box5.height/2 , this.sequenceObj[2]);
    this.onBox7.create(this.box7.x + this.box7.width/2, this.box7.y + this.box7.height/2 , this.sequenceObj[3]);
    
    
    this.onBox1.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    
    this.onBox3.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox5.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox7.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    

    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,controlPanel);
        //game.add.tween(object).to( { y: 400 }, 3000, Phaser.Easing.Cubic.InOut, true, 0, Number.MAX_VALUE, true);
        //object.input.enableSnap(120*sx,150*sy,false,true);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(OBJECT_AMOUNT);
};

Activity4.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
}

Activity4.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkSequence();
}

Activity4.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
}

Activity4.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
}
    
Activity4.prototype.onDragStop = function(sprite, pointer){
    //box1.frame = 0;
    this.box2.frame = 0;
    this.box4.frame = 0;
    this.box6.frame = 0;
    this.box8.frame = 0;
    sprite.frame = 0;
    
    if (this.onBox2.total < 1 && this.isInside(sprite,this.box2)   && (this.onPanel.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox6.removeChild(sprite) || this.onBox8.removeChild(sprite))){
        this.onBox2.add(sprite);
    }
    else if (this.onBox2.total <1 && !this.isInside(sprite,this.box2) && this.onBox2.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box2) && this.isInside(sprite,this.box4)){
        //do nothing
    }
    else if(this.onBox2.total == 1 && this.isInside(sprite, this.box2)  ){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox2 (sprite, this.onBox2.getChildAt(0));
    }
    else if (this.onBox2.total == 1 && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box6) && !this.isInside(sprite,this.box8) && this.onBox2.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else{
        console.log("something is wrong from box2 message-box or do nothing");
    }
    
    if (this.onBox4.total <1 && this.isInside(sprite,this.box4) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox6.removeChild(sprite) || this.onBox8.removeChild(sprite))){
        this.onBox4.add(sprite);
    }
    else if (this.onBox4.total< 1 && !this.isInside(sprite,this.box4)&& this.onBox4.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box4) && this.isInside(sprite,this.box6)){
        //do nothing
    }
    else if(this.onBox4.total == 1 && this.isInside(sprite, this.box4)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox4(sprite, this.onBox4.getChildAt(0));        
    }
    else if (this.onBox4.total == 1 && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box6) && !this.isInside(sprite,this.box8) && this.onBox4.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else{
        console.log("something is wrong from box4 message-box or do nothing");
    }
    
    
    if (this.onBox6.total <1 && this.isInside(sprite,this.box6) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox8.removeChild(sprite))){        
        this.onBox6.add(sprite);
    }
    else if (this.onBox6.total< 1 && !this.isInside(sprite,this.box6)&& this.onBox6.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box6) && this.isInside(sprite,this.box8)){
        //do nothing
    }
    else if(this.onBox6.total == 1 && this.isInside(sprite, this.box6)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox6(sprite, this.onBox6.getChildAt(0));
    }
    else if (this.onBox6.total == 1 && !this.isInside(sprite,this.box6) && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box8) &&  this.onBox6.removeChild(sprite)){    
        this.onPanel.add(sprite);
    }
    else {
        console.log("something is wrong from box6 message-box or do nothing");
    }
    
    if (this.onBox8.total <1 && this.isInside(sprite,this.box8) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox6.removeChild(sprite))){
        this.onBox8.add(sprite);
    }
    else if (this.onBox8.total< 1 && !this.isInside(sprite,this.box8)&& this.onBox8.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box6) && this.isInside(sprite,this.box8)){
        //do nothing
    }
    else if(this.onBox8.total == 1 && this.isInside(sprite, this.box8)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox8(sprite, this.onBox8.getChildAt(0) ) ;
    }
    else if (this.onBox8.total == 1 && !this.isInside(sprite,this.box8) && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box6) &&  this.onBox8.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else {
        console.log("something is wrong from box8 message-box or do nothing");
    }
    
    //set the position of objects in thier corresponding box / panel
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
        this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox2.total; i++)
    {
        this.onBox2.getChildAt(i).x = this.box2.x + this.box2.width/2;
        this.onBox2.getChildAt(i).y = this.box2.y + this.box2.height/2;
    }
    
    for (var i = 0; i < this.onBox4.total; i++)
    {
        this.onBox4.getChildAt(i).x = this.box4.x + this.box4.width/2;
        this.onBox4.getChildAt(i).y = this.box4.y + this.box4.height/2;
    }
    
    for (var i = 0; i < this.onBox6.total; i++)
    {
        this.onBox6.getChildAt(i).x = this.box6.x + this.box6.width/2;
        this.onBox6.getChildAt(i).y = this.box6.y + this.box6.height/2;
    }
    
    for (var i = 0; i < this.onBox8.total; i++)
    {
        this.onBox8.getChildAt(i).x = this.box8.x + this.box8.width/2;
        this.onBox8.getChildAt(i).y = this.box8.y + this.box8.height/2;
    }
    

}

Activity4.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite, this.box1)){
        this.box1.frame = 1;
    }
    else {
        this.box1.frame = 0;
    }
    
    if (this.isInside(sprite, this.box2)){
        this.box2.frame = 1;
    }
    else {
        this.box2.frame = 0;
    }
    
    if (this.isInside(sprite,this.box4)){
        this.box4.frame = 1;
    }
    else {
        this.box4.frame = 0;
    }
    
    if (this.isInside(sprite,this.box6)){
        this.box6.frame = 1;
    }
    else {
        this.box6.frame = 0;
    }
    
    if (this.isInside(sprite,this.box8)){
        this.box8.frame = 1;
    }
    else {
        this.box8.frame = 0;
    }

}

Activity4.prototype.swapWithBox2 = function(sprite, spriteInBox2){
    
    //if the sprite can be remove from onpanel group
    // add the sprite to box2 and add spriteInbox2 to onpanel 
    // remove the first child in box2
    if (this.onPanel.removeChild(sprite)){
        this.onBox2.addChild(sprite);
        this.onPanel.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    }
    else if (this.onBox4.removeChild(sprite)){
        this.onBox2.addChild(sprite);
        this.onBox4.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    
    }
    else if(this.onBox6.removeChild(sprite)){
        this.onBox2.addChild(sprite);
        this.onBox6.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    }
    else if (this.onBox8.removeChild(sprite) ){
        this.onBox2.addChild(sprite);
        this.onBox8.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}
Activity4.prototype.swapWithBox4 = function(sprite, spriteInBox4){
    if (this.onPanel.removeChild(sprite)){
        this.onBox4.addChild(sprite);
        this.onPanel.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox2.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox2.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if(this.onBox6.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox6.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox8.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox8.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}

Activity4.prototype.swapWithBox6 = function(sprite, spriteInBox6){

    if (this.onPanel.removeChild(sprite)  ){
        this.onBox6.addChild(sprite);
        this.onPanel.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox2.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onBox2.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if(this.onBox4.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onBox4.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox8.removeChild(sprite) ){
        this.onBox6.addChild(sprite);
        this.onBox8.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}

Activity4.prototype.swapWithBox8 = function(sprite, spriteInBox8 ){

    if (this.onPanel.removeChild(sprite)){
        this.onBox8.addChild(sprite);
        this.onPanel.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else if (this.onBox2.removeChild(sprite) ){
        this.onBox8.addChild(sprite);
        this.onBox2.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else if(this.onBox4.removeChild(sprite) ){
        this.onBox8.addChild(sprite);
        this.onBox4.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else if (this.onBox6.removeChild(sprite) ){
        this.onBox8.addChild(sprite);
        this.onBox6.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}
    
Activity4.prototype.checkOverlapWithRect = function(spriteA, rectangle) {
    var boundsA = spriteA.getBounds();
    return Phaser.Rectangle.intersects(boundsA, rectangle);
}

Activity4.prototype.isInside = function(object,box){
    if (object.x>box.x+0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity4.prototype.checkSequence = function(){
    
    //if the box are empty, display incorrect message 
    //avoid the case for children[0] being undefined whenever there is an empty box
    if(this.onBox2.children[0] == null || this.onBox4.children[0] == null || this.onBox6.children[0] == null || this.onBox8.children[0] == null){
        this.attempts++;
        this.publishRobotGameFeedback();
    }
    
    else if (this.onBox1.children[0].key == this.onBox2.children[0].key &&
        this.onBox3.children[0].key == this.onBox4.children[0].key &&
        this.onBox5.children[0].key == this.onBox6.children[0].key &&
        this.onBox7.children[0].key == this.onBox8.children[0].key )
    {
        this.publishRobotSuccessMessage();
        this.gameComplete();
        return true;
    }
    else{
        this.attempts++;
        this.publishRobotGameFeedback();
    }
    


}

Activity4.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< OBJECT_AMOUNT; i++){
        boxArrayX[i] = box.x+ 130 *this.sx+(i%4)* 150*this.sx;
        boxArrayY[i] = box.y+ 50 *this.sy + Math.floor(i/4)* 50 *this.sy;
    }
}

module.exports = Activity4;

},{"../../SARGame.js":1,"../../constants.js":2}],19:[function(require,module,exports){
/**
  * Space Ship Tidy Up Activity 5
  * -----------------------------
  *      
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 5;
const OBJECT_AMOUNT = 10;
const COLUMNS_PANEL = 5;
/**
 * Constructor for Activity 5
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity5(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onbox1;
    this.onbox2;

    this.box1;
    this.box2;
    this.box3;
    this.box4;
    this.box5;
    this.box6;                   

    //set a string array called objectname and shuffle them
    // tempObj is randomize
    this.tempObj1 = "object" + this.game.rnd.integerInRange(1,2);
    this.tempObj2 = "object" + this.game.rnd.integerInRange(3,4);
    this.tempObj3 = "object" + this.game.rnd.integerInRange(5,6);
    this.tempObj4 = "object" + this.game.rnd.integerInRange(1,6); 
    
    this.objectname  =  ['object1', 'object2', 'object3', 'object4' , 'object5', 'object6', this.tempObj1, this.tempObj2, this.tempObj3, this.tempObj4];
    this.shuffle(this.objectname);
    
    //set up a sequence array
    this.sequenceObj = [this.tempObj1, this.tempObj2, this.tempObj3];
    this.shuffle(this.sequenceObj);

    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity5.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity5.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');

    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Rocks/Moonrock1_sheet.png',610,400,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Rocks/Moonrock4_sheet.png',510,410,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Crystals/Crystal5_sheet.png',410,360,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Crystals/Crystal2_sheet.png',460,510,2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH+'Items/Stars/Star1_sheet.png',410,410,2);
    this.game.load.spritesheet('object6', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    this.game.load.spritesheet('bigBox', CONST.ASSETS_PATH+'Items/colorBox/BoxBlue_sheetnew.png',1510,510,2);
    this.game.load.spritesheet('test', CONST.ASSETS_PATH+'Items/BoxRotatedOver.png',720,871 ,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity5.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    var controlPanel = this.game.add.sprite(0,0,'controlPanel');
    controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    controlPanel.x = this.game.world.centerX - controlPanel.width/2;
    controlPanel.y = this.game.world.centerY - controlPanel.height/2;
    
    var button = this.game.add.sprite(0,0,'button');
    button.scale.setTo(0.3*this.sx);
    button.x = controlPanel.x+controlPanel.width/2-button.width/2;
    button.y = controlPanel.y+controlPanel.height-button.height-0.05*this.game.height;
    button.frame=0;
    button.inputEnabled=true;
    button.events.onInputDown.add(this.onButtonPress,this);
    button.events.onInputUp.add(this.onButtonRelease,this);
    
    this.box1 = this.game.add.sprite(0,0, 'box');
    this.box1.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box1.x = controlPanel.x + 220*this.sx; //controlPanel.x + 1*((controlPanel.width - 220*sx -220*sx - 80*sx )/ 6); 
    //box1.y = controlPanel.y + controlPanel.height/2 - box1.height/2 - button.height - 50 *sx;
    this.box1.y = controlPanel.y +controlPanel.height/8;
    this.box1.frame = 0;
    this.box1.visible = false;
                            
    var rec0 = this.game.add.sprite(0,0, 'bigBox');
    //rec0.scale.setTo(1.25*sx,0.25*sy);
    rec0.scale.setTo(0.755*this.sx,0.4*this.sy);
    rec0.x = this.box1.x - 40*this.sx;
    rec0.y = this.box1.y - 25*this.sy;//box1.y - 20*sy;
    rec0.frame = 0;
    rec0.visible = true;
    
    this.box2 = this.game.add.sprite(0,0, 'box');
    this.box2.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box2.x = this.box1.x + this.box1.width + 20*this.sx; //controlPanel.x +  2*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)
    this.box2.y = controlPanel.y +controlPanel.height/8;
    this.box2.frame = 0;
    this.box2.visible = false;
    
    this.box3 = this.game.add.sprite(0,0, 'box');
    this.box3.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box3.x = this.box2.x + this.box2.width + 20*this.sx; // controlPanel.x + 3*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)
    this.box3.y = controlPanel.y +controlPanel.height/8;
    this.box3.frame = 0;
    this.box3.visible=false;
    
    this.box4 = this.game.add.sprite(0,0, 'box');
    this.box4.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box4.x = this.box3.x + this.box3.width + 20*this.sx; // controlPanel.x + 4*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)
    this.box4.y = controlPanel.y +controlPanel.height/8;
    this.box4.frame = 0;
    //box4.visible = false;
    
    this.box5 = this.game.add.sprite(0,0, 'box');
    this.box5.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box5.x = this.box4.x + this.box4.width + 20*this.sx; //controlPanel.x  + 5*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)//box4.x + box4.width + 20*sx;
    this.box5.y = controlPanel.y +controlPanel.height/8;
    this.box5.frame = 0;
    //box5.visible = false;
    
    this.box6 = this.game.add.sprite(0,0, 'box');
    this.box6.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box6.x = this.box5.x + this.box5.width + 20*this.sx; //controlPanel.x + 6*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6) box5.x + box5.width + 20*sx;
    this.box6.y = controlPanel.y +controlPanel.height/8;
    this.box6.frame = 0;
    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    this.onBox4 = this.game.add.group();
    this.onBox5 = this.game.add.group();
    this.onBox6 = this.game.add.group();

    //generate each object on panel
    for (var i=0; i< OBJECT_AMOUNT; i++){
        this.panel.CoordX[i] = controlPanel.x + controlPanel.width/2 - 400*this.sx +(i%COLUMNS_PANEL) * 180*this.sx;
        this.panel.CoordY[i] = controlPanel.y + controlPanel.height/2 + Math.floor(i/COLUMNS_PANEL) * 140*this.sy;
        
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }
    

    //create the sequence/pattern on box1/2/3/4
    this.onBox1.create(this.box1.x + this.box1.width/2, this.box1.y + this.box1.height/2, this.sequenceObj[0]);
    this.onBox2.create(this.box2.x + this.box2.width/2, this.box2.y + this.box2.height/2, this.sequenceObj[1]);
    this.onBox3.create(this.box3.x + this.box3.width/2, this.box3.y + this.box3.height/2, this.sequenceObj[2]);
    
    this.onBox1.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    
    this.onBox2.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox3.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox4.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox5.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox6.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    

    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,controlPanel);
        //game.add.tween(object).to( { y: 400 }, 3000, Phaser.Easing.Cubic.InOut, true, 0, Number.MAX_VALUE, true);
        //object.input.enableSnap(120*sx,150*sy,false,true);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(OBJECT_AMOUNT);
};

Activity5.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');

}


Activity5.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkSequence();
}


Activity5.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
}

Activity5.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
}
    
Activity5.prototype.onDragStop = function(sprite, pointer){
    this.box4.frame = 0;
    this.box5.frame = 0;
    this.box6.frame = 0;
    sprite.frame = 0;                   
    
    if (this.onBox4.total <1 && this.isInside(sprite,this.box4) && (this.onPanel.removeChild(sprite) || this.onBox5.removeChild(sprite) || this.onBox6.removeChild(sprite))){        
        this.onBox4.add(sprite);
    }
        
    else if (this.onBox4.total< 1 && !this.isInside(sprite,this.box4)&& this.onBox4.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box4) && this.isInside(sprite,this.box5)){
        //do nothing
    }
    else if(this.onBox4.total == 1 && this.isInside(sprite, this.box4)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox4(sprite, this.onBox4.getChildAt(0));
    }
    else if (this.onBox4.total == 1 && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box5) && !this.isInside(sprite,this.box6) && this.onBox4.removeChild(sprite)  ){
        this.onPanel.add(sprite);
    }
    else {
        console.log("something is wrong from box4 message-box or do nothing");
    }
    
    if (this.onBox5.total <1 && this.isInside(sprite,this.box5) && (this.onPanel.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox6.removeChild(sprite))){
        this.onBox5.add(sprite);
    }
    else if (this.onBox5.total< 1 && !this.isInside(sprite,this.box5)&& this.onBox5.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box5) && this.isInside(sprite,this.box6)){
        //do nothing
    }
    else if(this.onBox5.total == 1 && this.isInside(sprite, this.box5)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox5(sprite, this.onBox5.getChildAt(0));
    }
    else if (this.onBox5.total == 1 && !this.isInside(sprite,this.box5) && !this.isInside(sprite,this.box6) && this.onBox5.removeChild(sprite)  ){
        this.onPanel.add(sprite);
    }
    else {
            console.log("something is wrong from box5 message-box or do nothing");
    }
    
    if (this.onBox6.total<1 && this.isInside(sprite, this.box6) &&  (this.onPanel.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox5.removeChild(sprite))){    
        this.onBox6.add(sprite);
    }
    else if(this.onBox6.total < 1 && !this.isInside(sprite, this.box6) && this.onBox6.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box5) && this.isInside(sprite,this.box6)){
        //do nothing
    }
    else if(this.onBox6.total ==1 && this.isInside(sprite,this.box6)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox6(sprite, this.onBox6.getChildAt(0));
    }
    else if (this.onBox6.total == 1 && !this.isInside(sprite,this.box6)  && !this.isInside(sprite,this.box5) &&  this.onBox6.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else{
        console.log("do nothing");
    }
    
    //set the position of objects in thier corresponding box / panel
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
        this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox4.total; i++)
    {
        this.onBox4.getChildAt(i).x = this.box4.x + this.box4.width/2;
        this.onBox4.getChildAt(i).y = this.box4.y + this.box4.height/2;
    }
    
    for (var i = 0; i < this.onBox5.total; i++)
    {
        this.onBox5.getChildAt(i).x = this.box5.x + this.box5.width/2;
        this.onBox5.getChildAt(i).y = this.box5.y + this.box5.height/2;
    }
    

    for (var i = 0; i < this.onBox6.total; i++)
    {
        this.onBox6.getChildAt(i).x = this.box6.x + this.box6.width/2;
        this.onBox6.getChildAt(i).y = this.box6.y + this.box6.height/2;
    }
}

Activity5.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite, this.box4)){
        this.box4.frame = 1;
    }
    else if (!this.isInside(sprite, this.box4)){
        this.box4.frame = 0;
    }
    
    if (this.isInside(sprite, this.box5)){
        this.box5.frame = 1;
    }
    else if (!this.isInside(sprite, this.box5)){
        this.box5.frame = 0;
    }

    if (this.isInside(sprite, this.box6)){
        this.box6.frame = 1;
    }
    else if (!this.isInside(sprite,this.box6)){
        this.box6.frame = 0;
    }
}

Activity5.prototype.swapWithBox4 = function(sprite, spriteInBox4 ){
    
    //if the sprite can be remove from onpanel group
    // add the sprite to box4 and add spriteInbox5 to onpanel 
    // remove the first child in box4
    if (this.onPanel.removeChild(sprite)){
        this.onBox4.addChild(sprite);
        this.onPanel.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox5.removeChild(sprite)){
        this.onBox4.addChild(sprite);
        this.onBox5.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox6.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox6.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}

Activity5.prototype.swapWithBox5 = function(sprite, spriteInBox5 ){
    if (this.onPanel.removeChild(sprite)){
        this.onBox5.addChild(sprite);
        this.onPanel.addChild(spriteInBox5);
        this.onBox5.removeChild(spriteInBox5);
    }
    else if (this.onBox4.removeChild(sprite)){
        this.onBox5.addChild(sprite);
        this.onBox4.addChild(spriteInBox5);
        this.onBox5.removeChild(spriteInBox5);
    }
    
    else if (this.onBox6.removeChild(sprite) ){
        this.onBox5.addChild(sprite);
        this.onBox6.addChild(spriteInBox5);
        this.onBox5.removeChild(spriteInBox5);
    }
}

Activity5.prototype.swapWithBox6 = function(sprite, spriteInBox6 ){
    if (this.onPanel.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onPanel.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox4.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onBox4.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox5.removeChild(sprite) ){
        this.onBox6.addChild(sprite);
        this.onBox5.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else{
        console.log("random error I didnt consider or do nothing");
    }
}
    

Activity5.prototype.isInside = function(object,box){
    if (object.x>box.x+0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity5.prototype.checkSequence = function(){
    //if the box are empty, display incorrect message 
    //avoid the case for children[0] being undefined whenever there is an empty box
    if(this.onBox4.children[0] == null || this.onBox5.children[0] == null || this.onBox6.children[0] == null ){
        this.attempts++;
        alert("Incorrect, Please fill up the empty box");
        this.publishRobotGameFeedback();
    }
    else if (this.onBox1.children[0].key == this.onBox4.children[0].key &&
        this.onBox2.children[0].key == this.onBox5.children[0].key &&
        this.onBox3.children[0].key == this.onBox6.children[0].key  )
    {
        this.publishRobotSuccessMessage();
        this.gameComplete();
        return true;
    }
    else{
        this.attempts++;
        alert("Incorrect, wrong sequence");
        this.publishRobotGameFeedback();
    }
}

module.exports = Activity5;

},{"../../SARGame.js":1,"../../constants.js":2}],20:[function(require,module,exports){
'use strict';


var Activity1 = require('./activity1.js');
var Activity2 = require('./activity2.js');
var Activity3 = require('./activity3.js');
var Activity4 = require('./activity4.js');
var Activity5 = require('./activity5.js');
var Activity6 = require('./activity1.js');
var Activity7 = require('./activity2.js');
var Activity8 = require('./activity3.js');
var Activity9 = require('./activity4.js');
var Activity10 = require('./activity5.js');

var GameWrapper = require('../../SARGame.js').GameWrapper;
var CONST = require('../../constants.js');


function SpaceshipTidyup(ros, level, exitCallBack) {
    GameWrapper.call(this, ros, level, exitCallBack);
    this.gameid = CONST.Games.SPACESHIP_TIDYUP;
    // this.activities = [Activity1, Activity2];
    this.activities = [Activity1, Activity2, Activity3, Activity4, Activity5, Activity6, Activity7, Activity8, Activity9, Activity10];
    this.start();
}

SpaceshipTidyup.prototype.__proto__ = GameWrapper.prototype;

module.exports = SpaceshipTidyup;

},{"../../SARGame.js":1,"../../constants.js":2,"./activity1.js":15,"./activity2.js":16,"./activity3.js":17,"./activity4.js":18,"./activity5.js":19}],21:[function(require,module,exports){
'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');


function AlienCodesTutorial(ros, completionCallBack) {
    // Call Parent constructor - no level, gameID, or activityid
    SARGame.call(this, ros, null, null, completionCallBack, null);

    var gameWidth = 1920;
    var gameHeight = 1080;

    this.gameid = CONST.Games.ALIEN_CODES;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;

    this.inBox = false;
    this.partOne = true;
    this.partTwo = false;
}
AlienCodesTutorial.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
AlienCodesTutorial.prototype.preload = function() {
    // Background Assets
    this.game.load.image('spaceship_bg', CONST.ASSETS_PATH + 'Background/HomeScreen.png');
    this.game.load.image('shipInside_bg', CONST.ASSETS_PATH + 'Background/ShipInterior.png');

    // UI Assets
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanel.png');
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    this.game.load.image('container', CONST.ASSETS_PATH + 'Items/BoxLong.png');
    this.game.load.image('clear', CONST.ASSETS_PATH + 'Items/Clear.png');
    this.game.load.image('right', CONST.ASSETS_PATH + 'Items/Right.png');
    this.game.load.image('backButton', CONST.ASSETS_PATH + 'Items/Undo.png');

    // Sprites
    this.game.load.spritesheet('crystal', CONST.ASSETS_PATH+'Items/Crystals/Crystal4_sheet.png',430,410,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};

// Entry point for tutorial. Start with Spaceship BG
AlienCodesTutorial.prototype.create  = function() {
    this.background = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'spaceship_bg');
    this.background.anchor.setTo(0.5, 0.5);
    console.log('Game width is '+this.game.width);
    console.log('Game height is '+this.game.height);

    this.backButton = this.game.add.sprite(50, 50, 'backButton');
    this.backButton.frame = 0;
    this.backButton.scale.setTo(0.2);
    this.backButton.inputEnabled = true;
    this.backButton.events.onInputUp.add(this.onBackButton, this);

    this.publishTutorialSequenceBlocking(this.spaceShipBackground.bind(this));
};

AlienCodesTutorial.prototype.spaceShipBackground = function() {
    this.background.destroy();
    this.background = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'shipInside_bg');
    this.background.anchor.setTo(0.5, 0.5);

    this.game.world.bringToTop(this.backButton);

    this.publishTutorialSequenceBlocking(this.controlPanelBackground.bind(this));
};

AlienCodesTutorial.prototype.controlPanelBackground = function() {
    // Below is the box for feedback to the user.
    this.controlPanel = this.game.add.sprite(this.game.world.centerX - 1075 * this.sx, this.game.world.centerY - 250 * this.sy, 'controlPanel');
    this.controlPanel.scale.setTo(1.125 * this.sx, 0.37 * this.sy);

    this.publishTutorialSequenceBlocking(this.controlPanelItems.bind(this));
};


AlienCodesTutorial.prototype.controlPanelItems = function() {
    // Verify/Finish button - pressed when the user is done with task
    var verifyButton = this.game.add.button(this.game.world.centerX + 375 * this.sx,
                                            this.game.world.centerY + 260 * this.sy,
                                            'button', this.onButtonPress, this, 0, 0, 1, 0);
    verifyButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    var right = this.game.add.sprite(this.game.world.centerX + 418 * this.sx, this.game.world.centerY + 297 * this.sy, 'right');
    right.scale.setTo(0.1 * this.sx, 0.1 * this.sy);

    // Container is the box holding the items the user will click and drag around
    this.container = this.game.add.sprite(this.game.world.centerX - 375 * this.sx, this.game.world.centerY + 200 * this.sy, 'container');
    this.container.scale.setTo(0.5 * this.sx, 0.6 * this.sy);
    this.container.GridX = this.game.world.centerX - 350 * this.sx;
    this.container.GridY = this.game.world.centerY + 240 * this.sy;

    // Deposit box
    this.box = this.game.add.sprite(this.game.world.centerX - (700 * this.sx), this.game.world.centerY - 90 * this.sy, 'box');
    this.box.scale.setTo(0.23 * this.sx, 0.23 * this.sy);
    this.box.GridX = this.box.x + (this.box.width / 4);
    this.box.GridY = this.box.y + (this.box.height / 4);

    this.crystal = this.game.add.sprite(this.container.GridX, this.container.GridY, 'crystal');
    this.crystal.scale.setTo(0.25 * this.sx, 0.25 * this.sy);
    this.crystal.frame = 0;

    this.publishTutorialSequenceBlocking(() => this.publishTutorialSequenceBlocking(() => {
        this.crystal.inputEnabled = true;
        this.crystal.input.enableDrag(false,false,false,255,null, null);
        this.crystal.events.onDragStart.add(this.onDragStart,this);
        this.crystal.events.onDragStop.add(this.onDragStop,this);
        this.crystal.events.onDragUpdate.add(this.onDragUpdate,this);
    }));
};

AlienCodesTutorial.prototype.mistakeSegment = function() {
    this.partTwo = true;
    // Reset button
    var resetButton = this.game.add.button(this.game.world.centerX - 550 * this.sx,
                                           this.game.world.centerY + 260 * this.sy,
                                           'button', this.resetImages, this, 0, 0, 1, 0);
    resetButton.scale.setTo(0.6 * this.sx, 0.6 * this.sy);
    resetButton.inputEnabled = false;
    var clear = this.game.add.sprite(this.game.world.centerX - 535 * this.sx, this.game.world.centerY + 285 * this.sy, 'clear');
    clear.scale.setTo(0.25 * this.sx, 0.25 * this.sy);
    this.publishTutorialSequenceBlocking(() => this.publishTutorialSequenceBlocking(() => resetButton.inputEnabled = true));
}

AlienCodesTutorial.prototype.checkImages = function() {
    return;
}

AlienCodesTutorial.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;
};

AlienCodesTutorial.prototype.onDragStop = function(sprite, pointer){
    this.box.frame = 0;
    sprite.frame = 0;

    if (this.isInside(sprite, this.box)) {
        sprite.x = this.box.GridX;
        sprite.y = this.box.GridY;
        this.inBox = true;
    }
    else {
        sprite.x = this.container.GridX;
        sprite.y = this.container.GridY;
        this.inBox = false;
    }
}


AlienCodesTutorial.prototype.onDragUpdate = function (sprite,pointer){
    if (this.isInside(sprite, this.box)) {
        this.box.frame = 1;
    }
    else {
        this.box.frame = 0;
    }
}

AlienCodesTutorial.prototype.onButtonPress = function(sprite,pointer){

    if (this.partOne && this.inBox){
        this.partOne = false;
        this.publishRobotSuccessMessage();
        //publish the star removing instructions
        this.mistakeSegment();
    } else if (this.partTwo && !this.inBox) {
        this.publishTutorialSequenceBlocking(() => this.gameComplete());
    } else {
        // TODO PUBLISH FAIL MESSAGE??
        // maybe we should say "Before pressing the button, make sure to move the star into the box." for part one, etc
    }
};

AlienCodesTutorial.prototype.onBackButton = function() {
    this.gameComplete();
}

AlienCodesTutorial.prototype.resetImages = function() {
    this.crystal.x = this.container.GridX;
    this.crystal.y = this.container.GridY;
    this.inBox = false;
    if (this.partTwo) {
        this.publishTutorialSequenceBlocking(() => this.gameComplete());
    }
}

AlienCodesTutorial.prototype.isInside = function(object, box){
    return Phaser.Rectangle.intersects(object.getBounds(), box.getBounds());
}
module.exports = AlienCodesTutorial;

},{"../../SARGame.js":1,"../../constants.js":2}],22:[function(require,module,exports){
'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');


function GalacticTravelerTutorial(ros, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, -1, -1, completionCallBack, -1);

    var gameWidth = 1920;
    var gameHeight = 1080;

    this.gameid = CONST.Games.GALACTIC_TRAVELER;

    // Declare object properties
    this.objects, this.spaceship;
    this.currentCount = 0;
    this.target;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;

    this.inBox = false;
    this.partOne = true;
    this.partTwo = false;
}
GalacticTravelerTutorial.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
GalacticTravelerTutorial.prototype.preload = function() {
    this.game.load.image('splash', CONST.ASSETS_PATH + 'Background/HomeScreen.png');
    this.game.load.image('atmo', CONST.ASSETS_PATH + 'Background/Atmosphere1.png');
    this.game.load.image('shipInside', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');
    this.game.load.image('backButton', CONST.ASSETS_PATH+'Items/Undo.png');
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);

    this.game.load.spritesheet('star1', CONST.ASSETS_PATH+'Items/Stars/Star1_sheet.png',410,410,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
GalacticTravelerTutorial.prototype.create  = function() {
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'splash');
    this.logo.anchor.setTo(0.5, 0.5);

    this.backButton = this.game.add.sprite(50, 50, 'backButton');
    this.backButton.frame = 0;
    this.backButton.scale.setTo(0.2);
    this.backButton.inputEnabled=true;
    this.backButton.events.onInputUp.add(this.onBackButton,this);

    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    this.publishTutorialSequenceBlocking(this.spaceShipBackground.bind(this));
};

GalacticTravelerTutorial.prototype.onBackButton = function() {
    this.gameComplete();
};

GalacticTravelerTutorial.prototype.spaceShipBackground = function() {
    this.logo.destroy();
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'shipInside');
    this.logo.anchor.setTo(0.5, 0.5);
    this.game.world.bringToTop(this.backButton);

    this.publishTutorialSequenceBlocking(this.controlPanelBackground.bind(this));
};

GalacticTravelerTutorial.prototype.controlPanelBackground = function() {
    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;

    this.publishTutorialSequenceBlocking(this.controlPanelItems.bind(this));
};

GalacticTravelerTutorial.prototype.controlPanelItems = function() {
    //button image is loaded
    this.button = this.game.add.sprite(this.controlPanel.x + this.controlPanel.width/2,
                                       this.controlPanel.y + this.controlPanel.height-200, 'button');
    this.button.scale.setTo(0.3);
    this.button.x = this.controlPanel.x + this.controlPanel.width / 2 - this.button.width / 2;
    this.button.y = this.controlPanel.y + this.controlPanel.height - this.button.height-0.05 * this.game.height;
    //button frame 
    this.button.frame=0;
    //Enables all kind of input actions on this image (click, etc)
    this.button.inputEnabled=true;
    //onButtonPress function
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);

    this.box = this.game.add.sprite(0,0, 'box');
    this.box.scale.setTo(0.6 * this.sx, 0.8 * this.sy);
    this.box.x = this.controlPanel.x + this.controlPanel.width - this.box.width - 130 * this.sx;
    this.box.y = this.controlPanel.y + 80 * this.sx;
    this.box.inputEnabled = true;
    this.box.frame = 0;

    this.onPanel = this.game.add.group();
    this.onBox = this.game.add.group();

    //create one star
    var object = this.onPanel.create(this.controlPanel.x + 220, this.controlPanel.y + 130, "star1");
    object.anchor.setTo(0.5,0.5);
    object.frame = 0;
    object.scale.setTo(0.25 * this.sx);
    //object.originalPosition = object.position.clone();
    //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
    //object.input.enableSnap(130 *sx,150 *sy,false,true);

    this.publishTutorialSequenceBlocking(() => this.publishTutorialSequenceBlocking(() => {
        object.inputEnabled = true;
        object.input.enableDrag(false,false,false,255,null, this.controlPanel);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    }));
};

GalacticTravelerTutorial.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;

    if (this.partOne && this.inBox){
        this.partOne = false;
        this.publishRobotSuccessMessage();
        //publish the star removing instructions
        this.publishTutorialSequenceBlocking(() => this.publishTutorialSequenceBlocking(() => this.partTwo = true));
    } else if (this.partTwo && !this.inBox) {
        this.publishTutorialSequenceBlocking(() => this.gameComplete());
    } else {
        // TODO PUBLISH FAIL MESSAGE??
        // maybe we should say "Before pressing the button, make sure to move the star into the box." for part one, etc
    }
};

GalacticTravelerTutorial.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
};

GalacticTravelerTutorial.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;
};

GalacticTravelerTutorial.prototype.onDragStop = function(sprite,pointer){
    sprite.frame=0;
    this.box.frame = 0;
                
    if (this.isInside(sprite))
    {
        if (this.onPanel.removeChild(sprite)) {
            this.onBox.add(sprite);
            this.inBox = true;
        }
        this.onBox.getChildAt(0).x = this.controlPanel.x + 960 * this.sx;
        this.onBox.getChildAt(0).y = this.controlPanel.y + 160 * this.sy;
    } else if (!this.isInside(sprite))
    {
        if (this.onBox.removeChild(sprite)) {
            this.onPanel.add(sprite);
            this.inBox = false;
        }
        this.onPanel.getChildAt(0).x = this.controlPanel.x + 220;
        this.onPanel.getChildAt(0).y = this.controlPanel.y + 130;
    }
}

GalacticTravelerTutorial.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite)) this.box.frame = 1;
    else if (!this.isInside(sprite)){
        this.box.frame = 0;
    }
}


GalacticTravelerTutorial.prototype.isInside = function(object){
    if (object.x > this.box.x+0.01*this.game.width && 
        object.y > this.box.y +0.01*this.game.height && 
        object.x < this.box.x+this.box.width-0.01*this.game.width && 
        object.y < this.box.y+this.box.height-0.01*this.game.height) return true;
    else return false;
}

module.exports = GalacticTravelerTutorial;
},{"../../SARGame.js":1,"../../constants.js":2}],23:[function(require,module,exports){
'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');


function SpaceshipTidyupTutorial(ros, completionCallBack) {
    // Call Parent constructor - no level, gameID, or activityid
    SARGame.call(this, ros, null, null, completionCallBack, null);

    var gameWidth = 1920;
    var gameHeight = 1080;

    this.gameid = CONST.Games.SPACESHIP_TIDYUP;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;

    this.inBox = false;
    this.partOne = true;
    this.partTwo = false;
}
SpaceshipTidyupTutorial.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
SpaceshipTidyupTutorial.prototype.preload = function() {
    // Background Assets
    this.game.load.image('spaceship_bg', CONST.ASSETS_PATH + 'Background/HomeScreen.png');
    this.game.load.image('shipInside_bg', CONST.ASSETS_PATH + 'Background/ShipInterior.png');

    // UI Assets
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    this.game.load.image('backButton', CONST.ASSETS_PATH + 'Items/Undo.png');

    // Sprites
    this.game.load.spritesheet('star', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('moonrock', CONST.ASSETS_PATH+'Items/Rocks/Moonrock2_sheet.png',460,360,2);
    this.game.load.spritesheet('crystal', CONST.ASSETS_PATH+'Items/Crystals/Crystal3_sheet.png',410,460,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
SpaceshipTidyupTutorial.prototype.create  = function() {
    this.background = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'spaceship_bg');
    this.background.anchor.setTo(0.5, 0.5);
    console.log('Game width is '+this.game.width);
    console.log('Game height is '+this.game.height);

    this.backButton = this.game.add.sprite(50, 50, 'backButton');
    this.backButton.frame = 0;
    this.backButton.scale.setTo(0.2);
    this.backButton.inputEnabled = true;
    this.backButton.events.onInputUp.add(this.onBackButton, this);

    this.publishTutorialSequenceBlocking(this.spaceShipBackground.bind(this));
};

SpaceshipTidyupTutorial.prototype.spaceShipBackground = function() {
    this.background.destroy();
    this.background = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'shipInside_bg');
    this.background.anchor.setTo(0.5, 0.5);

    this.game.world.bringToTop(this.backButton);

    this.publishTutorialSequenceBlocking(this.controlPanelBackground.bind(this));
};

SpaceshipTidyupTutorial.prototype.controlPanelBackground = function() {
    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;

    this.publishTutorialSequenceBlocking(this.controlPanelItems.bind(this));
};

SpaceshipTidyupTutorial.prototype.controlPanelItems = function() {
    //button image is loaded
    this.button = this.game.add.sprite(this.controlPanel.x + this.controlPanel.width/2,
                                       this.controlPanel.y + this.controlPanel.height-200, 'button');
    this.button.scale.setTo(0.3);
    this.button.x = this.controlPanel.x + this.controlPanel.width / 2 - this.button.width / 2;
    this.button.y = this.controlPanel.y + this.controlPanel.height - this.button.height-0.05 * this.game.height;
    //button frame 
    this.button.frame=0;
    //Enables all kind of input actions on this image (click, etc)
    this.button.inputEnabled=true;
    //onButtonPress function
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);

    // Create 3 deposit boxes
    this.box1 = this.createBox(0, 0);
    var posX = this.controlPanel.x + this.controlPanel.width  - this.box1.width - 140*this.sx; // X-pos of all boxes
    this.box1.x = posX;
    this.box1.y = this.controlPanel.y + this.controlPanel.height/3 - this.box1.height/3 - this.button.height - 50 *this.sx;
    this.box2 = this.createBox(posX, this.controlPanel.y + this.box1.y + 50 * this.sx);
    this.box3 = this.createBox(posX, this.controlPanel.y + this.box2.y + 50 * this.sx);
    
    // Add item labels to deposit boxes (moonrock, crystal, and star label)
    var upperLHItem1 = this.game.add.sprite(this.box1.x, this.box1.y, 'moonrock');
    upperLHItem1.scale.setTo(0.1*this.sx,0.1*this.sy);
    var upperLHItem2 = this.game.add.sprite(this.box2.x, this.box2.y, 'star');
    upperLHItem2.scale.setTo(0.1*this.sx,0.1*this.sy);
    var upperLHItem3 = this.game.add.sprite(this.box3.x, this.box3.y, 'crystal');
    upperLHItem3.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();

    //create one star
    var star = this.onPanel.create(this.controlPanel.x + 220, this.controlPanel.y + 130, "star");
    star.anchor.setTo(0.5,0.5);
    star.frame = 0;
    star.scale.setTo(0.25 * this.sx);
    star.originalPosition = star.position.clone();
 
    this.publishTutorialSequenceBlocking(() => this.publishTutorialSequenceBlocking(() => {
        star.inputEnabled = true;
        star.input.enableDrag(false,false,false,255,null, this.controlPanel);
        star.events.onDragStart.add(this.onDragStart,this);
        star.events.onDragStop.add(this.onDragStop,this);
        star.events.onDragUpdate.add(this.onDragUpdate,this);
    }));
};

SpaceshipTidyupTutorial.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;

    if (this.partOne && this.inBox){
        this.partOne = false;
        this.publishRobotSuccessMessage();
        //publish the star removing instructions
        this.publishTutorialSequenceBlocking(() => this.publishTutorialSequenceBlocking(() => this.partTwo = true));
    } else if (this.partTwo && !this.inBox) {
        this.publishTutorialSequenceBlocking(() => this.gameComplete());
    } else {
        // TODO PUBLISH FAIL MESSAGE??
        // maybe we should say "Before pressing the button, make sure to move the star into the box." for part one, etc
    }
};

SpaceshipTidyupTutorial.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
};

SpaceshipTidyupTutorial.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;
};

SpaceshipTidyupTutorial.prototype.onDragStop = function(sprite, pointer){
    this.inBox = this.partOne ? false : true; // default to false during part one (getting star into box)
    this.box1.frame = 0;
    this.box2.frame = 0;
    this.box3.frame = 0;
    sprite.frame = 0;
    
    
    // Move sprite into the appropriate control panel or deposit box group depending on location
    if(this.isInside(sprite, this.box1) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox3.removeChild(sprite))){
        this.onBox1.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box1) && this.onBox1.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    if(this.isInside(sprite,this.box2 )&& (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox3.removeChild(sprite))){
        this.onBox2.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box2) && this.onBox2.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    if(this.isInside(sprite,this.box3) && (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox2.removeChild(sprite))){
        this.onBox3.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box3) && this.onBox3.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    // Update the children object's location to simulate snapping to grid
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.controlPanel.x + 220;
        this.onPanel.getChildAt(i).y = this.controlPanel.y + 130;
        this.inBox = false;
    }
    
    for (var i = 0; i < this.onBox1.total; i++)
    {
        this.onBox1.getChildAt(i).x = this.box1.x + 130 * this.sx;
        this.onBox1.getChildAt(i).y = this.box1.y + this.box1.height / 2;
    }
    
    for (var i = 0; i < this.onBox2.total; i++)
    {
        this.onBox2.getChildAt(i).x = this.box2.x + 130 * this.sx;
        this.onBox2.getChildAt(i).y = this.box2.y + this.box2.height / 2;
        this.inBox = true;
    }
    
    for (var i = 0; i < this.onBox3.total; i++)
    {
        this.onBox3.getChildAt(i).x = this.box3.x + 130 * this.sx;
        this.onBox3.getChildAt(i).y = this.box3.y + this.box3.height / 2;
    }

    
}


SpaceshipTidyupTutorial.prototype.onDragUpdate = function (sprite,pointer){
    for (let box of [this.box1, this.box2, this.box3]) {
        if (this.isInside(sprite, box)){
            box.frame = 1;
        }
        else if (!this.isInside(sprite, box)) {
            box.frame = 0;
        }
    }
}

SpaceshipTidyupTutorial.prototype.onBackButton = function() {
    this.gameComplete();
}

SpaceshipTidyupTutorial.prototype.isInside = function(object, box){
    if (object.x > box.x+0.01*this.game.width && 
        object.y > box.y +0.01*this.game.height && 
        object.x < box.x+box.width-0.01*this.game.width && 
        object.y < box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

// Creates a new deposit box
SpaceshipTidyupTutorial.prototype.createBox = function(posX, posY) {
    var box = this.game.add.sprite(posX, posY, 'box');
    box.scale.setTo(0.8*this.sx,0.15*this.sy);
    box.frame = 0;
    return box;
}
module.exports = SpaceshipTidyupTutorial;

},{"../../SARGame.js":1,"../../constants.js":2}],24:[function(require,module,exports){
var SAR = require('./SARGame');

exports.constants = require('./constants.js');
exports.SARGame = SAR.SARGame;
exports.LoadingScreen = SAR.LoadingScreen;
exports.Menu = SAR.Menu;

// Games
exports.GalacticTraveler = require('./games/gt/main.js');
exports.SpaceshipTidyup = require('./games/st/main.js');
exports.AlienCodes = require('./games/ac/main.js');

// Tutorials
exports.GalacticTravelerTutorial = require('./games/tutorial/gt_tutorial.js');
exports.SpaceshipTidyupTutorial = require('./games/tutorial/st_tutorial.js');
exports.AlienCodesTutorial = require('./games/tutorial/ac_tutorial.js');

},{"./SARGame":1,"./constants.js":2,"./games/ac/main.js":8,"./games/gt/main.js":14,"./games/st/main.js":20,"./games/tutorial/ac_tutorial.js":21,"./games/tutorial/gt_tutorial.js":22,"./games/tutorial/st_tutorial.js":23}],25:[function(require,module,exports){
var CONST = require('./constants.js');

exports.SUCCESS = [
    {
        msg: "<bounce,nb> Good job!",
        id: "congratulatory_1"
    },
    {
        msg: "<smile,nb> Great work!",
        id: "congratulatory_2"
    },
    {
        msg: "<smile,nb> Thanks for helping me!",
        id: "congratulatory_3"
    },
    {
        msg: "<excited,nb> Fantastic!",
        id: "congratulatory_4"
    },
    {
        msg: "<excited,nb> Excellent!",
        id: "congratulatory_5"
    },
    {
        msg: "<smile,nb> You're doing great!",
        id: "congratulatory_6"
    },
    {
        msg: "<bounce,nb> Awesome!",
        id: "congratulatory_7"
    },
    {
        msg: "<excited,nb> Perfect!",
        id: "congratulatory_8"
    },
    {
        msg: "<smile,nb> Amazing!",
        id: "congratulatory_9"
    },
    {
        msg: "<excited,nb> Super!",
        id: "congratulatory_10"
    },
];

exports.GENERAL_FEEDBACK = [
    {
        msg: "Try again!",
        id: "incorrect_1"
    },
    {
        msg: "Not quite. Try again!",
        id: "incorrect_2"
    },
    {
        msg: "Close, but not quite. Try again!",
        id: "incorrect_3"
    },
    {
        msg: "Well, I don't think that's correct. Let's try again!",
        id: "incorrect_4"
    },
    {
        msg: "Whoops! That's not right. How about we try again.",
        id: "incorrect_5"
    },
];

exports.GAMES = {
    GALACTIC_TRAVELER: {
        1: {
            instructions: [
                {
                    msg: "I want to bring moon rocks home <smile,nb> to show my friends! Will you help me pack some? <bounce,nb> Put {0} moon rock into the box!",
                    id: "game1_act1_inst0_{0}"
                },
            ],
            feedback: [
                {
                    msg: "Almost! We need {1} moon rocks, {0} total. Try again!",
                    id: "game1_act1_feedback1_{1}_{0}"
                },
                {
                    msg: "We are missing a few moon rocks or have some extra. Let's fix that.",
                    id: "game1_act1_feedback3"
                },
                {
                    msg: "We are missing moon rocks or we have too many. Try to have {1} moonrocks.",
                    id: "game1_act1_feedback4_{1}"
                },
                {
                    msg: "I think we didn't pack the right number of moon rocks, we need {0}. Let's try again.",
                    id: "game1_act1_feedback5_{0}"
                },
                {
                    msg: "We don't have the right number of moon rocks. Let's try fixing that with {1} moon rocks.",
                    id: "game1_act1_feedback6_{1}"
                },
                {
                    msg: "I think we need to fix the number of moom rocks by having {1}.",
                    id: "game1_act1_feedback7_{1}"
                },
            ]
        },
        2: {
            instructions: [
                {
                    msg: "It's <concerned,nb> a long trip home! So I need to <excited,nb> fully charge my spaceship! What number do you see? Place {0} energy {1} into the battery pack to charge the spaceship.",
                    id: "game1_act2_inst1_{0}"
                },
                {
                    msg: "<calm,nb> What number do <laugh,nb> you see? Put {0} energy {1} into the battery pack.",
                    id: "game1_act2_inst2_{0}"
                },
                {
                    msg: "<speak,nb> Put {0} energy {1} into the battery pack to charge the spaceship. <bounce,nb>",
                    id: "game1_act2_inst3_{0}"
                },
                {
                    msg: "<speak,nb> Place {0} energy {1} into the <laugh,nb> battery pack. ",
                    id: "game1_act2_inst4_{0}"
                },
                {
                    msg: "<scared,nb> Help me charge <speak,nb> my spaceship! Put {0} space {1} into the battery pack.",
                    id: "game1_act2_inst5_{0}"
                },
            ],
            feedback: [
                {
                    msg: "Close, but we need {1} energy crystals.",
                    id: "game1_act2_feedback1_{1}"
                },
                {
                    msg: "We don't have the right number of energy crystals, we need {0}.",
                    id: "game1_act2_feedback2_{0}"
                },
                {
                    msg: "We have the wrong number of crystals, let's have {1}",
                    id: "game1_act2_feedback3_{1}"
                },
            ]
        },
        3: {
            instructions: [
                {
                    msg: "I need to <laugh,nb> show my spaceship where to go! <smile,nb> Will you help me navigate? <speak,nb> Choose the galaxy that has {1} stars.",
                    id: "game1_act3_inst0_{1}"
                },
                {
                    msg: "Choose the galaxy that has {1} stars. <laugh,nb>",
                    id: "game1_act3_inst{1}"
                },
            ],
            feedback: [
                {
                    msg: "Whoops! That's not the right galaxy. Try again.",
                    id: "game1_act3_feedback1"
                },
                {
                    msg: "Uh oh. I don't think that galaxy has {1} stars in it. Try again.",
                    id: "game1_act3_feedback2_{1}"
                },
                {
                    msg: "I don't think that's the correct galaxy. How about we try again!",
                    id: "game1_act3_feedback3"
                },

                {
                    msg: "That doesn't look right! Choose the galaxy with {1} stars in it.",
                    id: "game1_act3_feedback4_{1}"
                }
            ]
        },
        4: {
            instructions: [
                {
                    msg: "I have to <laugh,nb> show my spaceship how to go home <calm,nb>. Will you help me navigate? Choose the planet with the number {0}.",
                    id: "game1_act4_inst5_{0}"
                },
                {
                    msg: "Choose the planet <laugh,nb> with the number {0}.",
                    id: "game1_act4_inst1_{0}"
                },
            ],
            feedback: [
                {
                    msg: "That's not the correct planet. Let's try again.",
                    id: "game1_act4_feedback1"
                },
                {
                    msg: "That planet isn't the right one. We want the planet with number {0}. Try again.",
                    id: "game1_act4_feedback2_{0}"
                },
            ]
        },
        5: {
            instructions :[
                {
                    msg: "Meet <excited,nb> Ooki and Yana! They're my space <speak,nb> pets! They eat <bounce,nb> stardust! Will you <laugh,nb> help me feed them? Feed Ooki and Yana the same amount <laugh,nb> of stardust.",
                    id: "game1_act5_inst0"
                },
                {
                    msg: "<calm,nb> Will you help me feed <laugh,nb> Ooki and <speak,nb> Yana? Feed them the same amount of stardust.",
                    id: "game1_act5_inst1"
                },
                {
                    msg: "<calm,nb> Feed my space pets the same amount of stardust.",
                    id: "game1_act5_inst2"
                },
                {
                    msg: "<calm,nb> Help me feed Ooki and Yana <laugh,nb> the same amount of stardust.",
                    id: "game1_act5_inst3"
                },
            ],
            feedback: [
                {
                    msg: "Whoops! We gave Ooki and Yana different amounts of food. Try again!",
                    id: "game1_act5_feedback1"
                },
                {
                    msg: "Ooki and Yana don't have the same amount of stardust. Try again!",
                    id: "game1_act5_feedback2"
                },
                {
                    msg: "My space pets need to eat the same amount of stardust. Try again.",
                    id: "game1_act5_feedback3"
                },
            ]
        }
    },
    SPACESHIP_TIDYUP: {
        1: {
            instructions: [
                {
                    msg: "<laugh,nb> My space pets also need to go home! Let's make <concerned,nb> sure we get all of them onboard! Move them into the spaceship in order from <bounce,nb> 1 to {0}. Count them out with me!",
                    id: "game2_act1_inst0_{0}"
                },
                {
                    msg: "<speak,nb> Move my space pets <speak,nb> into the spaceship <laugh,nb> in order <speak,nb> from 1 to {0}.",
                    id: "game2_act1_inst{0}"
                },
            ],
            feedback: []
        },
        2: {
            instructions: [
                {
                    msg: "<speak,nb> Help me pack my moonrocks!",
                    id: "game2_act2_inst0_1"
                },
                {
                    msg: "<bounce,nb> Sort the moon rocks by color into different boxes!",
                    id: "game2_act2_inst0_2"
                },
            ],
            feedback: []
        },
        3: {
            instructions: [
                {
                    msg: "<bounce,nb> I've collected items from all over outer space. Will you help me <speak,nb> organize them?",
                    id: "game2_act3_inst0"
                },
                {
                    msg: "<calm,nb> Move each item into its proper box. <speak,nb> For example, put all the <laugh,nb> moon rocks into the box that is labeled moon rocks!",
                    id: "game2_act3_inst1"
                },
            ],
            feedback: []
        },
        4: {
            instructions: [
                {
                    msg: "<laugh,nb> Help me <speak,nb> match the pattern you see!",
                    id: "game2_act4_inst0"
                },
                {
                    msg: "<calm,nb> Copy the pattern by moving the same <speak,nb> moonrocks, <laugh,nb> energy crystals, <speak,nb> and stars into the box!",
                    id: "game2_act4_inst1"
                },
            ],
            feedback: []
        },
        5: {
            instructions: [
                {
                    msg: "<laugh,nb> Help me <speak,nb> finish the pattern you see!",
                    id: "game2_act5_inst0"
                },
                {
                    msg: "<speak,nb> Finish the pattern by moving the right <laugh,nb> moon rock, <speak,nb> energy crystal, or star into the box!",
                    id: "game2_act5_inst1"
                },
            ],
            feedback: []
        },
    },
    ALIEN_CODES: {
        1: {
            instructions: [
                {
                    msg: "My alien friends sent you a code. I need your help to translate it. Please describe the code you see.",
                    id: "A1Intro"
                }
            ],
            feedback: []
        },
        2: {
            instructions: [
                {
                    msg: "I’m having trouble finishing the alien code. Will you tell me what I need to complete the code?",
                    id: "A2Intro"
                }
            ],
            feedback: []
        },
        3: {
            instructions: [
                {
                    msg: "My space pets, Ooki and Yana, are not feeling well! We have to give them pills to make them feel better. What pills do I need?",
                    id: "A3Intro"
                }
            ],
            feedback: []
        },
        4: {
            instructions: [
                {
                    msg: "I need to get into my spaceship, but it is locked! Help me unlock the spaceship. What code will open the door?",
                    id: "A4Intro"
                }
            ],
            feedback: []
        },
        5: {
            instructions: [
                {
                    msg: "Will you help me decode the alien messages? Match each object to the words you see.",
                    id: "A5Intro"
                }
            ],
            feedback: []
        },
    },
};

exports.TUTORIALS = {
    GALACTIC_TRAVELER: [
        {
            msg: "For these games, I will be a robot space explorer! I'm visiting all the planets and the stars of the universe! But I'm stuck here on Earth and can't get back home. I need your help so I can go back and tell all my friends about space! Will you help me?",
            id: "gt_tutorial_1",
            duration: 10
        },
        {
            msg: "This is my spaceship. Want to see how it works?",
            id: "gt_tutorial_2",
            duration: 3
        },
        {
            msg: "This is where I control my spaceship. And this is where you can help me!",
            id: "gt_tutorial_3",
            duration: 4
        },
        {
            msg: "To make my spaceship do something, I move stuff into the box, then I press this green button when I'm done!",
            id: "gt_tutorial_4",
            duration: 5
        },
        {
            msg: "Now you try! Use your finger to drag the star into the box. When you are done, press the green finish button.",
            id: "gt_tutorial_5",
            duration: 5
        },
        {
            msg: "Sometimes I make mistakes and need to remove stuff from the box. To do that, I drag stuff out of the box.",
            id: "gt_tutorial_6",
            duration: 5
        },
        {
            msg: "Can you try to remove the star? Use your finger to drag the star out of the box. Then press the finish button.",
            id: "gt_tutorial_7",
            duration: 5
        },
        {
            msg: "Perfect. You're going to be a great partner!",
            id: "gt_tutorial_8",
            duration: 3
        }
    ],
    SPACESHIP_TIDYUP: [
        {
            msg: "For these games, we'll be space explorers! However, I'm having lots of trouble on my spaceship. I'm going to need your help to tidy up my ship so that I can make it back home. Do you want to help me?",
            id: "st_tutorial_0",
            duration: 10
        },
        {
            msg: "This is my spaceship. Want to see how it works?",
            id: "st_tutorial_1",
            duration: 5
        },
        {
            msg: "This is how I control my spaceship. This is where you can help me!",
            id: "st_tutorial_2",
            duration: 5
        },
        {
            msg: "To make my spaceship do something, I move stuff into the correct box. But I need to make sure the stuff matches the label on the box. When I'm done, I press this green button!",
            id: "st_tutorial_3",
            duration: 10
        },
        {
            msg: "Now you try! Use your finger to drag the star into the box labeled with a star. When you are done, press the green finish button.",
            id: "st_tutorial_4",
            duration: 10
        },
        {
            msg: "Sometimes I make mistakes and need to remove stuff from the box. To do that, I drag stuff out of the box.",
            id: "st_tutorial_5",
            duration: 5
        },
        {
            msg: "Can you try to remove the star? Use your finger to drag the star out of the box. Then press the finish button.",
            id: "st_tutorial_6",
            duration: 5
        },
        {
            msg: "You're doing great! I can't wait to play for real!",
            id: "st_tutorial_7",
            duration: 7
        }
    ],
    ALIEN_CODES: [
        {
            msg: "During these next games, I will be a robot space explorer! I've been traveling space for a long time now. I've received lots of messages on my journey, but I cannot understand them. I need your help understanding these messages. Are you ready to learn how you can help me?",
            id: "ac_tutorial_0",
            duration: 10
        },
        {
            msg: "This is my spaceship. Want to see how it works?",
            id: "ac_tutorial_1",
            duration: 7
        },
        {
            msg: "This is where I control my ship and where you can help me!",
            id: "ac_tutorial_2",
            duration: 7
        },
        {
            msg: "To play with my friends, I drag stuff into the empty box. Then I press the green button with an arrow when I'm done.",
            id: "ac_tutorial_3",
            duration: 10
        },
        {
            msg: "Are you ready to try? Use your finger to drag the space crystal into the empty box. Then, press the green arrow button to finish!",
            id: "ac_tutorial_4",
            duration: 7
        },
        {
            msg: "Sometimes I make mistakes and need to remove stuff from the box. To do that, I press the restart button on the left.",
            id: "ac_tutorial_5",
            duration: 6
        },
        {
            msg: "Can you try to remove the item? Use your finger to touch the reset button.",
            id: "ac_tutorial_6",
            duration: 5
        },
        {
            msg: "Well done! You're doing great!!",
            id: "ac_tutorial_7",
            duration: 5
        }
    ]
};

exports.generateSpeechScripts = function() {
    var results = "";

    for (let one of this.SUCCESS.concat(this.GENERAL_FEEDBACK)) {
        results += "[" + one.id + "]" + one.msg + "\n";
    }

    var strings = ["crystals", "crystal", "more", "fewer"];
    for (let key in this.GAMES) {
        for (let activity in this.GAMES[key]) {
            for (let item in this.GAMES[key][activity]) {
                var messages = this.GAMES[key][activity][item];
                for (let message of messages) {
                    for (var i = 0; i <= (message.id.indexOf("{0}") == -1 ? 0: 10); i++) {
                        for (var j = 0; j < (message.id.indexOf("{1}") == -1 ? 1: strings.length); j++) {
                            var string = strings[j];
                            var str = ("[" + message.id + "]" + message.msg + "\n");
                            if (str.length > 3)
                                results += str.format(i, string);
                        }
                    }
                }
            }
        }
    }
    return results;
}

// console.log(exports.generateSpeechScripts());

},{"./constants.js":2}]},{},[24])(24)
});