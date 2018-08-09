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
