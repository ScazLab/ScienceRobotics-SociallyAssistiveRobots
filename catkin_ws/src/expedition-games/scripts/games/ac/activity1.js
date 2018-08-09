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
