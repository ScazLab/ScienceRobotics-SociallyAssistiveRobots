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
