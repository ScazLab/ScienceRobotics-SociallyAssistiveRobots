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
