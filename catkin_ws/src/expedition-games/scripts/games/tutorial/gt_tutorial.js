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