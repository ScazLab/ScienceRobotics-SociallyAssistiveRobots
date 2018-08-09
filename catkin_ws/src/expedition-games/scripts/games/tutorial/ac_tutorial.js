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
