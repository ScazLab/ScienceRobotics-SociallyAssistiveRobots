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
