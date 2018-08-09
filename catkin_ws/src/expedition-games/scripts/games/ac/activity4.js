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
