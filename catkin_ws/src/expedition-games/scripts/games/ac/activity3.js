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
