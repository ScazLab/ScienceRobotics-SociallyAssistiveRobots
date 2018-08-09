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
