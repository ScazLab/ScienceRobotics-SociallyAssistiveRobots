/**
  * Galactic Traveler Activity 3
  * -----------------------------
  *      Charge spaceship with [1-3, 4-6, 7-9] energy crystals. The level determines the number of crystals needed to
  *      charge the spaceship.
  *
  */

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const BOX_MIN_STARS = 1;
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
    this.objects;
    this.box1, this.box2;
    this.count1, this.count2;
    this.promptText, this.question, this.lessmore;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.offset, this.smallBoxMax, this.largeBoxMax;

}
Activity3.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity3.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH +'Items/ControlPanelcrop.png');
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH +'Items/Stars/Star1_sheet.png',410,410,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH +'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH +'Items/Stars/Star3_sheet.png',410,410,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH +'Items/Stars/Star4_sheet.png',410,410,2);
    this.game.load.image('galaxy1', CONST.ASSETS_PATH +'Items/Galaxies/Galaxy1.png');
    this.game.load.image('galaxy2', CONST.ASSETS_PATH +'Items/Galaxies/Galaxy2.png');
    this.game.load.spritesheet('box', CONST.ASSETS_PATH +'Items/Box_sheet.png',881,730,2);

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity3.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    var controlPanel = this.game.add.sprite(0,0,'controlPanel');
    controlPanel.scale.setTo(0.95*this.sx,0.95*this.sy);
    controlPanel.x = this.game.world.centerX - controlPanel.width/2;
    controlPanel.y = this.game.world.centerY - controlPanel.height/2;

    this.box1 = this.game.add.sprite(0,0, 'box');
    this.box1.scale.setTo(0.7*this.sx,0.8*this.sy);
    this.box1.x = controlPanel.x + 200*this.sx ;
    this.box1.y = controlPanel.y + controlPanel.height/2 - this.box1.height/2;
    this.box1.inputEnabled=true;
    this.box1.frame = 0;
    this.box1.events.onInputDown.add(this.onInputDown.bind(this), this);
    this.box1.events.onInputUp.add(onInputUp,this);

    this.box2 = this.game.add.sprite(0,0,'box');
    this.box2.scale.setTo(0.7*this.sx,0.8*this.sy);
    this.box2.x = controlPanel.x + controlPanel.width - 200*this.sx - this.box2.width;
    this.box2.y = controlPanel.y + controlPanel.height/2 - this.box2.height/2;
    this.box2.inputEnabled=true;
    this.box2.frame = 0;
    this.box2.events.onInputDown.add(this.onInputDown.bind(this), this);
    this.box2.events.onInputUp.add(onInputUp,this);

    var galaxy1 = this.game.add.sprite(this.box1.x+this.box1.width/2,this.box1.y+this.box1.height/2,'galaxy1');
    galaxy1.anchor.setTo(0.5,0.5);
    galaxy1.scale.setTo(0.7*this.sx);

    var galaxy2 = this.game.add.sprite(this.box2.x+this.box2.width/2,this.box2.y+this.box2.height/2,'galaxy2');
    galaxy2.anchor.setTo(0.5,0.5);
    galaxy2.scale.setTo(0.7*this.sx);

    //console.log('cp width and height are'+controlPanel.width+'x'+controlPanel.height);

    // get a 1 or 2 to select which is bigger, left or right box
    this.lessmore = this.game.rnd.integerInRange(1, 2); //1 is for less, 2 is for more

    this.offset = this.generateBoxNumberOffset();
    console.log("off: " + this.offset);
    this.largeBoxMax = this.generateLargeBoxMax();
    this.smallBoxMax = this.generateSmallBoxMax(this.largeBoxMax, this.offset);

    this.count1 = this.game.rnd.integerInRange(this.smallBoxMax - this.offset, this.smallBoxMax);
    this.count2 = this.count1 + this.offset;
    // do {
    //     this.count1 = this.game.rnd.integerInRange(this.smallBoxMax - this.offset, this.smallBoxMax);
    //     this.count2 = this.count1 - this.offset;
    // } while (this.count1 == this.count2);

    var arr = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16];
    arr = this.shuffle(arr);
    this.objects=this.game.add.group();
    for (var i = 1; i <= this.count1; i++){
        var objectname = 'object'+this.game.rnd.integerInRange(1,4);
        this.objects.create(this.box1.x+110*this.sx+((arr[i]-1)%4)*120*this.sx,this.box1.y+100*this.sy+Math.floor((arr[i]-1)/4)*120*this.sy, objectname);
    }
    arr = this.shuffle(arr);
    for (i = 1; i <= this.count2; i++){
        var objectname = 'object'+this.game.rnd.integerInRange(1,4);
        this.objects.create(this.box2.x+110*this.sx+((arr[i]-1)%4)*120*this.sx,this.box2.y+100*this.sy+Math.floor((arr[i]-1)/4)*120*this.sy, objectname);
    }
    this.objects.forEach(function(object){
        object.scale.setTo(0.25*this.sx);
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
    }, this);

    var selection;
    if (this.lessmore == 1) {selection = 'fewer';}
    else if (this.lessmore == 2) {selection = 'more';}
    this.promptText = this.game.add.text(this.game.width/2,0.03*this.game.height,'',{fontSize:'24px',fill:'#fff'});
    this.question = 'Please choose the box with ' + selection + ' stars in it';
    this.promptText.setText(this.question);
    this.promptText.x = this.game.width/2-this.promptText.width/2;

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.lessmore == 1 ? 2 : 1, selection);
};


/**
 * Handler for when button is pressed
 */
Activity3.prototype.onInputDown = function(sprite) {
    sprite.frame = 1;
    if (this.activityCompleted(sprite)) {
        this.stat = true;
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
    else{
        this.stat = false;
        this.attempts++;

        // provide specific feedback on the mistake
        this.publishRobotGameFeedback(this.lessmore == 1 ? 2 : 1, this.lessmore == 1 ? "fewer" : "more");
    }
};

Activity3.prototype.activityCompleted = function (sprite) {
    return (this.count1>this.count2 && this.lessmore == 2 && sprite == this.box1) ||
           (this.count1>this.count2 && this.lessmore == 1 && sprite == this.box2) ||
           (this.count1<this.count2 && this.lessmore == 2 && sprite == this.box2) ||
           (this.count1<this.count2 && this.lessmore == 1 && sprite == this.box1);
}

Activity3.prototype.generateBoxNumberOffset = function() {
    switch (this.level) {
        case 1:
            return this.game.rnd.integerInRange(4, 6);
        case 2:
            return this.game.rnd.integerInRange(2, 4);
        case 3:
            return this.game.rnd.integerInRange(1, 3);
        default:
            throw CONST.UNEXPECTED_LEVEL;
    }
}

Activity3.prototype.generateLargeBoxMax = function() {
    switch (this.level) {
        case 1:
            return this.game.rnd.integerInRange(12, 16);
        case 2:
            return this.game.rnd.integerInRange(10, 15);
        case 3:
            return this.game.rnd.integerInRange(8, 12);
        default:
            throw CONST.UNEXPECTED_LEVEL;
    }
}

Activity3.prototype.generateSmallBoxMax = function(largeBoxVal, offset) {
    return largeBoxVal - offset;
}
    

function onInputUp(sprite){

};


module.exports = Activity3;
