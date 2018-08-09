/**
  * Space Ship Tidy Up Activity 1
  * -----------------------------
  *      Board Space Pets - Arrange space pets based on their value 
  *      The level determines the numbers on the space pets
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
    this.objects, this.spaceship;
    this.currentCount = 0;
    this.target;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity1.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity1.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/Atmosphere1.png');

    this.game.load.spritesheet('object1',CONST.ASSETS_PATH+'Items/Aliens/Alien1Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object2',CONST.ASSETS_PATH+'Items/Aliens/Alien2Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object3',CONST.ASSETS_PATH+'Items/Aliens/Alien3Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object4',CONST.ASSETS_PATH+'Items/Aliens/Alien4Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object5',CONST.ASSETS_PATH+'Items/Aliens/Alien5Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object6',CONST.ASSETS_PATH+'Items/Aliens/Alien6Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object7',CONST.ASSETS_PATH+'Items/Aliens/Alien7Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object8',CONST.ASSETS_PATH+'Items/Aliens/Alien8Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object9',CONST.ASSETS_PATH+'Items/Aliens/Alien9Box_sheet.png',610,610,2);
    this.game.load.spritesheet('object10',CONST.ASSETS_PATH+'Items/Aliens/Alien10Box_sheet.png',610,610,2);
    this.game.load.spritesheet('spaceship',CONST.ASSETS_PATH+'Items/Spaceship_sheet.png',1090,586,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity1.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    this.target = this.generateTarget();

    this.spaceship = this.game.add.sprite(0.55*this.game.width,0.5*this.game.height,'spaceship');
    this.spaceship.scale.setTo(0.7*this.sx);
    this.spaceship.y = 0.5*this.game.height - this.spaceship.height/2;
    this.spaceship.inputEnabled = true;
    this.spaceship.frame = 0;

    var arr = [1,2,3,4,5,6,7,8,9,10];
    arr = this.shuffle(arr);
    this.objects=this.game.add.group();

    for (var i=1;i<=this.target;i++){
        var name = "object"+arr[i-1];
        console.log("Object number "+i+" object name "+name);
        var object = this.objects.create(0.1*this.game.width+((i-1)%4)*230*this.sx,
                                         150*this.sy+Math.floor((i-1)/4)*230*this.sy,name);
        object.addChildAt(this.game.make.text(-object.width/2,
                                         -object.height/2,i,
                                         {fontSize:250*this.sx+'px',fill:'#fff'}),0);
    }

    var bounds = new Phaser.Rectangle(0,0,logo.width,logo.height);

    this.objects.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.scale.setTo(0.30*this.sx);
        object.originalPosition = object.position.clone();
        object.frame=0;
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,bounds,null);
        object.events.onDragStart.add(this.onDragStart.bind(this), this);
        object.events.onDragStop.add(this.onDragStop.bind(this), this);
        object.events.onDragUpdate.add(this.onDragUpdate.bind(this), this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.target);
};


Activity1.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
    console.log("outside");
}

Activity1.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;
}

Activity1.prototype.onDragStop = function(sprite,pointer){
    sprite.frame=0;
    this.spaceship.frame = 0;
    if (this.isInside(sprite) && this.isCorrect(sprite)) sprite.destroy();
     
    else if (this.isInside(sprite)) {
        var num = sprite.getChildAt(0).text;
        sprite.x = 0.1*this.game.width+((num-1)%4)*230*this.sx;
        sprite.y = 150*this.sy+Math.floor((num-1)/4)*230*this.sy;
        this.attempts++;

        this.publishRobotGameFeedback(this.target);
    }
    
    else if (!this.isInside(sprite)){
        this.objects.forEach(function(object){
            object.position.copyFrom(object.originalPosition);
        });
    }
        
 
    if (this.currentCount==this.target){
        this.objects.forEach(function(object){object.input.draggable = false;});
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
}

Activity1.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite)) this.spaceship.frame = 1;
    else if (!this.isInside(sprite)) this.spaceship.frame = 0;
}

Activity1.prototype.isInside = function(object){
    if (object.x>this.spaceship.x+0.01*this.game.width && 
        object.y>this.spaceship.y+0.01*this.game.height && 
        object.x<this.spaceship.x+this.spaceship.width-0.01*this.game.width && 
        object.y<this.spaceship.y+this.spaceship.height-0.01*this.game.height) return true;
    else return false;
}

Activity1.prototype.isCorrect = function(object){
    if (object.getChildAt(0).text==this.currentCount+1){this.currentCount++; return true;}
    else return false;
};

Activity1.prototype.generateTarget = function() {
    switch(this.level) {
        case 1:
            return this.game.rnd.integerInRange(2, 4);
        case 2:
            return this.game.rnd.integerInRange(5, 7);
        case 3:
            return this.game.rnd.integerInRange(8, 10);
        default: 
            throw CONST.UNEXPECTED_LEVEL;
    }
};


module.exports = Activity1;
