/**
 * Galactic Traveler Activity 2
 * -----------------------------
 *      Charge spaceship with [1-3, 4-6, 7-9] energy crystals. The level determines the number of crystals needed to
 *      charge the spaceship.
 *
 */

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 5;

/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity5(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920,
        gameHeight = 1080;

    // Declare object properties
    this.objects;
    this.boxGroupL;
    this.boxGroupR;
    
    this.box1CoordX = new Array();
    this.box1CoordY = new Array();
    this.box2CoordX = new Array();
    this.box2CoordY = new Array();

    this.target;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.logo;
}

Activity5.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity5.prototype.preload = function() {
    // Load static image assets
    this.game.load.image('logo', '../../assets/Background/Atmosphere1.png'); // logo means background image

    // Load sprites
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Stars/Star1_sheet.png',410,410,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Stars/Star3_sheet.png',410,410,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Stars/Star4_sheet.png',410,410,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('yana', CONST.ASSETS_PATH+'Items/Aliens/Yana_sheet.png',730,881,2);
    this.game.load.spritesheet('yuki', CONST.ASSETS_PATH+'Items/Aliens/Yuki_sheet.png',730,881,2);

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets
 */
Activity5.prototype.create = function() {
    // background image is displayed
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    //  Moves the image anchor to the middle, so it centers inside the game properly
    this.logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+ this.game.width);
    console.log('Game height is'+ this.game.height);

    this.target = this.game.math.snapToCeil(2 * this.generateTarget(), 2,0);
    
    this.button = this.game.add.sprite(0,0,'button');
    this.button.scale.setTo(0.4*this.sx);
    this.button.x = this.game.world.centerX-this.button.width/2;
    this.button.y = this.game.world.height*0.9-this.button.height/2;
    this.button.frame=0;
    this.button.inputEnabled=true;
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);

    this.box1 = this.game.add.sprite(0,0,'yuki');
    this.box1.scale.setTo(0.7*this.sx);
    this.box1.x = 0.05*this.game.width;
    this.box1.y = this.game.height/2-this.box1.height/2;
    this.box1.inputEnabled=true;
    this.box1.frame = 0;

    this.box2 = this.game.add.sprite(0,0,'yana');
    this.box2.scale.setTo(0.7*this.sx);
    this.box2.x = 0.95*this.game.width-this.box2.width;
    this.box2.y = this.game.height/2-this.box2.height/2;
    this.box2.inputEnabled=true;
    this.box2.frame = 0;
    
    this.objects = this.game.add.group();
    this.boxGroupL = this.game.add.group();
    this.boxGroupR = this.game.add.group();
    
    this.boxCoordinates(this.box1CoordX,this.box1CoordY, this.box1);
    this.boxCoordinates(this.box2CoordX,this.box2CoordY, this.box2);
    
    for (var i=1;i<=this.target;i++){
        var name = "object"+(i%4+1);
        console.log("Object number "+i+" object name "+name);
        this.objects.create(this.game.world.centerX + Math.pow(-1,i%2)*75,0.1*this.game.height+Math.floor((i-1)/2)*100,name);
    }

    var bounds = new Phaser.Rectangle(0,0,this.logo.width,this.logo.height);

    this.objects.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        //clone the original star position
        object.originalPosition = object.position.clone();
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,this.bounds,null);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        //object.input.enableSnap(100,100,false,true);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.target);
};

Activity5.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
    console.log("outside");
}

Activity5.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.stat = this.countStars();
    if (!this.stat) {
        this.attempts++;

        // publish specific fail message
        this.publishRobotGameFeedback();
    } else {
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
}

Activity5.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
}

Activity5.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;
}

Activity5.prototype.onDragStop = function(sprite,pointer){
    sprite.frame=0;
                
    if (this.isInside(sprite, this.box1))
    {
        this.boxGroupL.addChild(sprite);
    }
    else if (this.isInside(sprite, this.box2)  )
    {
        this.boxGroupR.addChild(sprite);
    }
    
    else if (!(this.isInside(sprite, this.box1) || this.isInside(sprite, this.box2)) )
    {
        //add to the object group if the star is dragged outside of box and the total objects is not equal to total number of stars
        if(this.objects.countLiving()!== this.target){
            this.objects.addChild(sprite);
        }
        //set it back to its original position
        sprite.position.copyFrom(sprite.originalPosition); 
    }
        
    //Rearrange all the position in box1 and box2
    for (var i = 0; i < this.boxGroupL.total; i++)
    {
        this.boxGroupL.getChildAt(i).x = this.box1CoordX[i];
        this.boxGroupL.getChildAt(i).y = this.box1CoordY[i];
    }
    for (var i = 0; i < this.boxGroupR.total; i++)
    {
        this.boxGroupR.getChildAt(i).x = this.box2CoordX[i];
        this.boxGroupR.getChildAt(i).y = this.box2CoordY[i];
    }
}

Activity5.prototype.isInside = function(object,box){
    if (object.x > box.x + 0.01*this.game.width && 
        object.y > box.y + 0.01*this.game.height && 
        object.x < box.x + box.width - 0.01*this.game.width && 
        object.y < box.y + box.height - 0.01*this.game.height) return true;
    else return false;
}

Activity5.prototype.countStars = function(){
    var count1 = this.boxGroupL.total;
    var count2 = this.boxGroupR.total;

    if (count1==count2 && count1==this.target/2.0){ 
        this.boxGroupL.forEach(function(object){object.input.draggable = false;});
        this.boxGroupR.forEach(function(object){object.input.draggable = false;});
        return true;
    }
        else return false;
}


Activity5.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< this.target; i++){
        
        boxArrayX[i] = box.x+100*this.sx+(i%3)*150*this.sx;
        boxArrayY[i] = box.y+ 70 *this.sy + Math.floor(i/3)*100*this.sy;
    }
}

Activity5.prototype.generateTarget = function() {
    switch (this.level) {
        case 1: return this.game.rnd.integerInRange(3, 4);
        case 2: return this.game.rnd.integerInRange(5, 6);
        case 3: return this.game.rnd.integerInRange(7, 8);
        default: throw CONST.UNEXPECTED_LEVEL;
    }
}

module.exports = Activity5;
