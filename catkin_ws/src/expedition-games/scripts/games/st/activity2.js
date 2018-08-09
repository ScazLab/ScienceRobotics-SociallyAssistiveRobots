/**
  * Space Ship Tidy Up Activity 2
  * -----------------------------
  *      Organize rocks by color
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 2;

const ROCK_AMOUNT = 9;
const COLUMNS_PANEL = 3;
const SAME_COLOR_AMOUNT = 3;
/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity2(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.logoBackground;
    this.onPanel;
    this.onbox1;
    this.onbox2;
    this.onbox3;
    this.box1;
    this.box2;
    this.box3;

    this.objectname = new Array();
            
    // create a color class-object array 
    // Each color has its properties, rockString and boxColor(String) that have the same color
    //shuffle the array later
    this.color = [
        { rockString: "object3", boxColor: "boxOrange"},
        { rockString: "object4", boxColor: "boxPink"},
        { rockString: "object5", boxColor: "boxBlue"}
    ]
    
    this.shuffle(this.color);
    
    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }
    
    // keep track of the color count using array 
    // index 0 ==> 1st color;
    // index 1 ==> 2nd color;
    // index 2 ==> 3nd color;
                        
    this.countBox1Color = [0,0,0];
    
    this.countBox2Color = [0,0,0];
    
    this.countBox3Color = [0,0,0];
    
    
    this.numColor1 =0;
    this.numColor2 =0;
    this.numColor3 =0;

    this.controlPanel, this.button;

    this.sx = window.innerWidth / gameWidth;
    this.sy = window.innerHeight / gameHeight;
}
Activity2.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity2.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');

    this.game.load.spritesheet('object3',CONST.ASSETS_PATH+'Items/Rocks/Moonrock3_sheet.png',660,360,2);
    this.game.load.spritesheet('object4',CONST.ASSETS_PATH+'Items/Rocks/Moonrock4_sheet.png',510,410,2);
    this.game.load.spritesheet('object5',CONST.ASSETS_PATH+'Items/Rocks/Moonrock5_sheet.png',660,360,2);
    this.game.load.spritesheet('button',CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box',CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    this.game.load.spritesheet('boxBlue',CONST.ASSETS_PATH+'Items/colorBox/BoxBlue_sheetnew.png',1510,499,2);
    this.game.load.spritesheet('boxPink',CONST.ASSETS_PATH+'Items/colorBox/BoxPink_sheetnew.png',1510,499,2);
    this.game.load.spritesheet('boxOrange',CONST.ASSETS_PATH+'Items/colorBox/BoxOrange_sheetnew.png',1510,499,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets and event handlers for sprites
 */
Activity2.prototype.create  = function() {
    var logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+this.game.width);
    console.log('Game height is'+this.game.height);

    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;
    
    this.button = this.game.add.sprite(0,0,'button');
    this.button.scale.setTo(0.3*this.sx);
    this.button.x = this.controlPanel.x+this.controlPanel.width/2-this.button.width/2;
    this.button.y = this.controlPanel.y+this.controlPanel.height-this.button.height-0.05*this.game.height;
    this.button.frame=0;
    this.button.inputEnabled=true;
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);
    
    
    this.box1 = this.game.add.sprite(0,0, this.color[0].boxColor);
    // long box is used instead of small box 
    this.box1.scale.setTo(0.45*this.sx,0.2*this.sy);
    //box1.anchor.setTo(0.5,0.5);
    this.box1.x = this.controlPanel.x + this.controlPanel.width  - this.box1.width - 140*this.sx;
    this.box1.y = this.controlPanel.y + this.controlPanel.height/3 - this.box1.height/3 - this.button.height - 50 *this.sx;
    this.box1.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box1
    this.box1.CoordX = new Array();
    this.box1.CoordY = new Array();
    
    this.box2 = this.game.add.sprite(0,0, this.color[1].boxColor);
    //box2.anchor.setTo(0.5,0.5);
    this.box2.scale.setTo(0.45*this.sx,0.2*this.sy);
    this.box2.x = this.controlPanel.x + this.controlPanel.width  - this.box2.width - 140*this.sx;
    this.box2.y = this.controlPanel.y + this.box1.y + 50*this.sx;
    this.box2.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box2
    this.box2.CoordX = new Array();
    this.box2.CoordY = new Array();
    
    this.box3 = this.game.add.sprite(0,0, this.color[2].boxColor);
    //box3.anchor.setTo(0.5,0.5);
    this.box3.scale.setTo(0.45*this.sx,0.2*this.sy);
    this.box3.x = this.controlPanel.x + this.controlPanel.width  - this.box3.width - 140*this.sx;
    this.box3.y = this.controlPanel.y + this.box2.y+ 50*this.sx;
    this.box3.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box3
    this.box3.CoordX = new Array();
    this.box3.CoordY = new Array();
    
    //set up the rock image at the upper left corner
    var upperLHRock1= this.game.add.sprite(this.box1.x, this.box1.y, this.color[0].rockString);
    upperLHRock1.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHRock2 = this.game.add.sprite(this.box2.x, this.box2.y, this.color[1].rockString);
    upperLHRock2.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHRock3 = this.game.add.sprite(this.box3.x, this.box3.y, this.color[2].rockString);
    upperLHRock3.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    
    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    
    
    //set a string array called objectname and shuffle them
    //it is used to generate/ create rock on panel
    for (var i=0; i< SAME_COLOR_AMOUNT ; i++){
        this.objectname[i] = "object3";
        this.objectname[i+3] = "object4";
        this.objectname[i+6] = "object5";
        
    }
    
    this.shuffle(this.objectname);
    
    //create the rock on panel
    for (var i=0;i<ROCK_AMOUNT;i++){
        //objectname = "object" + game.rnd.integerInRange(3,5);
        this.panel.CoordX[i] = this.controlPanel.x + 220*this.sx +(i%COLUMNS_PANEL) * 180 *this.sx;
        this.panel.CoordY[i] = this.controlPanel.y + this.controlPanel.height/2 - 120*this.sy + Math.floor(i/COLUMNS_PANEL) * 100*this.sy;
        
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }
    
    this.boxCoordinates(this.box1.CoordX, this.box1.CoordY,this.box1);
    this.boxCoordinates(this.box2.CoordX, this.box2.CoordY,this.box2);
    this.boxCoordinates(this.box3.CoordX, this.box3.CoordY,this.box3);
    
    for (var i=0; i<this.onPanel.total ; i++){
    
        switch (this.onPanel.getChildAt(i).key){
            
            case this.color[0].rockString: // let's say 'object3':
                this.numColor1++;
                break;
            case this.color[1].rockString:
                this.numColor2++;
                break;
            case this.color[2].rockString:
                this.numColor3++;
                break;
            
        }
    }

    
    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        //object.originalPosition = object.position.clone();
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,this.controlPanel);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.ROCK_AMOUNT);
};

Activity2.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
}

Activity2.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkColor();
}

Activity2.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
    this.resetColorCount();
}

Activity2.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
}

Activity2.prototype.onDragStop = function(sprite, pointer){
    this.box1.frame = 0;
    this.box2.frame = 0;
    this.box3.frame = 0;
    sprite.frame = 0;
    
    
    if(this.onBox1.total < 4 && this.isInside(sprite,this.box1) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox3.removeChild(sprite))){
        this.onBox1.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box1) && this.onBox1.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    if(this.onBox2.total < 4 && this.isInside(sprite,this.box2)&& (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox3.removeChild(sprite))){    
        this.onBox2.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box2) && this.onBox2.removeChild(sprite))
    {
        this.onPanel.add(sprite);
    }
    
    if(this.onBox3.total < 4 && this.isInside(sprite,this.box3) && (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox2.removeChild(sprite))){
        this.onBox3.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box3) && this.onBox3.removeChild(sprite))
    {
        this.onPanel.add(sprite);
    }
    
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
        this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox1.total; i++)
    {
        this.onBox1.getChildAt(i).x = this.box1.CoordX[i];
        this.onBox1.getChildAt(i).y = this.box1.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox2.total; i++)
    {
        this.onBox2.getChildAt(i).x = this.box2.CoordX[i];
        this.onBox2.getChildAt(i).y = this.box2.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox3.total; i++)
    {
        this.onBox3.getChildAt(i).x = this.box3.CoordX[i];
        this.onBox3.getChildAt(i).y = this.box3.CoordY[i];
    }

    
}

Activity2.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite,this.box1)){
        this.box1.frame = 1;
    }
    else if (!this.isInside(sprite,this.box1)){
        this.box1.frame = 0;
    }
    
    if (this.isInside(sprite,this.box2)){
        this.box2.frame = 1;
    }
    
    else if (!this.isInside(sprite,this.box2)){
        this.box2.frame = 0;
    }
    if (this.isInside(sprite,this.box3)){
        this.box3.frame = 1;
    }
    else if (!this.isInside(sprite,this.box3)){
        this.box3.frame = 0;
    }
                
}

Activity2.prototype.isInside = function(object,box){
    if (object.x > box.x+ 0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity2.prototype.checkColor = function(){
    
    //count the Number of the Y/P/LB color rocks in each box
    for(var i = 0 ; i <this.onBox1.total ; i++){
        
        if(this.onBox1.getChildAt(i).key == this.color[0].rockString){
            this.countBox1Color[0]++;
        }
        else if (this.onBox1.getChildAt(i).key == this.color[1].rockString){
            this.countBox1Color[1]++;
        
        }
        else if (this.onBox1.getChildAt(i).key == this.color[2].rockString){
            this.countBox1Color[2]++;
        }
    }
    
    
    for(var i=0; i<this.onBox2.total; i++){
        
        if(this.onBox2.getChildAt(i).key == this.color[0].rockString){
            this.countBox2Color[0]++;
        }
        else if (this.onBox2.getChildAt(i).key == this.color[1].rockString){
            this.countBox2Color[1]++;
        
        }
        else if (this.onBox2.getChildAt(i).key == this.color[2].rockString){
            this.countBox2Color[2]++;
        }
    }
    
    for(var i=0; i<this.onBox3.total ; i++){
        
        if(this.onBox3.getChildAt(i).key == this.color[0].rockString ){
            this.countBox3Color[0]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.color[1].rockString){
            this.countBox3Color[1]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.color[2].rockString){
            this.countBox3Color[2]++;
        }
    }
    
    
    if  (   
            (this.numColor1 == this.countBox1Color[0] ) && 
            (this.numColor2 == this.countBox2Color[1] ) &&
            (this.numColor3 == this.countBox3Color[2] ) 
        )
        {
            this.publishRobotSuccessMessage();
            this.gameComplete();
            return true;
        }
    else{
        this.attempts++;
        this.publishRobotGameFeedback();
    }
            
}

Activity2.prototype.resetColorCount = function(){
    for(var i =0; i< this.countBox1Color.length; i++)
    {
        this.countBox1Color[i] =0;
    }
    for(var i =0; i< this.countBox2Color.length; i++)
    {
        this.countBox2Color[i] =0;
    }
    for(var i =0; i< this.countBox3Color.length; i++)
    {
        this.countBox3Color[i] =0;
    }
}

Activity2.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< ROCK_AMOUNT; i++){
        boxArrayX[i] = box.x+ 130 *this.sx+(i%4)* 150 *this.sx;
        boxArrayY[i] = box.y+ box.height/2 + Math.floor(i/4)* 50 *this.sy;
    }
}


module.exports = Activity2;
