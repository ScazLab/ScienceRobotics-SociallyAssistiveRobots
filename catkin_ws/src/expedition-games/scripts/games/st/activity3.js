/**
  * Space Ship Tidy Up Activity 3
  * -----------------------------
  * game6-1.html
  * Organize by item
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 3;

const OBJECT_AMOUNT = 9;
const COLUMNS_PANEL = 3;
const SAME_OBJECT_AMOUNT = 3;
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
    this.onPanel;
    this.onBox1;
    this.onBox2;
    this.onBox3;
    
    this.box1;
    this.box2;
    this.box3;
    
    
    this.target;
    
    this.objectname = new Array();
    //string array with different type of item (in term of filename : 'object1')
    this.itemType = ['object1','object2', 'object3'];
    this.shuffle(this.itemType);
    
    
    this.box1CoordX = new Array();
    this.box1CoordY = new Array();
    this.box2CoordX = new Array();
    this.box2CoordY = new Array();
    this.box3CoordX = new Array();
    this.box3CoordY = new Array();
    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }
    
    // keep track of the item count using array for each box 
    // index 0 ==> 1st item;
    // index 1 ==> 2nd item;
    this.countBox1ItemType = [0,0,0];
    
    this.countBox2ItemType = [0,0,0];
    
    this.countBox3ItemType= [0,0,0];
    
    
    this.num_type1 =0;
    this.num_type2 =0;
    this.num_type3 =0;

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity3.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity3.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');
                        
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Rocks/Moonrock2_sheet.png',460,360,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Crystals/Crystal3_sheet.png',410,460,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

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
    controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    controlPanel.x = this.game.world.centerX - controlPanel.width/2;
    controlPanel.y = this.game.world.centerY - controlPanel.height/2;
    
    var button = this.game.add.sprite(0,0,'button');
    button.scale.setTo(0.3*this.sx);
    button.x = controlPanel.x+controlPanel.width/2-button.width/2;
    button.y = controlPanel.y+controlPanel.height-button.height-0.05*this.game.height;
    button.frame=0;
    button.inputEnabled=true;
    button.events.onInputDown.add(this.onButtonPress,this);
    button.events.onInputUp.add(this.onButtonRelease,this);
    
    this.box1 = this.game.add.sprite(0,0, 'box');
    this.box1.scale.setTo(0.8*this.sx,0.15*this.sy);
    this.box1.x = controlPanel.x + controlPanel.width  - this.box1.width - 140*this.sx;
    this.box1.y = controlPanel.y + controlPanel.height/3 - this.box1.height/3 - button.height - 50 *this.sx;
    this.box1.frame = 0;
    //Custom properties, CoordX and CoordY, added to object box1
    this.box1.CoordX = new Array();
    this.box1.CoordY = new Array();
    
    console.log('Box1 height is '+this.box1.height);
    console.log('Box1 width is '+this.box1.width);
    
    this.box2 = this.game.add.sprite(0,0, 'box');
    this.box2.scale.setTo(0.8*this.sx,0.15*this.sy);
    this.box2.x = controlPanel.x + controlPanel.width  - this.box2.width - 140*this.sx;
    this.box2.y = controlPanel.y + this.box1.y + 50*this.sx;
    this.box2.frame = 0;
    //Custom properties, CoordX and CoordY, added to object box2
    this.box2.CoordX = new Array();
    this.box2.CoordY = new Array();
    console.log('Box2 height is '+this.box2.height);
    console.log('Box2 width is '+this.box2.width);
    
    this.box3 = this.game.add.sprite(0,0, 'box');
    this.box3.scale.setTo(0.8*this.sx,0.15*this.sy);
    this.box3.x = controlPanel.x + controlPanel.width  - this.box3.width - 140*this.sx;
    this.box3.y = controlPanel.y + this.box2.y+ 50*this.sx;
    this.box3.frame = 0;
    
    //Custom properties, CoordX and CoordY, added to object box3
    this.box3.CoordX = new Array();
    this.box3.CoordY = new Array();
    console.log('Box2 height is '+this.box3.height);
    console.log('Box2 width is '+this.box3.width);
    

    var upperLHItem1 = this.game.add.sprite(this.box1.x, this.box1.y, this.itemType[0]);
    upperLHItem1.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHItem2 = this.game.add.sprite(this.box2.x, this.box2.y, this.itemType[1]);
    upperLHItem2.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    var upperLHItem3 = this.game.add.sprite(this.box3.x, this.box3.y, this.itemType[2]);
    upperLHItem3.scale.setTo(0.1*this.sx,0.1*this.sy);
    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    
    
    // set up all the item/object on the panel
    for (var i=0; i< SAME_OBJECT_AMOUNT ; i++){
        this.objectname[i] = "object1";
        this.objectname[i+3] = "object2";
        this.objectname[i+6] = "object3";
    }

    this.shuffle(this.objectname);
    
    //create all the object on the panel
    for (var i=0;i<OBJECT_AMOUNT;i++){
        //objectname = "object" + game.rnd.integerInRange(3,5);
        this.panel.CoordX[i] = controlPanel.x + 220*this.sx +(i%COLUMNS_PANEL) * 180 *this.sx;
        this.panel.CoordY[i] = controlPanel.y + controlPanel.height/2 - 120*this.sy + Math.floor(i/COLUMNS_PANEL) * 100*this.sy;
        
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }
    
    //set the position in each box
    this.boxCoordinates(this.box1.CoordX, this.box1.CoordY, this.box1);
    this.boxCoordinates(this.box2.CoordX, this.box2.CoordY, this.box2);
    this.boxCoordinates(this.box3.CoordX, this.box3.CoordY, this.box3);
    
    // assign the rock/crystal/star to an array type[0/1/2]
    for (var i=0; i<this.onPanel.total ; i++){
    
        switch (this.onPanel.getChildAt(i).key){
            
            case this.itemType[0]:// let say 'object1':
                this.num_type1++;
                break;
            case this.itemType[1]:
                this.num_type2++;
                break;
            case this.itemType[2]:
                this.num_type3++;
                break;
            
        }
    }

    
    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        object.originalPosition = object.position.clone();
        //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,controlPanel);
        //game.add.tween(object).to( { y: 400 }, 3000, Phaser.Easing.Cubic.InOut, true, 0, Number.MAX_VALUE, true);
        //object.input.enableSnap(130 *sx,150 *sy,false,true);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro();
};

Activity3.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
}


Activity3.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkItemType();
}

Activity3.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
    this.resetItemCount();
}

Activity3.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
    

}

Activity3.prototype.onDragStop = function(sprite, pointer){
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
    
    if(this.onBox2.total < 4 && this.isInside(sprite,this.box2 )&& (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox3.removeChild(sprite))){
        this.onBox2.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box2) && this.onBox2.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    
    if(this.onBox3.total < 4 && this.isInside(sprite,this.box3) && (this.onPanel.removeChild(sprite) || this.onBox1.removeChild(sprite) || this.onBox2.removeChild(sprite))){
        this.onBox3.add(sprite);                     
    }
    else if (!this.isInside(sprite,this.box3) && this.onBox3.removeChild(sprite)){
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

Activity3.prototype.onDragUpdate = function (sprite,pointer){
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


Activity3.prototype.isInside = function(object,box){
    if (object.x > box.x+ 0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity3.prototype.checkItemType = function(){
    //count the Number of the rock/crystal/star  in each box
    for(var i = 0 ; i <this.onBox1.total ; i++){
        
        if(this.onBox1.getChildAt(i).key == this.itemType[0]){
            this.countBox1ItemType[0]++;
        }
        else if (this.onBox1.getChildAt(i).key == this.itemType[1]){
            this.countBox1ItemType[1]++;
        
        }
        else if (this.onBox1.getChildAt(i).key == this.itemType[2]){
            this.countBox1ItemType[2]++;
        }

    }
    
    for(var i=0; i<this.onBox2.total; i++){
        
        if(this.onBox2.getChildAt(i).key == this.itemType[0]){
            this.countBox2ItemType[0]++;
        }
        else if (this.onBox2.getChildAt(i).key == this.itemType[1]){
            this.countBox2ItemType[1]++;
        }
        else if (this.onBox2.getChildAt(i).key == this.itemType[2]){
            this.countBox2ItemType[2]++;
        }
    }
    
    for(var i=0; i<this.onBox3.total ; i++){
        
        if(this.onBox3.getChildAt(i).key == this.itemType[0] ){
            this.countBox3ItemType[0]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.itemType[1]){
            this.countBox3ItemType[1]++;
        }
        else if (this.onBox3.getChildAt(i).key == this.itemType[2]){
            this.countBox3ItemType[2]++;
        }
    }
    
    if  (   (this.num_type1 == this.countBox1ItemType[0] ) && 
            (this.num_type2 == this.countBox2ItemType[1] ) &&
            (this.num_type3 == this.countBox3ItemType[2] ) 
        )
        {
            this.publishRobotSuccessMessage();
            this.gameComplete();
            return true;
        }
    else{
        attempts++;
        this.publishRobotGameFeedback();
    }
}

Activity3.prototype.resetItemCount = function(){
    for(var i =0; i< this.countBox1ItemType.length; i++)
    {
        this.countBox1ItemType[i] =0;
    }
    for(var i =0; i< this.countBox2ItemType.length; i++)
    {
        this.countBox2ItemType[i] =0;
    }
    for(var i =0; i< this.countBox3ItemType.length; i++)
    {
        this.countBox3ItemType[i] =0;
    }
}

Activity3.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< OBJECT_AMOUNT; i++){
        boxArrayX[i] = box.x + 130 *this.sx + (i%4)* 150 * this.sx;
        boxArrayY[i] = box.y + box.height/2 + Math.floor(i/4) * 50 * this.sy;
    }
}

module.exports = Activity3;
