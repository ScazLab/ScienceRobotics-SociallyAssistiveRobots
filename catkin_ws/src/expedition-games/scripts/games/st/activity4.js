/**
  * Space Ship Tidy Up Activity 4
  * -----------------------------
  *      game4-1-2.html
  *      Copy the sequence
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 4;

const OBJECT_AMOUNT = 9;
const COLUMNS_PANEL = 3;
const SEQUENCE_AMOUNT =4;
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
    this.onPanel;
    this.onBox1;
    this.onBox2;

    this.box1;
    this.box2;
    this.box3;
    this.box4;
    this.box5;
    this.box6;
    this.box7;
    this.box8;
    
    //set a string array called objectname and shuffle them
    // tempObj is randomize
    this.tempObj1 = "object" + this.game.rnd.integerInRange(1,2);
    this.tempObj2 = "object" + this.game.rnd.integerInRange(3,4);
    this.tempObj3 = "object" + this.game.rnd.integerInRange(5,6);
        
    this.objectname  =  ['object1', 'object2', 'object3', 'object4' , 'object5', 'object6', this.tempObj1, this.tempObj2, this.tempObj3];
    this.shuffle(this.objectname);
    
    //set up a sequence array
    this.sequenceObj = new Array();
    

    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity4.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity4.prototype.preload = function() {
    this.game.load.image('logo', CONST.ASSETS_PATH + 'Background/ShipInterior.png');
    this.game.load.image('controlPanel', CONST.ASSETS_PATH+'Items/ControlPanelcrop.png');
                        
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Rocks/Moonrock1_sheet.png',610,400,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Rocks/Moonrock4_sheet.png',510,410,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Crystals/Crystal5_sheet.png',410,360,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Crystals/Crystal2_sheet.png',460,510,2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH+'Items/Stars/Star1_sheet.png',410,410,2);
    this.game.load.spritesheet('object6', CONST.ASSETS_PATH+'Items/Stars/Star2_sheet.png',410,410,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);
    this.game.load.spritesheet('bigBox', CONST.ASSETS_PATH+'Items/colorBox/BoxBlue_sheetnew.png',1510,510,2);
    this.game.load.spritesheet('test', CONST.ASSETS_PATH+'Items/BoxRotatedOver.png',720,871 ,2);
    
    this.game.scale.scaleMode = Phaser.ScaleManager.SHOW_ALL;
    this.game.scale.updateLayout();

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
    
    /*
    box1/3/5/7 are used to set the sequence
    box2/4/6/8 are used to create the empty box on the blue rectangle box
    */
    
    this.box1 = this.game.add.sprite(0,0, 'box');
    this.box1.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box1.x = controlPanel.x + controlPanel.width  - box1.width - 140*sx;
    this.box1.x = controlPanel.x + controlPanel.width/2 -60*this.sx;
    //box1.y = controlPanel.y + controlPanel.height/2 - box1.height/2 - button.height - 50 *sx;
    this.box1.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box1.height/2 - 50 *this.sx;
    this.box1.frame = 0;
    this.box1.visible = false;
    console.log('Box1 height is '+ this.box1.height);
    console.log('Box1 width is '+ this.box1.width);           

    var rec0 = this.game.add.sprite(0,0,'bigBox');
    rec0.scale.setTo(0.488*this.sx,0.3*this.sy);
    rec0.x = this.box1.x  -40*this.sx ;
    rec0.y = this.box1.y -15*this.sx;
    rec0.frame = 0;
    
    //second blue rectangle box is created first so that it doesnt cover the empty box2/4/6/8
    var rec1 = this.game.add.sprite(0,0,'bigBox');
    rec1.scale.setTo(0.488*this.sx,0.3*this.sy);
    rec1.x = controlPanel.x + controlPanel.width/2 -60*this.sx  - 40*this.sx;
    rec1.y = controlPanel.y + this.box1.y + 50*this.sx - 20*this.sx;
    rec1.frame = 0;
    
    this.box2 = this.game.add.sprite(0,0, 'box');
    this.box2.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box2.x = controlPanel.x + controlPanel.width  - box2.width - 140*sx;
    this.box2.x = controlPanel.x + controlPanel.width/2 -60*this.sx;
    this.box2.y = controlPanel.y + this.box1.y + 50*this.sx;
    this.box2.frame = 0;
    
    this.box3 = this.game.add.sprite(0,0, 'box');
    this.box3.scale.setTo(0.18*this.sx,0.15*this.sy);
    this.box3.x = controlPanel.x +controlPanel.width/2-60*this.sx + this.box1.width + 10*this.sx;
    //box3.x = controlPanel.x + controlPanel.width  - 2*box3.width - 160*sx;
    //box3.y = controlPanel.y + controlPanel.height/2 - box3.height/2 - button.height - 50 *sx;
    this.box3.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box3.height/2 - 50 *this.sx;
    this.box3.frame = 0;
    this.box3.visible=false;
    
    this.box4 = this.game.add.sprite(0,0, 'box');
    this.box4.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box4.x = controlPanel.x + controlPanel.width  - 2*box4.width - 160*sx;
    this.box4.x = controlPanel.x +controlPanel.width/2-60*this.sx + this.box1.width + 10*this.sx;
    this.box4.y = controlPanel.y + this.box3.y + 50*this.sx;
    this.box4.frame = 0;

    this.box5 = this.game.add.sprite(0,0, 'box');
    this.box5.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box5.x = controlPanel.x + controlPanel.width  - 3*box5.width - 180*sx;
    this.box5.x = controlPanel.x + controlPanel.width/2 -60*this.sx + this.box1.width + this.box3.width +2*10*this.sx;
    this.box5.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box5.height/2 - 50 *this.sx;
    this.box5.frame = 0;
    this.box5.visible = false;
    
    this.box6 = this.game.add.sprite(0,0, 'box');
    this.box6.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box6.x = controlPanel.x + controlPanel.width  - 3*box6.width - 180*sx;
    this.box6.x = controlPanel.x + controlPanel.width/2 -60*this.sx + this.box1.width + this.box3.width +2*10*this.sx;
    this.box6.y = controlPanel.y + this.box5.y + 50*this.sx;
    this.box6.frame = 0;
    
    this.box7 = this.game.add.sprite(0,0, 'box');
    this.box7.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box7.x = controlPanel.x + controlPanel.width  - 4*box7.width - 200*sx;
    this.box7.x = controlPanel.x + controlPanel.width/2-60*this.sx  + this.box1.width + this.box3.width + this.box5.width + 3*10*this.sx;
    this.box7.y = controlPanel.y + controlPanel.height  - button.height - controlPanel.height/2 - this.box5.height/2 - 50 *this.sx;
    this.box7.frame = 0;
    this.box7.visible = false;
    
    this.box8 = this.game.add.sprite(0,0, 'box');
    this.box8.scale.setTo(0.18*this.sx,0.15*this.sy);
    //box8.x = controlPanel.x + controlPanel.width  - 4*box8.width - 200*sx;
    this.box8.x = controlPanel.x + controlPanel.width/2-60*this.sx  + this.box1.width + this.box3.width + this.box5.width + 3*10*this.sx;
    this.box8.y = controlPanel.y + this.box7.y + 50*this.sx;
    this.box8.frame = 0;

    // create groups 
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    this.onBox4 = this.game.add.group();
    this.onBox5 = this.game.add.group();
    this.onBox6 = this.game.add.group();
    this.onBox7 = this.game.add.group();
    this.onBox8 = this.game.add.group();

    //generate each object on panel
    for (var i=0;i< OBJECT_AMOUNT;i++){
        this.panel.CoordX[i] = controlPanel.x + 220*this.sx +(i%COLUMNS_PANEL) * 180*this.sx;
        this.panel.CoordY[i] = controlPanel.y + controlPanel.height/2 - 120*this.sy + Math.floor(i/COLUMNS_PANEL) * 100*this.sy;
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }

    for (var i=0;i< OBJECT_AMOUNT;i++){
        this.sequenceObj[i] = this.objectname[i];
    }

    this.shuffle(this.sequenceObj);
    
    //create the sequence/pattern on box1/3/5/7
    this.onBox1.create(this.box1.x + this.box1.width/2, this.box1.y + this.box1.height/2 , this.sequenceObj[0]);
    this.onBox3.create(this.box3.x + this.box3.width/2, this.box3.y + this.box3.height/2 , this.sequenceObj[1]);
    this.onBox5.create(this.box5.x +this.box5.width/2, this.box5.y + this.box5.height/2 , this.sequenceObj[2]);
    this.onBox7.create(this.box7.x + this.box7.width/2, this.box7.y + this.box7.height/2 , this.sequenceObj[3]);
    
    
    this.onBox1.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    
    this.onBox3.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox5.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox7.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    

    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
        //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,controlPanel);
        //game.add.tween(object).to( { y: 400 }, 3000, Phaser.Easing.Cubic.InOut, true, 0, Number.MAX_VALUE, true);
        //object.input.enableSnap(120*sx,150*sy,false,true);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(OBJECT_AMOUNT);
};

Activity4.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
}

Activity4.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkSequence();
}

Activity4.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
}

Activity4.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
}
    
Activity4.prototype.onDragStop = function(sprite, pointer){
    //box1.frame = 0;
    this.box2.frame = 0;
    this.box4.frame = 0;
    this.box6.frame = 0;
    this.box8.frame = 0;
    sprite.frame = 0;
    
    if (this.onBox2.total < 1 && this.isInside(sprite,this.box2)   && (this.onPanel.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox6.removeChild(sprite) || this.onBox8.removeChild(sprite))){
        this.onBox2.add(sprite);
    }
    else if (this.onBox2.total <1 && !this.isInside(sprite,this.box2) && this.onBox2.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box2) && this.isInside(sprite,this.box4)){
        //do nothing
    }
    else if(this.onBox2.total == 1 && this.isInside(sprite, this.box2)  ){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox2 (sprite, this.onBox2.getChildAt(0));
    }
    else if (this.onBox2.total == 1 && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box6) && !this.isInside(sprite,this.box8) && this.onBox2.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else{
        console.log("something is wrong from box2 message-box or do nothing");
    }
    
    if (this.onBox4.total <1 && this.isInside(sprite,this.box4) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox6.removeChild(sprite) || this.onBox8.removeChild(sprite))){
        this.onBox4.add(sprite);
    }
    else if (this.onBox4.total< 1 && !this.isInside(sprite,this.box4)&& this.onBox4.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box4) && this.isInside(sprite,this.box6)){
        //do nothing
    }
    else if(this.onBox4.total == 1 && this.isInside(sprite, this.box4)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox4(sprite, this.onBox4.getChildAt(0));        
    }
    else if (this.onBox4.total == 1 && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box6) && !this.isInside(sprite,this.box8) && this.onBox4.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else{
        console.log("something is wrong from box4 message-box or do nothing");
    }
    
    
    if (this.onBox6.total <1 && this.isInside(sprite,this.box6) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox8.removeChild(sprite))){        
        this.onBox6.add(sprite);
    }
    else if (this.onBox6.total< 1 && !this.isInside(sprite,this.box6)&& this.onBox6.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box6) && this.isInside(sprite,this.box8)){
        //do nothing
    }
    else if(this.onBox6.total == 1 && this.isInside(sprite, this.box6)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox6(sprite, this.onBox6.getChildAt(0));
    }
    else if (this.onBox6.total == 1 && !this.isInside(sprite,this.box6) && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box8) &&  this.onBox6.removeChild(sprite)){    
        this.onPanel.add(sprite);
    }
    else {
        console.log("something is wrong from box6 message-box or do nothing");
    }
    
    if (this.onBox8.total <1 && this.isInside(sprite,this.box8) && (this.onPanel.removeChild(sprite) || this.onBox2.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox6.removeChild(sprite))){
        this.onBox8.add(sprite);
    }
    else if (this.onBox8.total< 1 && !this.isInside(sprite,this.box8)&& this.onBox8.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box6) && this.isInside(sprite,this.box8)){
        //do nothing
    }
    else if(this.onBox8.total == 1 && this.isInside(sprite, this.box8)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox8(sprite, this.onBox8.getChildAt(0) ) ;
    }
    else if (this.onBox8.total == 1 && !this.isInside(sprite,this.box8) && !this.isInside(sprite,this.box2) && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box6) &&  this.onBox8.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else {
        console.log("something is wrong from box8 message-box or do nothing");
    }
    
    //set the position of objects in thier corresponding box / panel
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
        this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox2.total; i++)
    {
        this.onBox2.getChildAt(i).x = this.box2.x + this.box2.width/2;
        this.onBox2.getChildAt(i).y = this.box2.y + this.box2.height/2;
    }
    
    for (var i = 0; i < this.onBox4.total; i++)
    {
        this.onBox4.getChildAt(i).x = this.box4.x + this.box4.width/2;
        this.onBox4.getChildAt(i).y = this.box4.y + this.box4.height/2;
    }
    
    for (var i = 0; i < this.onBox6.total; i++)
    {
        this.onBox6.getChildAt(i).x = this.box6.x + this.box6.width/2;
        this.onBox6.getChildAt(i).y = this.box6.y + this.box6.height/2;
    }
    
    for (var i = 0; i < this.onBox8.total; i++)
    {
        this.onBox8.getChildAt(i).x = this.box8.x + this.box8.width/2;
        this.onBox8.getChildAt(i).y = this.box8.y + this.box8.height/2;
    }
    

}

Activity4.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite, this.box1)){
        this.box1.frame = 1;
    }
    else {
        this.box1.frame = 0;
    }
    
    if (this.isInside(sprite, this.box2)){
        this.box2.frame = 1;
    }
    else {
        this.box2.frame = 0;
    }
    
    if (this.isInside(sprite,this.box4)){
        this.box4.frame = 1;
    }
    else {
        this.box4.frame = 0;
    }
    
    if (this.isInside(sprite,this.box6)){
        this.box6.frame = 1;
    }
    else {
        this.box6.frame = 0;
    }
    
    if (this.isInside(sprite,this.box8)){
        this.box8.frame = 1;
    }
    else {
        this.box8.frame = 0;
    }

}

Activity4.prototype.swapWithBox2 = function(sprite, spriteInBox2){
    
    //if the sprite can be remove from onpanel group
    // add the sprite to box2 and add spriteInbox2 to onpanel 
    // remove the first child in box2
    if (this.onPanel.removeChild(sprite)){
        this.onBox2.addChild(sprite);
        this.onPanel.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    }
    else if (this.onBox4.removeChild(sprite)){
        this.onBox2.addChild(sprite);
        this.onBox4.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    
    }
    else if(this.onBox6.removeChild(sprite)){
        this.onBox2.addChild(sprite);
        this.onBox6.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    }
    else if (this.onBox8.removeChild(sprite) ){
        this.onBox2.addChild(sprite);
        this.onBox8.addChild(spriteInBox2);
        this.onBox2.removeChild(spriteInBox2);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}
Activity4.prototype.swapWithBox4 = function(sprite, spriteInBox4){
    if (this.onPanel.removeChild(sprite)){
        this.onBox4.addChild(sprite);
        this.onPanel.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox2.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox2.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if(this.onBox6.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox6.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox8.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox8.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}

Activity4.prototype.swapWithBox6 = function(sprite, spriteInBox6){

    if (this.onPanel.removeChild(sprite)  ){
        this.onBox6.addChild(sprite);
        this.onPanel.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox2.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onBox2.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if(this.onBox4.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onBox4.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox8.removeChild(sprite) ){
        this.onBox6.addChild(sprite);
        this.onBox8.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}

Activity4.prototype.swapWithBox8 = function(sprite, spriteInBox8 ){

    if (this.onPanel.removeChild(sprite)){
        this.onBox8.addChild(sprite);
        this.onPanel.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else if (this.onBox2.removeChild(sprite) ){
        this.onBox8.addChild(sprite);
        this.onBox2.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else if(this.onBox4.removeChild(sprite) ){
        this.onBox8.addChild(sprite);
        this.onBox4.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else if (this.onBox6.removeChild(sprite) ){
        this.onBox8.addChild(sprite);
        this.onBox6.addChild(spriteInBox8);
        this.onBox8.removeChild(spriteInBox8);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}
    
Activity4.prototype.checkOverlapWithRect = function(spriteA, rectangle) {
    var boundsA = spriteA.getBounds();
    return Phaser.Rectangle.intersects(boundsA, rectangle);
}

Activity4.prototype.isInside = function(object,box){
    if (object.x>box.x+0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity4.prototype.checkSequence = function(){
    
    //if the box are empty, display incorrect message 
    //avoid the case for children[0] being undefined whenever there is an empty box
    if(this.onBox2.children[0] == null || this.onBox4.children[0] == null || this.onBox6.children[0] == null || this.onBox8.children[0] == null){
        this.attempts++;
        this.publishRobotGameFeedback();
    }
    
    else if (this.onBox1.children[0].key == this.onBox2.children[0].key &&
        this.onBox3.children[0].key == this.onBox4.children[0].key &&
        this.onBox5.children[0].key == this.onBox6.children[0].key &&
        this.onBox7.children[0].key == this.onBox8.children[0].key )
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

Activity4.prototype.boxCoordinates = function(boxArrayX,boxArrayY, box){
    for (var i =0; i< OBJECT_AMOUNT; i++){
        boxArrayX[i] = box.x+ 130 *this.sx+(i%4)* 150*this.sx;
        boxArrayY[i] = box.y+ 50 *this.sy + Math.floor(i/4)* 50 *this.sy;
    }
}

module.exports = Activity4;
