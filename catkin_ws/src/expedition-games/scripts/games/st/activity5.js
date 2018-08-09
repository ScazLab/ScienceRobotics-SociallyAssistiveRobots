/**
  * Space Ship Tidy Up Activity 5
  * -----------------------------
  *      
  */    

'use strict';

var SARGame = require('../../SARGame.js').SARGame;
var CONST = require('../../constants.js');

const ACTIVITY_ID = 5;
const OBJECT_AMOUNT = 10;
const COLUMNS_PANEL = 5;
/**
 * Constructor for Activity 5
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity5(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920;
    var gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onbox1;
    this.onbox2;

    this.box1;
    this.box2;
    this.box3;
    this.box4;
    this.box5;
    this.box6;                   

    //set a string array called objectname and shuffle them
    // tempObj is randomize
    this.tempObj1 = "object" + this.game.rnd.integerInRange(1,2);
    this.tempObj2 = "object" + this.game.rnd.integerInRange(3,4);
    this.tempObj3 = "object" + this.game.rnd.integerInRange(5,6);
    this.tempObj4 = "object" + this.game.rnd.integerInRange(1,6); 
    
    this.objectname  =  ['object1', 'object2', 'object3', 'object4' , 'object5', 'object6', this.tempObj1, this.tempObj2, this.tempObj3, this.tempObj4];
    this.shuffle(this.objectname);
    
    //set up a sequence array
    this.sequenceObj = [this.tempObj1, this.tempObj2, this.tempObj3];
    this.shuffle(this.sequenceObj);

    //create variable panel with two coordinates properties
    this.panel = {
        CoordX : new Array(),
        CoordY : new Array()
    }

    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
}
Activity5.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity5.prototype.preload = function() {
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
Activity5.prototype.create  = function() {
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
    this.box1.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box1.x = controlPanel.x + 220*this.sx; //controlPanel.x + 1*((controlPanel.width - 220*sx -220*sx - 80*sx )/ 6); 
    //box1.y = controlPanel.y + controlPanel.height/2 - box1.height/2 - button.height - 50 *sx;
    this.box1.y = controlPanel.y +controlPanel.height/8;
    this.box1.frame = 0;
    this.box1.visible = false;
                            
    var rec0 = this.game.add.sprite(0,0, 'bigBox');
    //rec0.scale.setTo(1.25*sx,0.25*sy);
    rec0.scale.setTo(0.755*this.sx,0.4*this.sy);
    rec0.x = this.box1.x - 40*this.sx;
    rec0.y = this.box1.y - 25*this.sy;//box1.y - 20*sy;
    rec0.frame = 0;
    rec0.visible = true;
    
    this.box2 = this.game.add.sprite(0,0, 'box');
    this.box2.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box2.x = this.box1.x + this.box1.width + 20*this.sx; //controlPanel.x +  2*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)
    this.box2.y = controlPanel.y +controlPanel.height/8;
    this.box2.frame = 0;
    this.box2.visible = false;
    
    this.box3 = this.game.add.sprite(0,0, 'box');
    this.box3.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box3.x = this.box2.x + this.box2.width + 20*this.sx; // controlPanel.x + 3*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)
    this.box3.y = controlPanel.y +controlPanel.height/8;
    this.box3.frame = 0;
    this.box3.visible=false;
    
    this.box4 = this.game.add.sprite(0,0, 'box');
    this.box4.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box4.x = this.box3.x + this.box3.width + 20*this.sx; // controlPanel.x + 4*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)
    this.box4.y = controlPanel.y +controlPanel.height/8;
    this.box4.frame = 0;
    //box4.visible = false;
    
    this.box5 = this.game.add.sprite(0,0, 'box');
    this.box5.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box5.x = this.box4.x + this.box4.width + 20*this.sx; //controlPanel.x  + 5*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6)//box4.x + box4.width + 20*sx;
    this.box5.y = controlPanel.y +controlPanel.height/8;
    this.box5.frame = 0;
    //box5.visible = false;
    
    this.box6 = this.game.add.sprite(0,0, 'box');
    this.box6.scale.setTo(0.18*this.sx,0.2*this.sy);
    this.box6.x = this.box5.x + this.box5.width + 20*this.sx; //controlPanel.x + 6*((controlPanel.width - 220*sx -220*sx - 80*sx)/ 6) box5.x + box5.width + 20*sx;
    this.box6.y = controlPanel.y +controlPanel.height/8;
    this.box6.frame = 0;
    
    this.onPanel = this.game.add.group();
    this.onBox1 = this.game.add.group();
    this.onBox2 = this.game.add.group();
    this.onBox3 = this.game.add.group();
    this.onBox4 = this.game.add.group();
    this.onBox5 = this.game.add.group();
    this.onBox6 = this.game.add.group();

    //generate each object on panel
    for (var i=0; i< OBJECT_AMOUNT; i++){
        this.panel.CoordX[i] = controlPanel.x + controlPanel.width/2 - 400*this.sx +(i%COLUMNS_PANEL) * 180*this.sx;
        this.panel.CoordY[i] = controlPanel.y + controlPanel.height/2 + Math.floor(i/COLUMNS_PANEL) * 140*this.sy;
        
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], this.objectname[i]);
    }
    

    //create the sequence/pattern on box1/2/3/4
    this.onBox1.create(this.box1.x + this.box1.width/2, this.box1.y + this.box1.height/2, this.sequenceObj[0]);
    this.onBox2.create(this.box2.x + this.box2.width/2, this.box2.y + this.box2.height/2, this.sequenceObj[1]);
    this.onBox3.create(this.box3.x + this.box3.width/2, this.box3.y + this.box3.height/2, this.sequenceObj[2]);
    
    this.onBox1.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    
    this.onBox2.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox3.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox4.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox5.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.25*this.sx);
    });
    this.onBox6.forEach((object) => {
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

Activity5.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');

}


Activity5.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.checkSequence();
}


Activity5.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
}

Activity5.prototype.onDragStart = function(sprite, pointer){
    sprite.frame=1;
}
    
Activity5.prototype.onDragStop = function(sprite, pointer){
    this.box4.frame = 0;
    this.box5.frame = 0;
    this.box6.frame = 0;
    sprite.frame = 0;                   
    
    if (this.onBox4.total <1 && this.isInside(sprite,this.box4) && (this.onPanel.removeChild(sprite) || this.onBox5.removeChild(sprite) || this.onBox6.removeChild(sprite))){        
        this.onBox4.add(sprite);
    }
        
    else if (this.onBox4.total< 1 && !this.isInside(sprite,this.box4)&& this.onBox4.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box4) && this.isInside(sprite,this.box5)){
        //do nothing
    }
    else if(this.onBox4.total == 1 && this.isInside(sprite, this.box4)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox4(sprite, this.onBox4.getChildAt(0));
    }
    else if (this.onBox4.total == 1 && !this.isInside(sprite,this.box4) && !this.isInside(sprite,this.box5) && !this.isInside(sprite,this.box6) && this.onBox4.removeChild(sprite)  ){
        this.onPanel.add(sprite);
    }
    else {
        console.log("something is wrong from box4 message-box or do nothing");
    }
    
    if (this.onBox5.total <1 && this.isInside(sprite,this.box5) && (this.onPanel.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox6.removeChild(sprite))){
        this.onBox5.add(sprite);
    }
    else if (this.onBox5.total< 1 && !this.isInside(sprite,this.box5)&& this.onBox5.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box5) && this.isInside(sprite,this.box6)){
        //do nothing
    }
    else if(this.onBox5.total == 1 && this.isInside(sprite, this.box5)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox5(sprite, this.onBox5.getChildAt(0));
    }
    else if (this.onBox5.total == 1 && !this.isInside(sprite,this.box5) && !this.isInside(sprite,this.box6) && this.onBox5.removeChild(sprite)  ){
        this.onPanel.add(sprite);
    }
    else {
            console.log("something is wrong from box5 message-box or do nothing");
    }
    
    if (this.onBox6.total<1 && this.isInside(sprite, this.box6) &&  (this.onPanel.removeChild(sprite) || this.onBox4.removeChild(sprite) || this.onBox5.removeChild(sprite))){    
        this.onBox6.add(sprite);
    }
    else if(this.onBox6.total < 1 && !this.isInside(sprite, this.box6) && this.onBox6.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else if (this.isInside(sprite,this.box5) && this.isInside(sprite,this.box6)){
        //do nothing
    }
    else if(this.onBox6.total ==1 && this.isInside(sprite,this.box6)){
        // if something is dragged into box, check the condition and swap them
        // otherwise set it back to the its location
        this.swapWithBox6(sprite, this.onBox6.getChildAt(0));
    }
    else if (this.onBox6.total == 1 && !this.isInside(sprite,this.box6)  && !this.isInside(sprite,this.box5) &&  this.onBox6.removeChild(sprite)){
        this.onPanel.add(sprite);
    }
    else{
        console.log("do nothing");
    }
    
    //set the position of objects in thier corresponding box / panel
    for (var i = 0; i < this.onPanel.total; i++)
    {
        this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
        this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
    }
    
    for (var i = 0; i < this.onBox4.total; i++)
    {
        this.onBox4.getChildAt(i).x = this.box4.x + this.box4.width/2;
        this.onBox4.getChildAt(i).y = this.box4.y + this.box4.height/2;
    }
    
    for (var i = 0; i < this.onBox5.total; i++)
    {
        this.onBox5.getChildAt(i).x = this.box5.x + this.box5.width/2;
        this.onBox5.getChildAt(i).y = this.box5.y + this.box5.height/2;
    }
    

    for (var i = 0; i < this.onBox6.total; i++)
    {
        this.onBox6.getChildAt(i).x = this.box6.x + this.box6.width/2;
        this.onBox6.getChildAt(i).y = this.box6.y + this.box6.height/2;
    }
}

Activity5.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite, this.box4)){
        this.box4.frame = 1;
    }
    else if (!this.isInside(sprite, this.box4)){
        this.box4.frame = 0;
    }
    
    if (this.isInside(sprite, this.box5)){
        this.box5.frame = 1;
    }
    else if (!this.isInside(sprite, this.box5)){
        this.box5.frame = 0;
    }

    if (this.isInside(sprite, this.box6)){
        this.box6.frame = 1;
    }
    else if (!this.isInside(sprite,this.box6)){
        this.box6.frame = 0;
    }
}

Activity5.prototype.swapWithBox4 = function(sprite, spriteInBox4 ){
    
    //if the sprite can be remove from onpanel group
    // add the sprite to box4 and add spriteInbox5 to onpanel 
    // remove the first child in box4
    if (this.onPanel.removeChild(sprite)){
        this.onBox4.addChild(sprite);
        this.onPanel.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox5.removeChild(sprite)){
        this.onBox4.addChild(sprite);
        this.onBox5.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else if (this.onBox6.removeChild(sprite) ){
        this.onBox4.addChild(sprite);
        this.onBox6.addChild(spriteInBox4);
        this.onBox4.removeChild(spriteInBox4);
    }
    else{
        console.log("random error I didnt consider  or do nothing");
    }
}

Activity5.prototype.swapWithBox5 = function(sprite, spriteInBox5 ){
    if (this.onPanel.removeChild(sprite)){
        this.onBox5.addChild(sprite);
        this.onPanel.addChild(spriteInBox5);
        this.onBox5.removeChild(spriteInBox5);
    }
    else if (this.onBox4.removeChild(sprite)){
        this.onBox5.addChild(sprite);
        this.onBox4.addChild(spriteInBox5);
        this.onBox5.removeChild(spriteInBox5);
    }
    
    else if (this.onBox6.removeChild(sprite) ){
        this.onBox5.addChild(sprite);
        this.onBox6.addChild(spriteInBox5);
        this.onBox5.removeChild(spriteInBox5);
    }
}

Activity5.prototype.swapWithBox6 = function(sprite, spriteInBox6 ){
    if (this.onPanel.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onPanel.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox4.removeChild(sprite)){
        this.onBox6.addChild(sprite);
        this.onBox4.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else if (this.onBox5.removeChild(sprite) ){
        this.onBox6.addChild(sprite);
        this.onBox5.addChild(spriteInBox6);
        this.onBox6.removeChild(spriteInBox6);
    }
    else{
        console.log("random error I didnt consider or do nothing");
    }
}
    

Activity5.prototype.isInside = function(object,box){
    if (object.x>box.x+0.01*this.game.width && object.y>box.y+0.01*this.game.height && object.x<box.x+box.width-0.01*this.game.width && object.y<box.y+box.height-0.01*this.game.height) return true;
    else return false;
}

Activity5.prototype.checkSequence = function(){
    //if the box are empty, display incorrect message 
    //avoid the case for children[0] being undefined whenever there is an empty box
    if(this.onBox4.children[0] == null || this.onBox5.children[0] == null || this.onBox6.children[0] == null ){
        this.attempts++;
        alert("Incorrect, Please fill up the empty box");
        this.publishRobotGameFeedback();
    }
    else if (this.onBox1.children[0].key == this.onBox4.children[0].key &&
        this.onBox2.children[0].key == this.onBox5.children[0].key &&
        this.onBox3.children[0].key == this.onBox6.children[0].key  )
    {
        this.publishRobotSuccessMessage();
        this.gameComplete();
        return true;
    }
    else{
        this.attempts++;
        alert("Incorrect, wrong sequence");
        this.publishRobotGameFeedback();
    }
}

module.exports = Activity5;
