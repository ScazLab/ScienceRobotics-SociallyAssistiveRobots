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


const COLUMNS_PANEL = 4;
const COLUMNS_BATTERY = 4;

const ACTIVITY_ID = 2;

/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity2(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920,
        gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onBattery;
    this.box;
    this.battery;
    this.count = 0;
    this.target;
    this.numCrystals = 10;
    this.scoreText;
    this.promptText;
    this.dragging = false;
    this.draggingSecond = false;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.logo;
    this.controlPanel;
    this.button;

    //create variable panel with two coordinates properties
    this.panel = {
      CoordX : new Array(),
      CoordY : new Array()
    };
}
Activity2.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity2.prototype.preload = function() {
    // Load static image assets
    this.game.load.image('logo', '../../assets/Background/ShipInterior.png'); // logo means background image
    this.game.load.image('controlPanel', CONST.ASSETS_PATH + 'Items/ControlPanelcrop.png'); // frame box where crystals located

    // Load sprites
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH + 'Items/Crystals/Crystal1_sheet.png',460,560,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH + 'Items/Crystals/Crystal2_sheet.png',460,510,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH + 'Items/Crystals/Crystal3_sheet.png',410,460,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH + 'Items/Crystals/Crystal4_sheet.png',460,410,2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH + 'Items/Crystals/Crystal5_sheet.png',410,360,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH + 'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('battery', CONST.ASSETS_PATH + 'Items/Battery_sheet.png',560,810,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH + 'Items/Box_sheet.png',881,730,2);

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets
 */
Activity2.prototype.create = function() {
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    this.logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is ' + this.game.width);
    console.log('Game height is ' + this.game.height);

    // Control Panel
    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.95 * this.sx, 0.95 * this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;

    // Button
    this.button = this.game.add.sprite(this.controlPanel.x+this.controlPanel.width/2,this.controlPanel.y+this.controlPanel.height-200,'button');
    this.button.scale.setTo(0.4*this.sx);
    this.button.scaleMin = 0.3;
    this.button.x = this.controlPanel.x+this.controlPanel.width/2-this.button.width/2;
    this.button.y = this.controlPanel.y+this.controlPanel.height-this.button.height-0.1*this.game.height;
    this.button.frame=0;
    this.button.inputEnabled=true;
    this.button.events.onInputDown.add(this.onButtonPress.bind(this), this);
    this.button.events.onInputUp.add(onButtonRelease,this);

    // Battery
    this.battery = this.game.add.sprite(0,0, 'battery');
    this.battery.scale.setTo(0.8*this.sx,0.85*this.sy);
    this.battery.x = this.controlPanel.x + this.controlPanel.width - this.battery.width - 200*this.sx;
    this.battery.y = this.controlPanel.y + this.controlPanel.height/2 - this.battery.height/2;
    this.battery.inputEnabled=true;
    //Custom properties, CoordX and CoordY, added to object battery
    this.battery.CoordX = new Array();
    this.battery.CoordY = new Array();

    // Box
    this.box = this.game.add.sprite(0,0,'box');
    this.box.frame = 1;
    this.box.scale.setTo(0.2*this.sx,0.25*this.sy);
    this.box.x = this.controlPanel.x + this.controlPanel.width/2 - this.box.width/2;
    this.box.y = this.controlPanel.y + this.controlPanel.height/5 - this.box.height/2;
    
    // Set Phaser groups
    this.onPanel = this.game.add.group();
    this.onBattery = this.game.add.group();

    // this.numCrystals = this.randNumCrystals(); 
    // uncomment to randomize the total number of crystals
    
    for (var i=0;i<this.numCrystals;i++){
        var objectname = "object" + this.game.rnd.integerInRange(1,5);
        
        //var object = objects.create(controlPanel.x+220*sx+(arr[i]%4)*150*sx,controlPanel.y+150*sy+Math.floor(arr[i]/4)*180*sy,objectname);
        
        this.panel.CoordX[i] = this.controlPanel.x+220*this.sx+(i%COLUMNS_PANEL)*150*this.sx;
        this.panel.CoordY[i] = this.controlPanel.y+130*this.sy+Math.floor(i/COLUMNS_PANEL)*150*this.sy;
        //this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], objectname);
        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], objectname);
    }
    
    this.batteryCoordinates();

    this.onPanel.forEach(function(object){
        object.anchor.setTo(0.5,0.5);
        object.frame=0;
        object.scale.setTo(0.35*this.sx);
        object.inputEnabled=true;
        object.input.enableDrag(false,false,false,255,null,this.controlPanel);
        object.events.onDragStart.add(this.onDragStart.bind(this), this);
        object.events.onDragStop.add(this.onDragStop.bind(this), this);
        object.events.onDragUpdate.add(this.onDragUpdate.bind(this), this);
        //object.input.enableSnap(100*sx,100*sy,false,true);
    }, this);

    var fontSize = 100*this.sx;
    this.target = this.randNumTarget(this.numCrystals);
    this.scoreText = this.game.add.text(this.game.width/2,0.05*this.game.height,'Count: 0',{fontSize:'24px',fill:'#fff'});
    this.scoreText.x=this.game.width/2-this.scoreText.width/2;
    this.promptText = this.game.add.text(this.box.x+this.box.width/2,this.box.y+this.box.height/2,this.target,{fontSize:fontSize+'px',fill:'#fff'});
    this.promptText.anchor.setTo(0.5,0.5);

    // Publish Robot Command to give starting instructions
    // Only pluralize "crystal" if more than one crystal
    var crystalMsg = this.target > 1 ? "crystals" : "crystal";
    this.publishRobotGameIntro(this.target, crystalMsg);
};


/**
 * Computes the (x,y) coordinates of each position in the battery where a crystal can be placed
 */
Activity2.prototype.batteryCoordinates = function() {
    for (var i = 0; i < this.numCrystals; i++)
    {
        this.battery.CoordX[i] = this.controlPanel.x + this.battery.x +  20*this.sx + i%3* 120*this.sx;
        this.battery.CoordY[i] = this.controlPanel.y + this.battery.y + 100*this.sy + Math.floor(i/3)*145*this.sy;
    }
};


/**
 * Handler for when the button sprite is pressed
 *    Counts crystals in battery. If count same as target number, game is complete
 */
Activity2.prototype.onButtonPress = function(sprite,pointer) {
    sprite.frame=1;
    this.count = this.crystalCount();
    if (!this.finished(this.count))
        this.attempts++;
    else
        this.gameComplete();
};


function onButtonRelease(sprite,pointer){
    sprite.frame=0;
}


/**
 * Handler for when crystal is grabbed
 */
Activity2.prototype.onDragStart = function(sprite,pointer) {
    sprite.frame=1;
    if (this.isInside(sprite)) this.battery.frame = 1;

    if (this.dragging) 
        this.draggingSecond = true;
    this.dragging = true;
};


/**
 * Update crystal when dragged around
 */
Activity2.prototype.onDragUpdate = function(sprite,pointer) {
    if (this.isInside(sprite)) this.battery.frame = 1;
    else if (!this.isInside(sprite)) this.battery.frame = 0;
};


/**
 * Handler for when crystal is released
 */
Activity2.prototype.onDragStop = function(sprite,pointer) {
    sprite.frame=0;
    this.battery.frame = 0;
    if (this.isInside(sprite) && this.onPanel.removeChild(sprite))
    {
        this.onBattery.add(sprite);
    }
    else if (!this.isInside(sprite) && this.onBattery.removeChild(sprite))
    {
        this.onPanel.add(sprite);
    }
    console.log("onPanel.total: " + this.onPanel.total + " onBox.total: " + this.onBattery.total);

    if (this.draggingSecond)
        this.draggingSecond = false;
    else {
        this.dragging = false
        //Reorders the objects on the Panel
        for (var i = 0; i < this.onPanel.total; i++)
        {
            this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
            this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
        }
        
        //Reorders the objects on the Battery
        for (var i = 0; i < this.onBattery.total; i++)
        {
            this.onBattery.getChildAt(i).x = this.battery.CoordX[i];
            this.onBattery.getChildAt(i).y = this.battery.CoordY[i];
        }
    }
};


/**
 * Helper function to determine if an object is inside the battery sprite
 * @returns {Bool} True if object inside battery sprite, false otherwise
 */
Activity2.prototype.isInside = function(object) {
    if (object.x>this.battery.x+0.01*this.game.width && object.y>this.battery.y+0.01*this.game.height && object.x<this.battery.x+this.battery.width-0.01*this.game.width && object.y<this.battery.y+this.battery.height-0.01*this.game.height) return true;
    else return false;
}


/**
 * Count the number of crystals inside the battery
 * @returns {Int}
 */
Activity2.prototype.crystalCount = function() {
    var count = 0;
    this.onBattery.forEach(function(object){
        if (this.isInside(object)){
            console.log("Dropped inside the battery!");
            count++;
        }
    }, this);
    this.scoreText.setText('Count: '+count);
    return count;
}


/**
 * Determines if the target number of crystals dropped into the battery
 *    Publishes a robot command to provide feedback or congratulate player.
 * @return {Bool} True if game is complete, false otherwise
 */
Activity2.prototype.finished = function(crystals) {
    if (crystals==this.target){ 
        //alert('Yayy!');
        this.onBattery.forEach(function(object){ object.input.draggable = false;} );
        this.onPanel.forEach(function(object){ object.input.draggable = false;} );
        
        this.publishRobotSuccessMessage();
        return true;
    }
    else {
        // provide specific feedback on the mistake
        this.publishRobotGameFeedback(this.target, crystals > this.target ? 'fewer': 'more');
        return false;
    }
};


Activity2.prototype.randNumTarget = function(numCrystals) {
    switch(this.level) {
        case 1:
            return this.game.rnd.integerInRange(1, 3);
        case 2:
            return this.game.rnd.integerInRange(4, 6);
        case 3:
            return this.game.rnd.integerInRange(7, 9);
        default: 
            throw CONST.UNEXPECTED_LEVEL;
    }   
};

Activity2.prototype.randNumCrystals = function() {
    switch(this.level) {
        case 1:
            return this.game.rnd.integerInRange(8, 12);
        case 2:
            return this.game.rnd.integerInRange(13, 15);
        case 3:
            return this.game.rnd.integerInRange(16, 19);
        default:
            throw CONST.UNEXPECTED_LEVEL;
    }
}


module.exports = Activity2;
