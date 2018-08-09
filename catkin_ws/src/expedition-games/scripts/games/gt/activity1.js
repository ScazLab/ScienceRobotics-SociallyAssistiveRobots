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


const ROCK_AMOUNT = 10;
const COLUMNS_PANEL = 4;
const COLUMNS_BOX = 4;
const ACTIVITY_ID = 1;

/**
 * Constructor for Activity 2
 * @param {ROS} ros handler
 * @param {Int} level/difficulty
 */
function Activity1(ros, level, gameid, completionCallBack) {
    // Call Parent constructor
    SARGame.call(this, ros, level, gameid, completionCallBack, ACTIVITY_ID);

    var gameWidth = 1920,
        gameHeight = 1080;

    // Declare object properties
    this.onPanel;
    this.onBox;
    this.box;
    this.logo;
    this.button;
    this.controlPanel;
    this.scoreText;
    this.promptText;
    this.target;
    this.sx = window.innerWidth/gameWidth;
    this.sy = window.innerHeight/gameHeight;
    this.logo;
    this.controlPanel;
    this.button;
    this.dragging = false;
    this.draggingSecond = false;

    //create variable panel with two coordinates properties
    this.panel = {
      CoordX : new Array(),
      CoordY : new Array()
    };
}
Activity1.prototype.__proto__ = SARGame.prototype; // Inherit from SARGame


/**
 * Phaser Handler for loading assets
 */
Activity1.prototype.preload = function() {
    // Load static image assets
    this.game.load.image('logo', '../../assets/Background/ShipInterior.png'); // logo means background image
    this.game.load.image('controlPanel', CONST.ASSETS_PATH + 'Items/ControlPanelcrop.png'); // frame box where crystals located

    // Load sprites
    this.game.load.spritesheet('object1', CONST.ASSETS_PATH+'Items/Rocks/Moonrock1_sheet.png',610,400,2);
    this.game.load.spritesheet('object2', CONST.ASSETS_PATH+'Items/Rocks/Moonrock2_sheet.png',460,360,2);
    this.game.load.spritesheet('object3', CONST.ASSETS_PATH+'Items/Rocks/Moonrock3_sheet.png',660,360,2);
    this.game.load.spritesheet('object4', CONST.ASSETS_PATH+'Items/Rocks/Moonrock4_sheet.png',510,410,2);
    this.game.load.spritesheet('object5', CONST.ASSETS_PATH+'Items/Rocks/Moonrock5_sheet.png',660,360,2);
    this.game.load.spritesheet('button', CONST.ASSETS_PATH+'Items/Button_sheet.png',288,288,2);
    this.game.load.spritesheet('box', CONST.ASSETS_PATH+'Items/Box_sheet.png',881,730,2);

    // Number of moonrocks needed to be in box randomly generated
    this.target = this.generateTarget();

    window.addEventListener('resize', (event) => {this.resize();});
};


/**
 * Phaser Handler for creating sprites/assets
 */
Activity1.prototype.create = function() {
    // background image is displayed
    this.logo = this.game.add.sprite(this.game.world.centerX, this.game.world.centerY, 'logo');
    //  Moves the image anchor to the middle, so it centers inside the game properly
    this.logo.anchor.setTo(0.5, 0.5);
    console.log('Game width is'+ this.game.width);
    console.log('Game height is'+ this.game.height);

    // controlPanel where moonrock located image is created on screen
    this.controlPanel = this.game.add.sprite(0,0,'controlPanel');
    this.controlPanel.scale.setTo(0.8*this.sx,0.8*this.sy);
    this.controlPanel.x = this.game.world.centerX - this.controlPanel.width/2;
    this.controlPanel.y = this.game.world.centerY - this.controlPanel.height/2;


    //button image is loaded
    this.button = this.game.add.sprite(this.controlPanel.x + this.controlPanel.width/2,
                                       this.controlPanel.y + this.controlPanel.height-200, 'button');
    this.button.scale.setTo(0.3);
    this.button.x = this.controlPanel.x + this.controlPanel.width / 2 - this.button.width / 2;
    this.button.y = this.controlPanel.y + this.controlPanel.height - this.button.height-0.05 * this.game.height;
    //button frame 
    this.button.frame=0;
    //Enables all kind of input actions on this image (click, etc)
    this.button.inputEnabled=true;
    //onButtonPress function
    this.button.events.onInputDown.add(this.onButtonPress,this);
    this.button.events.onInputUp.add(this.onButtonRelease,this);

    this.box = this.game.add.sprite(0,0, 'box');
    this.box.scale.setTo(0.6 * this.sx, 0.8 * this.sy);
    this.box.x = this.controlPanel.x + this.controlPanel.width - this.box.width - 130 * this.sx;
    this.box.y = this.controlPanel.y + 80 * this.sx;
    this.box.inputEnabled = true;
    this.box.frame = 0;

    //Custom properties, CoordX and CoordY, added to object box1
    this.box.CoordX = new Array();
    this.box.CoordY = new Array();

    console.log('Box height is '+ this.box.height);
    console.log('Box width is '+ this.box.width);

    console.log('cp width and height are '+ this.controlPanel.width + 'x' + this.controlPanel.height);

    this.onPanel = this.game.add.group();
    this.onBox = this.game.add.group();


    for (var i=0;i<ROCK_AMOUNT;i++){
        var objectname = "object" + this.game.rnd.integerInRange(1,5);
        this.panel.CoordX[i] = this.controlPanel.x + 220 * this.sx + (i % COLUMNS_PANEL) * 150 * this.sx;
        this.panel.CoordY[i] = this.controlPanel.y + 130 * this.sy + Math.floor(i / COLUMNS_PANEL) * 150 * this.sy;

        this.onPanel.create(this.panel.CoordX[i], this.panel.CoordY[i], objectname);
    }

    // set the rocks coordinate/ position in the box
    this.boxCoordinates();

    this.onPanel.forEach((object) => {
        object.anchor.setTo(0.5,0.5);
        object.frame = 0;
        object.scale.setTo(0.25 * this.sx);
        //object.originalPosition = object.position.clone();
        //object.alignIn(controlPanel, Phaser.BOTTOM_LEFT, -20, -20);
        object.inputEnabled = true;
        object.input.enableDrag(false,false,false,255,null, this.controlPanel);
        //object.input.enableSnap(130 *sx,150 *sy,false,true);
        object.events.onDragStart.add(this.onDragStart,this);
        object.events.onDragStop.add(this.onDragStop,this);
        object.events.onDragUpdate.add(this.onDragUpdate,this);
    });


    this.scoreText = this.game.add.text(this.game.width / 2, 0.1 * this.game.height, 'Count: 0', {fontSize:'24px',fill:'#000'});
    this.scoreText.x = this.game.width / 2 - this.scoreText.width / 2;

    this.promptText = this.game.add.text(this.game.width / 2, 0.03 * this.game.height,'Please move moonrocks into the box',
                                         {fontSize:'24px',fill:'#fff'});
    this.promptText.x = this.game.width / 2 - this.promptText.width / 2;
    var question = 'Please move ' + this.target + ' moonrocks into the box';
    this.promptText.setText(question);

    // Publish Robot Command to give starting instructions
    this.publishRobotGameIntro(this.target);
};

Activity1.prototype.boxCoordinates = function() {
    for (var i = 0; i < ROCK_AMOUNT; i++)
    {
        this.box.CoordX[i] = this.controlPanel.x + 960 * this.sx + i%3*150*this.sx;
        this.box.CoordY[i] = this.controlPanel.y + 160 * this.sy + Math.floor(i/3)*100*this.sy;
    }
};

Activity1.prototype.outOfBox = function(sprite){
    sprite.loadTexture('box');
};

Activity1.prototype.onButtonPress = function(sprite,pointer){
    sprite.frame=1;
    this.stat = this.countRocks();
    if (!this.stat[0]) this.attempts++;

    if (this.stat[0]){
        this.publishRobotSuccessMessage();
        this.gameComplete();
    }
    else {
        var feedback = "That doesn't look right. ";
        if (this.stat[1] == 'less') {
            feedback += "Do we need more?";
        }
        else if (this.stat[1] == 'more') {
            feedback += "Is that too many?";
        }
        // provide specific feedback on the mistake
        this.publishRobotGameFeedback(this.target, this.stat[1] == 'less' ? 'more': 'fewer');
    }
};

Activity1.prototype.onButtonRelease = function(sprite,pointer){
    sprite.frame=0;
};

Activity1.prototype.onDragStart = function(sprite,pointer){
    sprite.frame=1;

    if (this.dragging) 
        this.draggingSecond = true;
    this.dragging = true;
};

Activity1.prototype.onDragStop = function(sprite,pointer){
    sprite.frame=0;
    this.box.frame = 0;
                
        if (this.isInside(sprite) && this.onPanel.removeChild(sprite))
        {
            this.onBox.add(sprite);
        }
        else if (!this.isInside(sprite) && this.onBox.removeChild(sprite))
        {
            this.onPanel.add(sprite);
        }
        console.log("onPanel.total: " + this.onPanel.total + " onBox.total: " + this.onBox.total);
    
    if (this.draggingSecond)
        this.draggingSecond = false;
    else {
        this.dragging = false
        //Reorders the rocks on the Panel
        for (var i = 0; i < this.onPanel.total; i++)
        {
            this.onPanel.getChildAt(i).x = this.panel.CoordX[i];
            this.onPanel.getChildAt(i).y = this.panel.CoordY[i];
        }
        
        //Reorders the rocks on the Box
        for (var i = 0; i < this.onBox.total; i++)
        {
            this.onBox.getChildAt(i).x = this.box.CoordX[i];
            this.onBox.getChildAt(i).y = this.box.CoordY[i];
        }
    }
}

Activity1.prototype.onDragUpdate = function(sprite,pointer){
    if (this.isInside(sprite)) this.box.frame = 1;
    else if (!this.isInside(sprite)){
        this.box.frame = 0;
    }
}

Activity1.prototype.isInside = function(object){
    if (object.x > this.box.x+0.01*this.game.width && 
        object.y > this.box.y +0.01*this.game.height && 
        object.x < this.box.x+this.box.width-0.01*this.game.width && 
        object.y < this.box.y+this.box.height-0.01*this.game.height) return true;
    else return false;
}

Activity1.prototype.countRocks = function(){
    console.log("Count in box! : " + this.onBox.total);

    this.scoreText.setText('Count: '+ this.onBox.total);
    if (this.onBox.total==this.target){
        //disable the drag properties
        this.onBox.forEach(function(object){object.input.draggable = false;});
        this.onPanel.forEach(function(object){object.input.draggable = false;});
        //game.input.mouse.enabled = false;
        return [true, true];
    } else 
        return [false,(this.onBox.total<this.target)?"less":"more"];
}

Activity1.prototype.generateTarget = function() {
    if (this.level == 1) {
        return this.game.rnd.integerInRange(1,3);
    } else if (this.level == 2) {
        return this.game.rnd.integerInRange(4,6);
    } else if (this.level == 3) {
        return this.game.rnd.integerInRange(7,9);
    } else {
        throw CONST.UNEXPECTED_LEVEL;
    }
}


module.exports = Activity1;
