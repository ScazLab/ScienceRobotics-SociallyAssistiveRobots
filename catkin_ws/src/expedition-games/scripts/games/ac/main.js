'use strict';

var Activity1 = require('./activity1.js');
var Activity2 = require('./activity2.js');
var Activity3 = require('./activity3.js');
var Activity4 = require('./activity4.js');
var Activity5 = require('./activity5.js');
var Activity6 = require('./activity1.js');
var Activity7 = require('./activity2.js');
var Activity8 = require('./activity3.js');
var Activity9 = require('./activity4.js');
var Activity10 = require('./activity5.js');

var GameWrapper = require('../../SARGame.js').GameWrapper;
var CONST = require('../../constants.js');


function AlienCodes(ros, level, exitCallBack) {
    GameWrapper.call(this, ros, level, exitCallBack);
    this.gameid = CONST.Games.ALIEN_CODES;
    this.activities = [Activity1, Activity2, Activity3, Activity4, Activity5, Activity6, Activity7, Activity8, Activity9, Activity10];
    // this.activities = [Activity1, Activity2];
    this.start();
}
AlienCodes.prototype.__proto__ = GameWrapper.prototype;


module.exports = AlienCodes;
