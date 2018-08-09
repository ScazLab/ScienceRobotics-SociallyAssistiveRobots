var SAR = require('./SARGame');

exports.constants = require('./constants.js');
exports.SARGame = SAR.SARGame;
exports.LoadingScreen = SAR.LoadingScreen;
exports.Menu = SAR.Menu;

// Games
exports.GalacticTraveler = require('./games/gt/main.js');
exports.SpaceshipTidyup = require('./games/st/main.js');
exports.AlienCodes = require('./games/ac/main.js');

// Tutorials
exports.GalacticTravelerTutorial = require('./games/tutorial/gt_tutorial.js');
exports.SpaceshipTidyupTutorial = require('./games/tutorial/st_tutorial.js');
exports.AlienCodesTutorial = require('./games/tutorial/ac_tutorial.js');
