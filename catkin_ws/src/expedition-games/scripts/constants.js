exports.Games = Object.freeze({STORYTELLING:0, ROCKET_BARRIER:1, GALACTIC_TRAVELER:2, SPACESHIP_TIDYUP:3, ALIEN_CODES:4, HOUSE_PERSPECTIVE_TAKING:5, TRAIN_SEQUENCING:6});
exports.GameCommands = Object.freeze({START:0, CONTINUE:1, PAUSE:2, END:3, WAIT_FOR_RESPONSE:4, SKIP_RESPONSE:5});
exports.GameStates = Object.freeze({START: 0, IN_PROGRESS: 1, PAUSED: 2, USER_TIMEOUT: 3, END: 4, READY: 5, TUTORIAL: 6});

exports.RobotCommands = Object.freeze({SLEEP:0, WAKEUP:1, DO:2});
exports.RobotActions = Object.freeze({SOME_ACTION:0}); // TODO(Vadim): figure out what controls what actions we can perform

exports.ASSETS_PATH = '../../assets/'; // Relative to main.js in scripts/build/ dir

exports.UNEXPECTED_LEVEL = "Unexpected Error";

/*
 * Custom string format function
 */
if (!String.prototype.format) {
  String.prototype.format = function() {
    var args = arguments;
    return this.replace(/{(\d+)}/g, function(match, number) {
      return typeof args[number] != 'undefined'
        ? args[number]
        : match
      ;
    });
  };
}
