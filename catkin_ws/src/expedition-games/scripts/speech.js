var CONST = require('./constants.js');

exports.SUCCESS = [
    {
        msg: "<bounce,nb> Good job!",
        id: "congratulatory_1"
    },
    {
        msg: "<smile,nb> Great work!",
        id: "congratulatory_2"
    },
    {
        msg: "<smile,nb> Thanks for helping me!",
        id: "congratulatory_3"
    },
    {
        msg: "<excited,nb> Fantastic!",
        id: "congratulatory_4"
    },
    {
        msg: "<excited,nb> Excellent!",
        id: "congratulatory_5"
    },
    {
        msg: "<smile,nb> You're doing great!",
        id: "congratulatory_6"
    },
    {
        msg: "<bounce,nb> Awesome!",
        id: "congratulatory_7"
    },
    {
        msg: "<excited,nb> Perfect!",
        id: "congratulatory_8"
    },
    {
        msg: "<smile,nb> Amazing!",
        id: "congratulatory_9"
    },
    {
        msg: "<excited,nb> Super!",
        id: "congratulatory_10"
    },
];

exports.GENERAL_FEEDBACK = [
    {
        msg: "Try again!",
        id: "incorrect_1"
    },
    {
        msg: "Not quite. Try again!",
        id: "incorrect_2"
    },
    {
        msg: "Close, but not quite. Try again!",
        id: "incorrect_3"
    },
    {
        msg: "Well, I don't think that's correct. Let's try again!",
        id: "incorrect_4"
    },
    {
        msg: "Whoops! That's not right. How about we try again.",
        id: "incorrect_5"
    },
];

exports.GAMES = {
    GALACTIC_TRAVELER: {
        1: {
            instructions: [
                {
                    msg: "I want to bring moon rocks home <smile,nb> to show my friends! Will you help me pack some? <bounce,nb> Put {0} moon rock into the box!",
                    id: "game1_act1_inst0_{0}"
                },
            ],
            feedback: [
                {
                    msg: "Almost! We need {1} moon rocks, {0} total. Try again!",
                    id: "game1_act1_feedback1_{1}_{0}"
                },
                {
                    msg: "We are missing a few moon rocks or have some extra. Let's fix that.",
                    id: "game1_act1_feedback3"
                },
                {
                    msg: "We are missing moon rocks or we have too many. Try to have {1} moonrocks.",
                    id: "game1_act1_feedback4_{1}"
                },
                {
                    msg: "I think we didn't pack the right number of moon rocks, we need {0}. Let's try again.",
                    id: "game1_act1_feedback5_{0}"
                },
                {
                    msg: "We don't have the right number of moon rocks. Let's try fixing that with {1} moon rocks.",
                    id: "game1_act1_feedback6_{1}"
                },
                {
                    msg: "I think we need to fix the number of moom rocks by having {1}.",
                    id: "game1_act1_feedback7_{1}"
                },
            ]
        },
        2: {
            instructions: [
                {
                    msg: "It's <concerned,nb> a long trip home! So I need to <excited,nb> fully charge my spaceship! What number do you see? Place {0} energy {1} into the battery pack to charge the spaceship.",
                    id: "game1_act2_inst1_{0}"
                },
                {
                    msg: "<calm,nb> What number do <laugh,nb> you see? Put {0} energy {1} into the battery pack.",
                    id: "game1_act2_inst2_{0}"
                },
                {
                    msg: "<speak,nb> Put {0} energy {1} into the battery pack to charge the spaceship. <bounce,nb>",
                    id: "game1_act2_inst3_{0}"
                },
                {
                    msg: "<speak,nb> Place {0} energy {1} into the <laugh,nb> battery pack. ",
                    id: "game1_act2_inst4_{0}"
                },
                {
                    msg: "<scared,nb> Help me charge <speak,nb> my spaceship! Put {0} space {1} into the battery pack.",
                    id: "game1_act2_inst5_{0}"
                },
            ],
            feedback: [
                {
                    msg: "Close, but we need {1} energy crystals.",
                    id: "game1_act2_feedback1_{1}"
                },
                {
                    msg: "We don't have the right number of energy crystals, we need {0}.",
                    id: "game1_act2_feedback2_{0}"
                },
                {
                    msg: "We have the wrong number of crystals, let's have {1}",
                    id: "game1_act2_feedback3_{1}"
                },
            ]
        },
        3: {
            instructions: [
                {
                    msg: "I need to <laugh,nb> show my spaceship where to go! <smile,nb> Will you help me navigate? <speak,nb> Choose the galaxy that has {1} stars.",
                    id: "game1_act3_inst0_{1}"
                },
                {
                    msg: "Choose the galaxy that has {1} stars. <laugh,nb>",
                    id: "game1_act3_inst{1}"
                },
            ],
            feedback: [
                {
                    msg: "Whoops! That's not the right galaxy. Try again.",
                    id: "game1_act3_feedback1"
                },
                {
                    msg: "Uh oh. I don't think that galaxy has {1} stars in it. Try again.",
                    id: "game1_act3_feedback2_{1}"
                },
                {
                    msg: "I don't think that's the correct galaxy. How about we try again!",
                    id: "game1_act3_feedback3"
                },

                {
                    msg: "That doesn't look right! Choose the galaxy with {1} stars in it.",
                    id: "game1_act3_feedback4_{1}"
                }
            ]
        },
        4: {
            instructions: [
                {
                    msg: "I have to <laugh,nb> show my spaceship how to go home <calm,nb>. Will you help me navigate? Choose the planet with the number {0}.",
                    id: "game1_act4_inst5_{0}"
                },
                {
                    msg: "Choose the planet <laugh,nb> with the number {0}.",
                    id: "game1_act4_inst1_{0}"
                },
            ],
            feedback: [
                {
                    msg: "That's not the correct planet. Let's try again.",
                    id: "game1_act4_feedback1"
                },
                {
                    msg: "That planet isn't the right one. We want the planet with number {0}. Try again.",
                    id: "game1_act4_feedback2_{0}"
                },
            ]
        },
        5: {
            instructions :[
                {
                    msg: "Meet <excited,nb> Ooki and Yana! They're my space <speak,nb> pets! They eat <bounce,nb> stardust! Will you <laugh,nb> help me feed them? Feed Ooki and Yana the same amount <laugh,nb> of stardust.",
                    id: "game1_act5_inst0"
                },
                {
                    msg: "<calm,nb> Will you help me feed <laugh,nb> Ooki and <speak,nb> Yana? Feed them the same amount of stardust.",
                    id: "game1_act5_inst1"
                },
                {
                    msg: "<calm,nb> Feed my space pets the same amount of stardust.",
                    id: "game1_act5_inst2"
                },
                {
                    msg: "<calm,nb> Help me feed Ooki and Yana <laugh,nb> the same amount of stardust.",
                    id: "game1_act5_inst3"
                },
            ],
            feedback: [
                {
                    msg: "Whoops! We gave Ooki and Yana different amounts of food. Try again!",
                    id: "game1_act5_feedback1"
                },
                {
                    msg: "Ooki and Yana don't have the same amount of stardust. Try again!",
                    id: "game1_act5_feedback2"
                },
                {
                    msg: "My space pets need to eat the same amount of stardust. Try again.",
                    id: "game1_act5_feedback3"
                },
            ]
        }
    },
    SPACESHIP_TIDYUP: {
        1: {
            instructions: [
                {
                    msg: "<laugh,nb> My space pets also need to go home! Let's make <concerned,nb> sure we get all of them onboard! Move them into the spaceship in order from <bounce,nb> 1 to {0}. Count them out with me!",
                    id: "game2_act1_inst0_{0}"
                },
                {
                    msg: "<speak,nb> Move my space pets <speak,nb> into the spaceship <laugh,nb> in order <speak,nb> from 1 to {0}.",
                    id: "game2_act1_inst{0}"
                },
            ],
            feedback: []
        },
        2: {
            instructions: [
                {
                    msg: "<speak,nb> Help me pack my moonrocks!",
                    id: "game2_act2_inst0_1"
                },
                {
                    msg: "<bounce,nb> Sort the moon rocks by color into different boxes!",
                    id: "game2_act2_inst0_2"
                },
            ],
            feedback: []
        },
        3: {
            instructions: [
                {
                    msg: "<bounce,nb> I've collected items from all over outer space. Will you help me <speak,nb> organize them?",
                    id: "game2_act3_inst0"
                },
                {
                    msg: "<calm,nb> Move each item into its proper box. <speak,nb> For example, put all the <laugh,nb> moon rocks into the box that is labeled moon rocks!",
                    id: "game2_act3_inst1"
                },
            ],
            feedback: []
        },
        4: {
            instructions: [
                {
                    msg: "<laugh,nb> Help me <speak,nb> match the pattern you see!",
                    id: "game2_act4_inst0"
                },
                {
                    msg: "<calm,nb> Copy the pattern by moving the same <speak,nb> moonrocks, <laugh,nb> energy crystals, <speak,nb> and stars into the box!",
                    id: "game2_act4_inst1"
                },
            ],
            feedback: []
        },
        5: {
            instructions: [
                {
                    msg: "<laugh,nb> Help me <speak,nb> finish the pattern you see!",
                    id: "game2_act5_inst0"
                },
                {
                    msg: "<speak,nb> Finish the pattern by moving the right <laugh,nb> moon rock, <speak,nb> energy crystal, or star into the box!",
                    id: "game2_act5_inst1"
                },
            ],
            feedback: []
        },
    },
    ALIEN_CODES: {
        1: {
            instructions: [
                {
                    msg: "My alien friends sent you a code. I need your help to translate it. Please describe the code you see.",
                    id: "A1Intro"
                }
            ],
            feedback: []
        },
        2: {
            instructions: [
                {
                    msg: "I’m having trouble finishing the alien code. Will you tell me what I need to complete the code?",
                    id: "A2Intro"
                }
            ],
            feedback: []
        },
        3: {
            instructions: [
                {
                    msg: "My space pets, Ooki and Yana, are not feeling well! We have to give them pills to make them feel better. What pills do I need?",
                    id: "A3Intro"
                }
            ],
            feedback: []
        },
        4: {
            instructions: [
                {
                    msg: "I need to get into my spaceship, but it is locked! Help me unlock the spaceship. What code will open the door?",
                    id: "A4Intro"
                }
            ],
            feedback: []
        },
        5: {
            instructions: [
                {
                    msg: "Will you help me decode the alien messages? Match each object to the words you see.",
                    id: "A5Intro"
                }
            ],
            feedback: []
        },
    },
};

exports.TUTORIALS = {
    GALACTIC_TRAVELER: [
        {
            msg: "For these games, I will be a robot space explorer! I'm visiting all the planets and the stars of the universe! But I'm stuck here on Earth and can't get back home. I need your help so I can go back and tell all my friends about space! Will you help me?",
            id: "gt_tutorial_1",
            duration: 10
        },
        {
            msg: "This is my spaceship. Want to see how it works?",
            id: "gt_tutorial_2",
            duration: 3
        },
        {
            msg: "This is where I control my spaceship. And this is where you can help me!",
            id: "gt_tutorial_3",
            duration: 4
        },
        {
            msg: "To make my spaceship do something, I move stuff into the box, then I press this green button when I'm done!",
            id: "gt_tutorial_4",
            duration: 5
        },
        {
            msg: "Now you try! Use your finger to drag the star into the box. When you are done, press the green finish button.",
            id: "gt_tutorial_5",
            duration: 5
        },
        {
            msg: "Sometimes I make mistakes and need to remove stuff from the box. To do that, I drag stuff out of the box.",
            id: "gt_tutorial_6",
            duration: 5
        },
        {
            msg: "Can you try to remove the star? Use your finger to drag the star out of the box. Then press the finish button.",
            id: "gt_tutorial_7",
            duration: 5
        },
        {
            msg: "Perfect. You're going to be a great partner!",
            id: "gt_tutorial_8",
            duration: 3
        }
    ],
    SPACESHIP_TIDYUP: [
        {
            msg: "For these games, we'll be space explorers! However, I'm having lots of trouble on my spaceship. I'm going to need your help to tidy up my ship so that I can make it back home. Do you want to help me?",
            id: "st_tutorial_0",
            duration: 10
        },
        {
            msg: "This is my spaceship. Want to see how it works?",
            id: "st_tutorial_1",
            duration: 5
        },
        {
            msg: "This is how I control my spaceship. This is where you can help me!",
            id: "st_tutorial_2",
            duration: 5
        },
        {
            msg: "To make my spaceship do something, I move stuff into the correct box. But I need to make sure the stuff matches the label on the box. When I'm done, I press this green button!",
            id: "st_tutorial_3",
            duration: 10
        },
        {
            msg: "Now you try! Use your finger to drag the star into the box labeled with a star. When you are done, press the green finish button.",
            id: "st_tutorial_4",
            duration: 10
        },
        {
            msg: "Sometimes I make mistakes and need to remove stuff from the box. To do that, I drag stuff out of the box.",
            id: "st_tutorial_5",
            duration: 5
        },
        {
            msg: "Can you try to remove the star? Use your finger to drag the star out of the box. Then press the finish button.",
            id: "st_tutorial_6",
            duration: 5
        },
        {
            msg: "You're doing great! I can't wait to play for real!",
            id: "st_tutorial_7",
            duration: 7
        }
    ],
    ALIEN_CODES: [
        {
            msg: "During these next games, I will be a robot space explorer! I've been traveling space for a long time now. I've received lots of messages on my journey, but I cannot understand them. I need your help understanding these messages. Are you ready to learn how you can help me?",
            id: "ac_tutorial_0",
            duration: 10
        },
        {
            msg: "This is my spaceship. Want to see how it works?",
            id: "ac_tutorial_1",
            duration: 7
        },
        {
            msg: "This is where I control my ship and where you can help me!",
            id: "ac_tutorial_2",
            duration: 7
        },
        {
            msg: "To play with my friends, I drag stuff into the empty box. Then I press the green button with an arrow when I'm done.",
            id: "ac_tutorial_3",
            duration: 10
        },
        {
            msg: "Are you ready to try? Use your finger to drag the space crystal into the empty box. Then, press the green arrow button to finish!",
            id: "ac_tutorial_4",
            duration: 7
        },
        {
            msg: "Sometimes I make mistakes and need to remove stuff from the box. To do that, I press the restart button on the left.",
            id: "ac_tutorial_5",
            duration: 6
        },
        {
            msg: "Can you try to remove the item? Use your finger to touch the reset button.",
            id: "ac_tutorial_6",
            duration: 5
        },
        {
            msg: "Well done! You're doing great!!",
            id: "ac_tutorial_7",
            duration: 5
        }
    ]
};

exports.generateSpeechScripts = function() {
    var results = "";

    for (let one of this.SUCCESS.concat(this.GENERAL_FEEDBACK)) {
        results += "[" + one.id + "]" + one.msg + "\n";
    }

    var strings = ["crystals", "crystal", "more", "fewer"];
    for (let key in this.GAMES) {
        for (let activity in this.GAMES[key]) {
            for (let item in this.GAMES[key][activity]) {
                var messages = this.GAMES[key][activity][item];
                for (let message of messages) {
                    for (var i = 0; i <= (message.id.indexOf("{0}") == -1 ? 0: 10); i++) {
                        for (var j = 0; j < (message.id.indexOf("{1}") == -1 ? 1: strings.length); j++) {
                            var string = strings[j];
                            var str = ("[" + message.id + "]" + message.msg + "\n");
                            if (str.length > 3)
                                results += str.format(i, string);
                        }
                    }
                }
            }
        }
    }
    return results;
}

// console.log(exports.generateSpeechScripts());
