using UnityEngine;
using System.Collections;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using UnityEngine.UI;


namespace HouseGame
{
    public class BothHouses : MonoBehaviour
    {
        private MainGameController gameController;
        private ChildBuilder childbuilder;
        private RobotGuesser guess;
        private int level;

        private ConstructedHouse builtHouse;
        private GuessedHouse guessedHouse;

        private RandomHouse correct;
        private RandomHouse guessed;

        private ChildGuesser child;
        private ChildGuesserTutorial childTut;
        public int scene;

        // Use this for initialization
        void Start()
        {
            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();

            if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
            {
                guess = GameObject.Find("RobotGuessingManager").GetComponent<RobotGuesser>();
                childbuilder = GameObject.Find("ExplainerSceneManager").GetComponent<ChildBuilder>();
                guessedHouse = guess.finalHouse;
                builtHouse = childbuilder.explainerHouse;
                gameController.SendRobotUtterance("child-builder-compare-houses",false, -1, -1, -1);

                /*var cl = GameObject.Find("child_left");
                var rr = GameObject.Find("robot_right");

                cl.gameObject.GetComponent<Text>().enabled = true;
                rr.gameObject.GetComponent<Text>().enabled = true;*/
            }
            else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
            {
                if (gameController.tutorial)
                {
                    childTut = GameObject.Find("ChildGuessingManager").GetComponent<ChildGuesserTutorial>();
                    correct = childTut.chosenHouse;
                    guessed = childTut.submittedHouse;
                }
                else
                {
                    child = GameObject.Find("ChildGuessingManager").GetComponent<ChildGuesser>();
                    correct = child.chosenHouse;
                    guessed = child.submittedHouse;
                }

                gameController.SendRobotUtterance("robot-builder-compare-houses", false, -1, -1, -1);

                /*var cr = GameObject.Find("child_right");
                var rl = GameObject.Find("robot_left");

                cr.gameObject.GetComponent<Text>().enabled = true;
                rl.gameObject.GetComponent<Text>().enabled = true;*/

            }
            level = gameController.levelNumber;

            showOnScreen();
            
        }

        public void NewGame()
        {

            if (gameController.internalGameState == Constants.END_GAME)
            {
                gameController.SendRobotUtterance("end-game", false, -1, -1, -1);
                Logger.Log("GoingToQuit");
                //Application.Quit();
            } 
            //gameController.StartNextGame();
            gameController.levelNumber = gameController.saveLevel;
            level = gameController.saveLevel;
            if (gameController.tutorial)
            {
                gameController.tutorial = false;
            }
            if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
            {
                gameController.scene = Constants.CHILD_GUESSING_SCENE;
                Destroy(GameObject.Find("ExplainerSceneManager"));
                Destroy(GameObject.Find("RobotGuessingManager"));
                SceneManager.LoadScene("Child_guessing_L"+gameController.levelNumber);
            }
            else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
            {
                gameController.scene = Constants.ROBOT_GUESSING_SCENE;
                Destroy(GameObject.Find("ExplainerSceneManager"));
                Destroy(GameObject.Find("ChildGuessingManager"));
                SceneManager.LoadScene("Builder_L"+gameController.levelNumber);
            }

        }


        private void checkIfEqual(int gw,int bw,int gr,int br,int gp1,int gp2,int bp1,int bp2,int gd1,int gd2,int bd1,int bd2)
        {
            bool equal = false;
            int errors = 0;

            bool different_walls = false;
            bool different_roofs = false;
            bool different_door1 = false;
            bool different_door2 = false;
            bool different_plant1 = false;
            bool different_plant2 = false;
            bool inverted_doors = false;
            bool inverted_plants = false;

            if (bw != gw)
            {
                errors++;
                different_walls = true;
            }
            if (gr != br)
            {
                errors++;
                different_roofs = true;
            }
            if (bd1 != gd1)
            {
                errors++;
                different_door1 = true;
            }
            if (bd2 != gd2)
            {
                errors++;
                different_door2 = true;
            }
            if (bp1 != gp1)
            {
                errors++;
                different_plant1 = true;
            }
            if (bp2 != gp2)
            {
                errors++;
                different_plant2 = true;
            }
            /*if ((bp1 == gp2) && (bp2 == gp1))
            {
                errors++;
                inverted_plants = true;
            }
            if ((bd1 == gd2) && (bd2 == gd1))
            {
                errors++;
                inverted_doors = true;
            }*/

            if (!different_walls && !different_roofs && !different_door1 && !different_door2 && !different_plant1 && !different_plant2)
            {
                equal = true;
            }

            if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
            {
                if (equal)
                {
                    gameController.SendRobotUtterance("child-builder-correct-house", true, -1, -1, -1);
                }
                else
                {
                    bool asked = false;

                    if (inverted_doors)
                    {
                        gameController.SendRobotUtterance("child-builder-inverted-pieces", true, 2, -1, -1);
                        asked = true;
                    }
                    if ((inverted_plants) && (asked == false))
                    {
                        gameController.SendRobotUtterance("child-builder-inverted-pieces", true, 4, -1, -1);
                        asked = true;
                    }
                    if ((different_walls) && (asked == false))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 1, -1, -1);
                        asked = true;
                    }
                    if ((different_roofs) && (asked == false))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 3, -1, -1);
                        asked = true;
                    }
                    if ((different_door1) && (asked == false) && (level < 3))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 2, -1, -1);
                        asked = true;
                    }
                    if ((different_door1) && (asked == false) && (level > 2))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 5, -1, -1);
                        asked = true;
                    }
                    if ((different_door2) && (asked == false))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 6, -1, -1);
                        asked = true;
                    }
                    if ((different_plant1) && (asked == false) && (level < 4))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 4, -1, -1);
                        asked = true;
                    }
                    if ((different_plant1) && (asked == false) && (level > 3))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 7, -1, -1);
                        asked = true;
                    }
                    if ((different_plant2) && (asked == false))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 8, -1, -1);
                        asked = true;
                    }
                }
            }
            else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
            {
                if (equal)
                {
                    gameController.SendRobotUtterance("robot-builder-correct-house", true, -1, -1, -1);
                }
                else
                {
                    bool asked = false;

                    if (inverted_doors)
                    {
                        gameController.SendRobotUtterance("robot-builder-inverted-pieces", true, 2, -1, -1);
                        asked = true;
                    }
                    if ((inverted_plants) && (asked == false))
                    {
                        gameController.SendRobotUtterance("robot-builder-inverted-pieces", true, 4, -1, -1);
                        asked = true;
                    }
                    if ((different_walls) && (asked == false))
                    {
                        gameController.SendRobotUtterance("robot-builder-wrong-piece", true, 1, -1, -1);
                        asked = true;
                    }
                    if ((different_roofs) && (asked == false))
                    {
                        gameController.SendRobotUtterance("robot-builder-wrong-piece", true, 3, -1, -1);
                        asked = true;
                    }
                    if ((different_door1) && (asked == false) && (level < 3))
                    {
                        gameController.SendRobotUtterance("robot-builder-wrong-piece", true, 2, -1, -1);
                        asked = true;
                    }
                    if ((different_door1) && (asked == false) && (level > 2))
                    {
                        gameController.SendRobotUtterance("child-builder-wrong-piece", true, 5, -1, -1);
                        asked = true;
                    }
                    if ((different_door2) && (asked == false))
                    {
                        gameController.SendRobotUtterance("robot-builder-wrong-piece", true, 6, -1, -1);
                        asked = true;
                    }
                    if ((different_plant1) && (asked == false) && (level < 4))
                    {
                        gameController.SendRobotUtterance("robot-builder-wrong-piece", true, 4, -1, -1);
                        asked = true;
                    }
                    if ((different_plant1) && (asked == false) && (level > 3))
                    {
                        gameController.SendRobotUtterance("robot-builder-wrong-piece", true, 7, -1, -1);
                        asked = true;
                    }
                    if ((different_plant2) && (asked == false))
                    {
                        gameController.SendRobotUtterance("robot-builder-wrong-piece", true, 8, -1, -1);
                        asked = true;
                    }
                }
            }
            if (!gameController.tutorial)
            {
                performanceMetrics(errors);
            }
            if (gameController.internalGameState == Constants.END_GAME)
            {
                // send goobye message

                //Application.Quit();
            }
            gameController.SendRobotUtterance("restart", false, -1, -1, -1);
        }

        private void performanceMetrics(int errors)
        {
            int total = 1; ;
            if (level == 1)
                total = 3;
            if (level == 2)
                total = 4;
            if (level == 3)
                total = 5;
            if (level == 4)
                total = 6;

            // calculate the percentage of correct pieces
            float percentage = (float)(total - errors) / total;


            if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
            {
                if (gameController.performanceMetricsChildGuesser == 0)
                {
                    gameController.performanceMetricsChildGuesser = percentage;
                }
                else
                {
                    gameController.performanceMetricsChildGuesser = (gameController.performanceMetricsChildGuesser + percentage) / 2;
                    //averagePerformanceMetrics.Add("child-guesser-percentage-correct", percentage);
                }
            }
            else if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
            {
                if (gameController.performanceMetricsRobotGuesser == 0)
                {
                    gameController.performanceMetricsRobotGuesser = percentage;
                }
                else
                {
                    gameController.performanceMetricsRobotGuesser = (gameController.performanceMetricsRobotGuesser + percentage) / 2;
                    //averagePerformanceMetrics.Add("child-builder-percentage-correct", percentage);
                }
            }


            /*Dictionary<string, float> averagePerformanceMetrics = new Dictionary<string, float>();

            if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                averagePerformanceMetrics.Add("child-guesser-percentage-correct", percentage);
            else if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
            averagePerformanceMetrics.Add("child-builder-percentage-correct", percentage);*/

            // send to ROS the percentage of correct pieces 
            /*if (gameController.sendReceiveROSMessages)
            {
                gameController.clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishGameStateMsg(
                    Constants.STATE_ROSTOPIC,
                    Constants.GAME_STATE_END,
                    averagePerformanceMetrics));
            }

            Logger.Log(errors);*/

        }

        private void showOnScreen()
        {
            int gw = -1;
            int bw = -1;
            int gr = -1;
            int br = -1;
            int gp1 = -1;
            int gp2 = -1;
            int bp1 = -1;
            int bp2 = -1;
            int gd1 = -1;
            int gd2 = -1;
            int bd1 = -1;
            int bd2 = -1;
        
            if (level == 1)
            {
                // builder house ----------------------------------------------------------------------------
                var Bw1 = GameObject.Find("built_w1");
                var Bw2 = GameObject.Find("built_w2");
                var Bw3 = GameObject.Find("built_w3");
                var Bw4 = GameObject.Find("built_w4");
                var Bd1 = GameObject.Find("built_d1");
                var Br1 = GameObject.Find("built_r1");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {

                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bdoor = Instantiate(gameController.doorHousePieceOptions[builtHouse.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[builtHouse.roof]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);

                    bw = builtHouse.wall;
                    bd1 = builtHouse.door1;
                    br = builtHouse.roof;

                }
                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bdoor = Instantiate(gameController.doorHousePieceOptions[correct.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[correct.roof]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);

                    bw = correct.wall;
                    bd1 = correct.door1;
                    br = correct.roof;
                }


                // guessed house ----------------------------------------------------------------------------
                var Gw1 = GameObject.Find("guess_w1");
                var Gw2 = GameObject.Find("guess_w2");
                var Gw3 = GameObject.Find("guess_w3");
                var Gw4 = GameObject.Find("guess_w4");
                var Gd1 = GameObject.Find("guess_d1");
                var Gr1 = GameObject.Find("guess_r1");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessedHouse.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessedHouse.roof]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);

                    gw = guessedHouse.wall;
                    gd1 = guessedHouse.door1;
                    gr = guessedHouse.roof;
                }

                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessed.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessed.roof]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);

                    gw = guessed.wall;
                    gd1 = guessed.door1;
                    gr = guessed.roof;
                }

            }
            if (level == 2)
            {
                // builder house ----------------------------------------------------------------------------
                var Bw1 = GameObject.Find("built_w1");
                var Bw2 = GameObject.Find("built_w2");
                var Bw3 = GameObject.Find("built_w3");
                var Bw4 = GameObject.Find("built_w4");
                var Bd1 = GameObject.Find("built_d1");
                var Br1 = GameObject.Find("built_r1");
                var Bp1 = GameObject.Find("built_p1");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {
                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bdoor1 = Instantiate(gameController.doorHousePieceOptions[builtHouse.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[builtHouse.roof]);
                    GameObject Bplant1 = Instantiate(gameController.plantHousePieceOptions[builtHouse.plant1]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor1.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);
                    Bplant1.transform.SetParent(Bp1.transform, false);

                    bw = builtHouse.wall;
                    bd1 = builtHouse.door1;
                    br = builtHouse.roof;
                    bp1 = builtHouse.plant1;
                }

                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bdoor1 = Instantiate(gameController.doorHousePieceOptions[correct.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[correct.roof]);
                    GameObject Bplant1 = Instantiate(gameController.plantHousePieceOptions[correct.plant1]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor1.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);
                    Bplant1.transform.SetParent(Bp1.transform, false);

                    bw = correct.wall;
                    bd1 = correct.door1;
                    br = correct.roof;
                    bp1 = correct.plant1;
                }

                    // guessed house ----------------------------------------------------------------------------
                    var Gw1 = GameObject.Find("guess_w1");
                var Gw2 = GameObject.Find("guess_w2");
                var Gw3 = GameObject.Find("guess_w3");
                var Gw4 = GameObject.Find("guess_w4");
                var Gd1 = GameObject.Find("guess_d1");
                var Gr1 = GameObject.Find("guess_r1");
                var Gp1 = GameObject.Find("guess_p1");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessedHouse.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessedHouse.roof]);
                    GameObject Gplant1 = Instantiate(gameController.plantHousePieceOptions[guessedHouse.plant1]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);
                    Gplant1.transform.SetParent(Gp1.transform, false);

                    gw = guessedHouse.wall;
                    gd1 = guessedHouse.door1;
                    gr = guessedHouse.roof;
                    gp1 = guessedHouse.plant1;
                }

                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessed.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessed.roof]);
                    GameObject Gplant1 = Instantiate(gameController.plantHousePieceOptions[guessed.plant1]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);
                    Gplant1.transform.SetParent(Gp1.transform, false);

                    gw = guessed.wall;
                    gd1 = guessed.door1;
                    gr = guessed.roof;
                    gp1 = guessed.plant1;

                }
            }
            if (level == 3)
            {
                // builder house ----------------------------------------------------------------------------
                var Bw1 = GameObject.Find("built_w1");
                var Bw2 = GameObject.Find("built_w2");
                var Bw3 = GameObject.Find("built_w3");
                var Bw4 = GameObject.Find("built_w4");
                var Bd1 = GameObject.Find("built_d1");
                var Br1 = GameObject.Find("built_r1");
                var Bd2 = GameObject.Find("built_d2");
                var Bp1 = GameObject.Find("built_p1");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {
                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bdoor1 = Instantiate(gameController.doorHousePieceOptions[builtHouse.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[builtHouse.roof]);
                    GameObject Bdoor2 = Instantiate(gameController.doorHousePieceOptions[builtHouse.door2]);
                    GameObject Bplant1 = Instantiate(gameController.plantHousePieceOptions[builtHouse.plant1]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor1.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);
                    Bdoor2.transform.SetParent(Bd2.transform, false);
                    Bplant1.transform.SetParent(Bp1.transform, false);

                    bw = builtHouse.wall;
                    bd1 = builtHouse.door1;
                    br = builtHouse.roof;
                    bp1 = builtHouse.plant1;
                    bd2 = builtHouse.door2;
                }

                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bdoor1 = Instantiate(gameController.doorHousePieceOptions[correct.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[correct.roof]);
                    GameObject Bdoor2 = Instantiate(gameController.doorHousePieceOptions[correct.door2]);
                    GameObject Bplant1 = Instantiate(gameController.plantHousePieceOptions[correct.plant1]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor1.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);
                    Bdoor2.transform.SetParent(Bd2.transform, false);
                    Bplant1.transform.SetParent(Bp1.transform, false);

                    bw = correct.wall;
                    bd1 = correct.door1;
                    br = correct.roof;
                    bp1 = correct.plant1;
                    bd2 = correct.door2;
                }

                    // guessed house ----------------------------------------------------------------------------
                    var Gw1 = GameObject.Find("guess_w1");
                var Gw2 = GameObject.Find("guess_w2");
                var Gw3 = GameObject.Find("guess_w3");
                var Gw4 = GameObject.Find("guess_w4");
                var Gd1 = GameObject.Find("guess_d1");
                var Gr1 = GameObject.Find("guess_r1");
                var Gd2 = GameObject.Find("guess_d2");
                var Gp1 = GameObject.Find("guess_p1");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessedHouse.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessedHouse.roof]);
                    GameObject Gdoor2 = Instantiate(gameController.doorHousePieceOptions[guessedHouse.door2]);
                    GameObject Gplant1 = Instantiate(gameController.plantHousePieceOptions[guessedHouse.plant1]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);
                    Gdoor2.transform.SetParent(Gd2.transform, false);
                    Gplant1.transform.SetParent(Gp1.transform, false);

                    gw = guessedHouse.wall;
                    gd1 = guessedHouse.door1;
                    gr = guessedHouse.roof;
                    gp1 = guessedHouse.plant1;
                    gd2 = guessedHouse.door2;
                }

                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessed.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessed.roof]);
                    GameObject Gdoor2 = Instantiate(gameController.doorHousePieceOptions[guessed.door2]);
                    GameObject Gplant1 = Instantiate(gameController.plantHousePieceOptions[guessed.plant1]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);
                    Gdoor2.transform.SetParent(Gd2.transform, false);
                    Gplant1.transform.SetParent(Gp1.transform, false);

                    gw = guessed.wall;
                    gd1 = guessed.door1;
                    gr = guessed.roof;
                    gp1 = guessed.plant1;
                    gd2 = guessed.door2;
                }
            }
            if (level == 4)
            {
                // builder house ----------------------------------------------------------------------------
                var Bw1 = GameObject.Find("built_w1");
                var Bw2 = GameObject.Find("built_w2");
                var Bw3 = GameObject.Find("built_w3");
                var Bw4 = GameObject.Find("built_w4");
                var Bd1 = GameObject.Find("built_d1");
                var Br1 = GameObject.Find("built_r1");
                var Bd2 = GameObject.Find("built_d2");
                var Bp1 = GameObject.Find("built_p1");
                var Bp2 = GameObject.Find("built_p2");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {
                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[builtHouse.wall]);
                    GameObject Bdoor1 = Instantiate(gameController.doorHousePieceOptions[builtHouse.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[builtHouse.roof]);
                    GameObject Bdoor2 = Instantiate(gameController.doorHousePieceOptions[builtHouse.door2]);
                    GameObject Bplant1 = Instantiate(gameController.plantHousePieceOptions[builtHouse.plant1]);
                    GameObject Bplant2 = Instantiate(gameController.plantHousePieceOptions[builtHouse.plant2]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor1.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);
                    Bdoor2.transform.SetParent(Bd2.transform, false);
                    Bplant1.transform.SetParent(Bp1.transform, false);
                    Bplant2.transform.SetParent(Bp2.transform, false);

                    bw = builtHouse.wall;
                    bd1 = builtHouse.door1;
                    br = builtHouse.roof;
                    bp1 = builtHouse.plant1;
                    bd2 = builtHouse.door2;
                    bp2 = builtHouse.plant2;
                }

                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[correct.wall]);
                    GameObject Bdoor1 = Instantiate(gameController.doorHousePieceOptions[correct.door1]);
                    GameObject Broof = Instantiate(gameController.roofHousePieceOptions[correct.roof]);
                    GameObject Bdoor2 = Instantiate(gameController.doorHousePieceOptions[correct.door2]);
                    GameObject Bplant1 = Instantiate(gameController.plantHousePieceOptions[correct.plant1]);
                    GameObject Bplant2 = Instantiate(gameController.plantHousePieceOptions[correct.plant2]);

                    Bwall1.transform.SetParent(Bw1.transform, false);
                    Bwall2.transform.SetParent(Bw2.transform, false);
                    Bwall3.transform.SetParent(Bw3.transform, false);
                    Bwall4.transform.SetParent(Bw4.transform, false);
                    Bdoor1.transform.SetParent(Bd1.transform, false);
                    Broof.transform.SetParent(Br1.transform, false);
                    Bdoor2.transform.SetParent(Bd2.transform, false);
                    Bplant1.transform.SetParent(Bp1.transform, false);
                    Bplant2.transform.SetParent(Bp2.transform, false);

                    bw = correct.wall;
                    bd1 = correct.door1;
                    br = correct.roof;
                    bp1 = correct.plant1;
                    bd2 = correct.door2;
                    bp2 = correct.plant2;

                }

                    // guessed house ----------------------------------------------------------------------------
                    var Gw1 = GameObject.Find("guess_w1");
                var Gw2 = GameObject.Find("guess_w2");
                var Gw3 = GameObject.Find("guess_w3");
                var Gw4 = GameObject.Find("guess_w4");
                var Gd1 = GameObject.Find("guess_d1");
                var Gr1 = GameObject.Find("guess_r1");
                var Gd2 = GameObject.Find("guess_d2");
                var Gp1 = GameObject.Find("guess_p1");
                var Gp2 = GameObject.Find("guess_p2");

                if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessedHouse.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessedHouse.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessedHouse.roof]);
                    GameObject Gdoor2 = Instantiate(gameController.doorHousePieceOptions[guessedHouse.door2]);
                    GameObject Gplant1 = Instantiate(gameController.plantHousePieceOptions[guessedHouse.plant1]);
                    GameObject Gplant2 = Instantiate(gameController.plantHousePieceOptions[guessedHouse.plant2]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);
                    Gdoor2.transform.SetParent(Gd2.transform, false);
                    Gplant1.transform.SetParent(Gp1.transform, false);
                    Gplant2.transform.SetParent(Gp2.transform, false);

                    gw = guessedHouse.wall;
                    gd1 = guessedHouse.door1;
                    gr = guessedHouse.roof;
                    gp1 = guessedHouse.plant1;
                    gd2 = guessedHouse.door2;
                    gp2 = guessedHouse.plant2;
                }

                else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                {
                    GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[guessed.wall]);
                    GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[guessed.door1]);
                    GameObject Groof = Instantiate(gameController.roofHousePieceOptions[guessed.roof]);
                    GameObject Gdoor2 = Instantiate(gameController.doorHousePieceOptions[guessed.door2]);
                    GameObject Gplant1 = Instantiate(gameController.plantHousePieceOptions[guessed.plant1]);
                    GameObject Gplant2 = Instantiate(gameController.plantHousePieceOptions[guessed.plant2]);

                    Gwall1.transform.SetParent(Gw1.transform, false);
                    Gwall2.transform.SetParent(Gw2.transform, false);
                    Gwall3.transform.SetParent(Gw3.transform, false);
                    Gwall4.transform.SetParent(Gw4.transform, false);
                    Gdoor.transform.SetParent(Gd1.transform, false);
                    Groof.transform.SetParent(Gr1.transform, false);
                    Gdoor2.transform.SetParent(Gd2.transform, false);
                    Gplant1.transform.SetParent(Gp1.transform, false);
                    Gplant2.transform.SetParent(Gp2.transform, false);

                    gw = guessed.wall;
                    gd1 = guessed.door1;
                    gr = guessed.roof;
                    gp1 = guessed.plant1;
                    gd2 = guessed.door2;
                    gp2 = guessed.plant2;
                }

                
            }
            checkIfEqual(gw, bw, gr, br, gp1, gp2, bp1, bp2, gd1, gd2, bd1, bd2);
        }
        
    }
}
