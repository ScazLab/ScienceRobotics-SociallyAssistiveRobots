using UnityEngine;
using System.Collections;
using UnityEngine.SceneManagement;
using System.Collections.Generic;
using UnityEngine.UI;


namespace TrainGame
{
    public class BothHouses : MonoBehaviour
    {
        private MainGameController gameController;
        private ChildBuilder childbuilder;
        Dictionary<string, float> averagePerformanceMetrics = new Dictionary<string, float>();

        // Use this for initialization
        void Start()
        {
            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();
            childbuilder = GameObject.Find("ExplainerSceneManager").GetComponent<ChildBuilder>();

            // level = gameController.levelNumber;

            showOnScreen();
            CorrectTrain();
            
        }



        public void NewGame()
        {
            if (gameController.internalGameState == Constants.END_GAME)
            {
                gameController.SendRobotUtterance("end-game", false, -1, -1, -1,-1);
                gameController.sendPerformanceMetrics();
                Logger.Log("GoingToQuit");
                Application.Quit();
            }

            gameController.StartNextGame();
            if (gameController.tutorial)
            {
                gameController.tutorial = false;
            }
            //Destroy(GameObject.Find("ExplainerSceneManager"));
            Destroy(GameObject.Find("BothHousesManager"));
            //SceneManager.LoadScene("Builder_L1");
            childbuilder.lastPieceTypeSelected = Constants.NONE_SELECTED;
            childbuilder.step = 1;
            SceneManager.LoadScene("Start_menu");
            //Application.LoadLevel(gameController.previousLevel);

        }


        private void CorrectTrain()
        {
            int train_errors = 0;

            if (childbuilder.childTrain.front != childbuilder.robotTrain.front)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.body != childbuilder.robotTrain.body)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.cabin != childbuilder.robotTrain.cabin)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.back != childbuilder.robotTrain.back)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.wheel1 != childbuilder.robotTrain.wheel1)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.wheel2 != childbuilder.robotTrain.wheel2)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.wheel3 != childbuilder.robotTrain.wheel3)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.wheel4 != childbuilder.robotTrain.wheel4)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.smoke1 != childbuilder.robotTrain.smoke1)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.smoke2 != childbuilder.robotTrain.smoke2)
            {
                train_errors++;
            }
            if (childbuilder.childTrain.smoke3 != childbuilder.robotTrain.smoke3)
            {
                train_errors++;
            }

            if (train_errors == 0)
            {
                gameController.SendRobotUtterance("both-trains-right", false, -1, -1, -1, -1);
            }
            else
            {
                gameController.SendRobotUtterance("both-trains-wrong", false, -1, -1, -1, -1);
            }
            int order_errors = childbuilder.score;

           //gameController.SendRobotUtterance("child-builder-inverted-pieces", true, 2, -1, -1);
                        
            performanceMetrics(train_errors, order_errors);
            if (gameController.internalGameState == Constants.END_GAME)
            {
                // send goobye message
                gameController.SendRobotUtterance("end", false, -1, -1, -1, -1);
                Application.Quit();
            }
            gameController.SendRobotUtterance("another-game", false, -1, -1, -1,-1);
        }

        private void performanceMetrics(int train_errors,int order_errors)
        {

            if (gameController.performanceMetricsTrainErrors == 0 && gameController.performanceMetricsOrderErrors == 0)
            {
                gameController.performanceMetricsOrderErrors = order_errors;
                gameController.performanceMetricsTrainErrors = train_errors;
            }
            else
            {
                gameController.performanceMetricsTrainErrors = (gameController.performanceMetricsTrainErrors + train_errors) / 2;
                gameController.performanceMetricsOrderErrors = (gameController.performanceMetricsOrderErrors + train_errors) / 2;
            }

        }

        private void showOnScreen()
        {
                // child built train ----------------------------------------------------------------------------
                var cf = GameObject.Find("c_front");
                var cbd = GameObject.Find("c_body");
                var cc = GameObject.Find("c_cabin");
                var cb = GameObject.Find("c_back");
                var cw1 = GameObject.Find("c_wheel1");
                var cw2 = GameObject.Find("c_wheel2");
                var cw3 = GameObject.Find("c_wheel3");
                var cw4 = GameObject.Find("c_wheel4");
                var cs1 = GameObject.Find("c_smoke1");
                var cs2 = GameObject.Find("c_smoke2");
                var cs3 = GameObject.Find("c_smoke3");


                GameObject c_front = Instantiate(gameController.frontPieceOptions[childbuilder.childTrain.front]);
                GameObject c_body = Instantiate(gameController.bodyPieceOptions[childbuilder.childTrain.body]);
                GameObject c_cabin = Instantiate(gameController.cabinPieceOptions[childbuilder.childTrain.cabin]);
                GameObject c_back = Instantiate(gameController.backPieceOptions[childbuilder.childTrain.back]);
                GameObject c_wheel1 = Instantiate(gameController.wheelPieceOptions[childbuilder.childTrain.wheel1]);
                GameObject c_wheel2 = Instantiate(gameController.wheelPieceOptions[childbuilder.childTrain.wheel2]);
                GameObject c_wheel3 = Instantiate(gameController.wheelPieceOptions[childbuilder.childTrain.wheel3]);
                GameObject c_wheel4 = Instantiate(gameController.wheelPieceOptions[childbuilder.childTrain.wheel4]);
                GameObject c_smoke1 = Instantiate(gameController.smokePieceOptions[childbuilder.childTrain.smoke1]);
                GameObject c_smoke2 = Instantiate(gameController.smokePieceOptions[childbuilder.childTrain.smoke2]);
                GameObject c_smoke3 = Instantiate(gameController.smokePieceOptions[childbuilder.childTrain.smoke3]);

                c_front.transform.SetParent(cf.transform, false);
                c_body.transform.SetParent(cbd.transform, false);
                c_cabin.transform.SetParent(cc.transform, false);
                c_back.transform.SetParent(cb.transform, false);
                c_wheel1.transform.SetParent(cw1.transform, false);
                c_wheel2.transform.SetParent(cw2.transform, false);
                c_wheel3.transform.SetParent(cw3.transform, false);
                c_wheel4.transform.SetParent(cw4.transform, false);
                c_smoke1.transform.SetParent(cs1.transform, false);
                c_smoke2.transform.SetParent(cs2.transform, false);
                c_smoke3.transform.SetParent(cs3.transform, false);

            // robot train ----------------------------------------------------------------------------
            var rf = GameObject.Find("r_front");
            var rbd = GameObject.Find("r_body");
            var rc = GameObject.Find("r_cabin");
            var rb = GameObject.Find("r_back");
            var rw1 = GameObject.Find("r_wheel1");
            var rw2 = GameObject.Find("r_wheel2");
            var rw3 = GameObject.Find("r_wheel3");
            var rw4 = GameObject.Find("r_wheel4");
            var rs1 = GameObject.Find("r_smoke1");
            var rs2 = GameObject.Find("r_smoke2");
            var rs3 = GameObject.Find("r_smoke3");


            GameObject r_front = Instantiate(gameController.frontPieceOptions[childbuilder.robotTrain.front]);
            GameObject r_body = Instantiate(gameController.bodyPieceOptions[childbuilder.robotTrain.body]);
            GameObject r_cabin = Instantiate(gameController.cabinPieceOptions[childbuilder.robotTrain.cabin]);
            GameObject r_back = Instantiate(gameController.backPieceOptions[childbuilder.robotTrain.back]);
            GameObject r_wheel1 = Instantiate(gameController.wheelPieceOptions[childbuilder.robotTrain.wheel1]);
            GameObject r_wheel2 = Instantiate(gameController.wheelPieceOptions[childbuilder.robotTrain.wheel2]);
            GameObject r_wheel3 = Instantiate(gameController.wheelPieceOptions[childbuilder.robotTrain.wheel3]);
            GameObject r_wheel4 = Instantiate(gameController.wheelPieceOptions[childbuilder.robotTrain.wheel4]);
            GameObject r_smoke1 = Instantiate(gameController.smokePieceOptions[childbuilder.robotTrain.smoke1]);
            GameObject r_smoke2 = Instantiate(gameController.smokePieceOptions[childbuilder.robotTrain.smoke2]);
            GameObject r_smoke3 = Instantiate(gameController.smokePieceOptions[childbuilder.robotTrain.smoke3]);

            r_front.transform.SetParent(rf.transform, false);
            r_body.transform.SetParent(rbd.transform, false);
            r_cabin.transform.SetParent(rc.transform, false);
            r_back.transform.SetParent(rb.transform, false);
            r_wheel1.transform.SetParent(rw1.transform, false);
            r_wheel2.transform.SetParent(rw2.transform, false);
            r_wheel3.transform.SetParent(rw3.transform, false);
            r_wheel4.transform.SetParent(rw4.transform, false);
            r_smoke1.transform.SetParent(rs1.transform, false);
            r_smoke2.transform.SetParent(rs2.transform, false);
            r_smoke3.transform.SetParent(rs3.transform, false);



            //checkIfEqual(gw, bw, gr, br, gp1, gp2, bp1, bp2, gd1, gd2, bd1, bd2);
        }
        
    }
}
