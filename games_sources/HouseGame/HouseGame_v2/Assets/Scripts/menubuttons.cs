using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Collections.Generic;

namespace HouseGame
{
    public class menubuttons : MonoBehaviour
    {
        public Button startButton;
        public Button tutorialButton;
        private BothHouses bh;
        private MainGameController gameController;

        void Awake()
        {

            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();
        }

        void Start()
        {
            //bh = GameObject.Find("BothHousesManager").GetComponent<BothHouses>();
            Button sta = startButton.GetComponent<Button>();
            Button tut = tutorialButton.GetComponent<Button>();
            sta.onClick.AddListener(Game_click);
            tut.onClick.AddListener(Tutorial_click);


        }

        public void Game_click()
        {
            
                    if (gameController.levelNumber == 1)
                    {
                        if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-guesser-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Child_guessing_L1");
                        }
                        else if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-builder-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Builder_L1");
                        }
                    }
                    if (gameController.levelNumber == 2)
                    {
                        if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-guesser-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Child_guessing_L2");
                        }
                        else if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-builder-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Builder_L2");
                        }
                    }
                    if (gameController.levelNumber == 3)
                    {
                        if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-guesser-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Child_guessing_L3");
                        }
                        else if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-builder-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Builder_L3");
                        }
                    }
                    if (gameController.levelNumber == 4)
                    {
                        if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-guesser-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Child_guessing_L4");
                        }
                        else if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
                        {
                            gameController.SendRobotUtterance("child-builder-start", true, -1, -1, -1);
                            SceneManager.LoadScene("Builder_L4");
                        }
                    }

        }

        public void Tutorial_click()
        {
            //gameController.clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishGameStateMsg(Constants.STATE_ROSTOPIC, Constants.TUTORIAL, new Dictionary<string, float>()));
            gameController.internalGameState = Constants.TUTORIAL;
            gameController.tutorial = true;
            gameController.saveLevel = gameController.levelNumber;
            gameController.levelNumber = 1;
            if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
            {
                gameController.SendRobotUtterance("child-guesser-start", true, -1, -1, -1);
                SceneManager.LoadScene("Child_guessing_L1");
            }
            else if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
            {
                gameController.SendRobotUtterance("child-builder-start", true, -1, -1, -1);
                SceneManager.LoadScene("Builder_Tutorial");
            }
        }

    }
}


