using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

namespace TrainGame
{
    public class menu : MonoBehaviour
    {
        public Button startButton;
        public Button tutorialButton;
        //private BothHouses bh;
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
            //gameController.SendRobotUtterance("game-start", false, -1,-1,-1,-1);


        }

        public void Game_click()
        {
            SceneManager.LoadScene("Builder_L1");
        }

        public void Tutorial_click()
        {
            gameController.internalGameState = Constants.TUTORIAL;
            gameController.tutorial = true;
            gameController.saveLevel = gameController.levelNumber;
            gameController.levelNumber = 1;
            SceneManager.LoadScene("Builder_L1");
            //SceneManager.LoadScene("Builder_Tutorial");
        }

    }
}


