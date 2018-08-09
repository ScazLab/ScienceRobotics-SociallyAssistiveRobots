using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

namespace HouseGame
{
    public class ExitTutorial : MonoBehaviour
    {
        public Button exitButton;
        private BothHouses bh;
        private MainGameController gameController;


        void Start()
        {
            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();
            Button exit = exitButton.GetComponent<Button>();
            exit.onClick.AddListener(Clicked);


        }

        public void Clicked()
        {
            gameController.levelNumber = gameController.saveLevel;
            gameController.tutorial = false;
            Destroy(GameObject.Find("Both_houses_tutorial"));
            Destroy(GameObject.Find("Builder_tutorial"));
            Destroy(GameObject.Find("Child_guessing_tutorial"));
            //Destroy(GameObject.Find(""));
            //SceneManager.LoadScene("Builder_L" + gameController.levelNumber);
            if (gameController.tutorial)
            {
                gameController.tutorial = false;
            }
            if (gameController.scene == Constants.ROBOT_GUESSING_SCENE)
            {
                gameController.scene = Constants.CHILD_GUESSING_SCENE;
                Destroy(GameObject.Find("ExplainerSceneManager"));
                Destroy(GameObject.Find("RobotGuessingManager"));
                SceneManager.LoadScene("Child_guessing_L" + gameController.levelNumber);
            }
            else if (gameController.scene == Constants.CHILD_GUESSING_SCENE)
            {
                gameController.scene = Constants.ROBOT_GUESSING_SCENE;
                Destroy(GameObject.Find("ExplainerSceneManager"));
                Destroy(GameObject.Find("ChildGuessingManager"));
                SceneManager.LoadScene("Builder_L" + gameController.levelNumber);
            }
        }

    }
}





