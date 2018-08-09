using UnityEngine;
using System.Collections;
using UnityEngine.SceneManagement;
using System.Collections.Generic;


namespace HouseGame
{
    public class BothHousesTutorial : MonoBehaviour
    {
        private MainGameController gameController;
        public int scene;

        // Use this for initialization
        void Start()
        {
            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();
            showOnScreen();

        }

        public void NewGame()
        {
                SceneManager.LoadScene("Child_guessing_tutorial");
        }

        private void showOnScreen()
        {

                // builder house ----------------------------------------------------------------------------
                var Bw1 = GameObject.Find("built_w1");
                var Bw2 = GameObject.Find("built_w2");
                var Bw3 = GameObject.Find("built_w3");
                var Bw4 = GameObject.Find("built_w4");
                var Bd1 = GameObject.Find("built_d1");
                var Br1 = GameObject.Find("built_r1");
                var Bd2 = GameObject.Find("built_d2");

                GameObject Bwall1 = Instantiate(gameController.wallHousePieceOptions[0]);
                GameObject Bwall2 = Instantiate(gameController.wallHousePieceOptions[0]);
                GameObject Bwall3 = Instantiate(gameController.wallHousePieceOptions[0]);
                GameObject Bwall4 = Instantiate(gameController.wallHousePieceOptions[0]);
                GameObject Bdoor1 = Instantiate(gameController.doorHousePieceOptions[0]);
                GameObject Broof = Instantiate(gameController.roofHousePieceOptions[0]);
                GameObject Bdoor2 = Instantiate(gameController.doorHousePieceOptions[1]);

                Bwall1.transform.SetParent(Bw1.transform, false);
                Bwall2.transform.SetParent(Bw2.transform, false);
                Bwall3.transform.SetParent(Bw3.transform, false);
                Bwall4.transform.SetParent(Bw4.transform, false);
                Bdoor1.transform.SetParent(Bd1.transform, false);
                Broof.transform.SetParent(Br1.transform, false);
                Bdoor2.transform.SetParent(Bd2.transform, false);

                // guessed house ----------------------------------------------------------------------------
                var Gw1 = GameObject.Find("guess_w1");
                var Gw2 = GameObject.Find("guess_w2");
                var Gw3 = GameObject.Find("guess_w3");
                var Gw4 = GameObject.Find("guess_w4");
                var Gd1 = GameObject.Find("guess_d1");
                var Gr1 = GameObject.Find("guess_r1");
                var Gd2 = GameObject.Find("guess_d2");

                GameObject Gwall1 = Instantiate(gameController.wallHousePieceOptions[1]);
                GameObject Gwall2 = Instantiate(gameController.wallHousePieceOptions[1]);
                GameObject Gwall3 = Instantiate(gameController.wallHousePieceOptions[1]);
                GameObject Gwall4 = Instantiate(gameController.wallHousePieceOptions[1]);
                GameObject Gdoor = Instantiate(gameController.doorHousePieceOptions[0]);
                GameObject Groof = Instantiate(gameController.roofHousePieceOptions[0]);
                GameObject Gdoor2 = Instantiate(gameController.doorHousePieceOptions[1]);

                Gwall1.transform.SetParent(Gw1.transform, false);
                Gwall2.transform.SetParent(Gw2.transform, false);
                Gwall3.transform.SetParent(Gw3.transform, false);
                Gwall4.transform.SetParent(Gw4.transform, false);
                Gdoor.transform.SetParent(Gd1.transform, false);
                Groof.transform.SetParent(Gr1.transform, false);
                Gdoor2.transform.SetParent(Gd2.transform, false);

            gameController.SendRobotUtterance("robot-both-houses-tutorial-1", true, -1, -1, -1);
        }

    }
}