using UnityEngine;
using System.Collections;
using UnityEngine.SceneManagement;


namespace HouseGame
{
    public class RobotGuesserTutorial : MonoBehaviour
    {

        private MainGameController gameController;
        int tutorial_step = 0;
        bool updated = false;
        public GameObject x;
        public GameObject right;
        public GameObject left;

        // Use this for initialization
        void Start()
        {
            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();


            var w1 = GameObject.Find("body1");
            var w2 = GameObject.Find("body2");
            var w3 = GameObject.Find("body3");
            var w4 = GameObject.Find("body4");
            var d1 = GameObject.Find("door1");
            var r1 = GameObject.Find("roof1");
            var d2 = GameObject.Find("door2");

            GameObject wall1 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall2 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall3 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall4 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject door = Instantiate(gameController.doorHousePieceOptions[0]);
            GameObject roof = Instantiate(gameController.roofHousePieceOptions[0]);
            GameObject door2 = Instantiate(gameController.doorHousePieceOptions[1]);

            wall1.transform.SetParent(w1.transform, false);
            wall2.transform.SetParent(w2.transform, false);
            wall3.transform.SetParent(w3.transform, false);
            wall4.transform.SetParent(w4.transform, false);
            door.transform.SetParent(d1.transform, false);
            roof.transform.SetParent(r1.transform, false);
            door2.transform.SetParent(d2.transform, false);

            gameController.SendRobotUtterance("robot-guessing-tutorial", true, -1, -1, -1);
        }

        // Update is called once per frame
        void Update()
        {
            
            if ((tutorial_step == 0) && (!updated))
            {
                gameController.SendRobotUtterance("robot-guessing-tutorial-1", true, -1, -1, -1);
                GameObject question = Instantiate(gameController.wallHousePieceOptions[0]);
                var canvas = GameObject.Find("Guess_Panel");
                question.transform.SetParent(canvas.transform, false);
                GameObject y = GameObject.Find("circle-yes");
                y.gameObject.GetComponent<Renderer>().enabled = true;
                updated = true;
            }
            if ((tutorial_step == 1) && (!updated))
            {
                gameController.SendRobotUtterance("robot-guessing-tutorial-2", true, -1, -1, -1);
                GameObject question = Instantiate(gameController.doorHousePieceOptions[1]);
                var canvas = GameObject.Find("Guess_Panel");
                question.transform.SetParent(canvas.transform, false);
                GameObject right_arrow = Instantiate(right);
                right_arrow.transform.SetParent(canvas.transform, false);
                GameObject y = GameObject.Find("circle-yes");
                y.gameObject.GetComponent<Renderer>().enabled = true;
                updated = true;
            }
            if ((tutorial_step == 2) && (!updated))
            {
                gameController.SendRobotUtterance("robot-guessing-tutorial-3", true, -1, -1, -1);
                GameObject y = GameObject.Find("circle-yes");
                y.gameObject.GetComponent<Renderer>().enabled = false;
                GameObject question = Instantiate(gameController.roofHousePieceOptions[1]);
                var canvas = GameObject.Find("Guess_Panel");
                question.transform.SetParent(canvas.transform, false);
                GameObject n = GameObject.Find("circle-no");
                n.gameObject.GetComponent<Renderer>().enabled = true;
                updated = true;

            }
            if ((tutorial_step == 3) && (!updated))
            {
                gameController.SendRobotUtterance("robot-guessing-tutorial-4", true, -1, -1, -1);
                GameObject n = GameObject.Find("circle-no");
                n.gameObject.GetComponent<Renderer>().enabled = false;
                GameObject question = Instantiate(gameController.colorHousePieceOptions[0]);
                var canvas = GameObject.Find("Guess_Panel");
                question.transform.SetParent(canvas.transform, false);
                GameObject y = GameObject.Find("circle-yes");
                y.gameObject.GetComponent<Renderer>().enabled = true;
                updated = true;
            }
            if ((tutorial_step == 4) && (!updated))
            {
                updated = true;
                SceneManager.LoadScene("Both_houses_tutorial");
            }
        }


        public void UpdatePossibleHouses(int ispresent)
        {
            if (tutorial_step == 0)
            {
                GameObject question = Instantiate(gameController.wallHousePieceOptions[0]);
                string panelspot = "g1";
                var canvas = GameObject.Find(panelspot);
                question.transform.SetParent(canvas.transform, false);
                if (ispresent == 0)
                {
                    GameObject x1 = Instantiate(x);
                    x1.transform.SetParent(canvas.transform, false);
                }
            }
            if (tutorial_step == 1)
            {
                GameObject question = Instantiate(gameController.doorHousePieceOptions[1]);
                string panelspot = "g2";
                var canvas = GameObject.Find(panelspot);
                question.transform.SetParent(canvas.transform, false);
                if (ispresent == 0)
                {
                    GameObject x1 = Instantiate(x);
                    x1.transform.SetParent(canvas.transform, false);
                }
                GameObject right_arrow = Instantiate(right);
                right_arrow.transform.SetParent(canvas.transform, false);
            }
            if (tutorial_step == 2)
            {
                GameObject question = Instantiate(gameController.roofHousePieceOptions[1]);
                string panelspot = "g3";
                var canvas = GameObject.Find(panelspot);
                question.transform.SetParent(canvas.transform, false);
                if (ispresent == 0)
                {
                    GameObject x1 = Instantiate(x);
                    x1.transform.SetParent(canvas.transform, false);
                }

            }
            if (tutorial_step == 3)
            {
                GameObject question = Instantiate(gameController.colorHousePieceOptions[0]);
                string panelspot = "g4";
                var canvas = GameObject.Find(panelspot);
                question.transform.SetParent(canvas.transform, false);
                if (ispresent == 0)
                {
                    GameObject x1 = Instantiate(x);
                    x1.transform.SetParent(canvas.transform, false);
                }
            }
            tutorial_step++;
            updated = false;

        }
    }
}
