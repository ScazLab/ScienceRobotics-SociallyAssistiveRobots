using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System;

namespace HouseGame
{
    public class GuessHouse : MonoBehaviour
    {
        public Button yourButton;
        private ChildGuesser child_guesser;
        //private MainGameController gameController;
        //private ChildGuesserTutorial child_guesser_tut;

        // Use this for initialization
        void Start()
        {
            //gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();

                Button btn = yourButton.GetComponent<Button>();
                btn.onClick.AddListener(Clicked);


        }

        // Update is called once per frame
        public void Clicked()
        {
            /*
                if (yourButton.name == "final")
                {
                    child_guesser.SubmitFinalAnswer();
                }
                else
                {
                    child_guesser.xOrCheckHouse(yourButton.name);
                    //xOrCheckHouse(yourButton.name);
                }
                */
        }
    }
}
