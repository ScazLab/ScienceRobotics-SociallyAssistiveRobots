﻿using UnityEngine;
using UnityEngine.UI;
using System.Collections;

namespace HouseGame
{
    public class IsPresentButton : MonoBehaviour
    {
        public Button yourButton;
        private RobotGuesser rb;

        void Start()
        {
            Button btn = yourButton.GetComponent<Button>();
            btn.onClick.AddListener(Clicked);
            rb = GameObject.Find("RobotGuessingManager").GetComponent<RobotGuesser>();
        }

        public void Clicked()
        {
            rb.UpdatePossibleHouses(1);
            //Logger.Log("PRESSED BUTTON");
        }

    }
}
