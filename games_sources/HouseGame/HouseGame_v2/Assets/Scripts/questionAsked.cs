using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System;

namespace HouseGame
{
    public class questionAsked : MonoBehaviour
    {
        public Button q;
        private ChildGuesser child_guesser;

        void Start()
        {
            child_guesser = GameObject.Find("ChildGuessingManager").GetComponent <ChildGuesser>();
            Button btn = q.GetComponent<Button>();
            btn.onClick.AddListener(Clicked);
        }

        public void Clicked()
        {
            string id_name = q.name.Remove(0, 1);
            int id = Int32.Parse(id_name);
            child_guesser.AnswerQuestion(id-1);
        }
    }
}
