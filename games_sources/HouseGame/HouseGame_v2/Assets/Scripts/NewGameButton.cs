using UnityEngine;
using System.Collections;
using UnityEngine.UI;

namespace HouseGame
{
    public class NewGameButton : MonoBehaviour
    {
        public Button yourButton;
        private BothHouses bh;


        void Start()
        {
            bh = GameObject.Find("BothHousesManager").GetComponent<BothHouses>();
            Button btn = yourButton.GetComponent<Button>();
            btn.onClick.AddListener(Clicked);

            
        }

        public void Clicked()
        {
            bh.NewGame();
            //Logger.Log("PRESSED BUTTON");
        }

    }
}
