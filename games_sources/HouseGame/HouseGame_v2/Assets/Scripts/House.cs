using UnityEngine;
using System.Collections;


namespace HouseGame
{
    public class House : MonoBehaviour
    {
        public int wall;
        public int roof ;
        public int door1;
        public int door2;
        public int plant1;
        public int plant2;     
        
        public House()
        {
            wall = -1;
            roof = -1;
            door1 = -1;
            door2 = -1;
            plant1 = -1;
            plant2 = -1;
        }   
    }
}
