using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;

namespace HouseGame
{
    public class GuessedHouse
    {
        public int wall;
        public int roof;
        public int door1;
        public int door2;
        public int plant1;
        public int plant2;
    }

    public class RobotGuesser : MonoBehaviour
    {

        private MainGameController gameController;
        private ChildBuilderTutorial childbuildertutorial;
        private ChildBuilder childbuilder;

        private List<int[]> possibleHouses;
        private List<int[]> charHouses;
        private int numbposs;
        private int pieceoptions;
        private int houseoptions;
        public int level;
        public int current_guess = -1;
        private int questionhousepiece;
        private int colorquestion;
        private int typepiecequestion;
        private int indexpiecequestion;
        private int numberguesses;
        public GameObject x;
        public GameObject right;
        public GameObject left;
        private GameObject question;
        public GuessedHouse finalHouse = new GuessedHouse();


        void Awake()
        {
            DontDestroyOnLoad(gameObject);
            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();

            if (!gameController.tutorial)
            {
                childbuilder = GameObject.Find("ExplainerSceneManager").GetComponent<ChildBuilder>();
            }
            else
            {
                childbuildertutorial = GameObject.Find("ExplainerSceneManager").GetComponent<ChildBuilderTutorial>();
            }
            gameController.currentScene = Constants.ROBOT_GUESSING_SCENE;
            level = gameController.levelNumber;


        }

        void CreateAllHouses()
        {
            possibleHouses = new List<int[]>();
            int[] h = new int[numbposs * houseoptions];

            int counter = 0;
            for (int i = 1; i < houseoptions + 1; ++i)
            {
                for (int j = 0; j < (int)Math.Pow(pieceoptions, i); ++j)
                {
                    for (int k = 0; k < (int)Math.Pow(pieceoptions, houseoptions - i); ++k)
                    {
                        //int m = (int)Math.Pow(pieceoptions, i);
                        h[counter] = j % pieceoptions;
                        Console.WriteLine(counter);
                        Console.WriteLine(j % pieceoptions);
                        ++counter;
                    }
                }
            }

            for (int i = 0; i < numbposs; ++i)
            {
                int[] house = new int[houseoptions];
                for (int j = 0; j < houseoptions; ++j)
                {
                    house[j] = h[i + j * numbposs];

                }

                possibleHouses.Add(house);
            }
        }

        void CreateCharacteristics()
        {
            //int[,] charHouse = new int[possibleHouses.Count,]; // blue, red, wood, bricks, green
            charHouses = new List<int[]>(); // 0 = blue, 1 = red, 2 = wood, 3 = bricks, 4 = green
            for (int i = 0; i < possibleHouses.Count; i++)
            {
                int[] houseCh = new int[5] { 0, 0, 0, 0, 0 };
                for (int j = 0; j < houseoptions; ++j)
                {
                    if (j == 0)  // wall
                    {
                        if (possibleHouses[i][j] == Constants.TYPE_WALL_BLUE) { houseCh[0] = 1; } // blue
                        if (possibleHouses[i][j] == Constants.TYPE_WALL_BRICK) { houseCh[3] = 1; } // brick
                        if (possibleHouses[i][j] == Constants.TYPE_WALL_LEGO) { houseCh[0] = 1; houseCh[1] = 1; houseCh[3] = 1; houseCh[4] = 1; } // lego
                        if (possibleHouses[i][j] == Constants.TYPE_WALL_STRAW) { houseCh[2] = 1; } // straw
                        if (possibleHouses[i][j] == Constants.TYPE_WALL_WINDOW) { houseCh[0] = 1; houseCh[2] = 1; } // window
                        if (possibleHouses[i][j] == Constants.TYPE_WALL_WOOD) { houseCh[2] = 1; } // wood
                    }
                    if ((j == 1) || (j == 4)) // door or door2
                    {
                        if (possibleHouses[i][j] == Constants.TYPE_DOOR_GLASS) { houseCh[0] = 1; } // glass
                        //if (possibleHouses[i][j] == 1) {  } // plain
                        if (possibleHouses[i][j] == Constants.TYPE_DOOR_RECTANGELS) { houseCh[1] = 1; } //  rectangles
                                                                           //if (possibleHouses[i][j] == 3) {  } // shutters
                        if (possibleHouses[i][j] == Constants.TYPE_DOOR_TWO) { houseCh[2] = 1; } // two
                        if (possibleHouses[i][j] == Constants.TYPE_DOOR_WINDOW) { houseCh[0] = 1; } // window
                    }
                    if (j == 2) // roof
                    {
                        if (possibleHouses[i][j] == Constants.TYPE_ROOF_PANEL) { houseCh[0] = 1; } // panel
                        if (possibleHouses[i][j] == Constants.TYPE_ROOF_PALMTREE) { houseCh[4] = 1; } // palm
                        if (possibleHouses[i][j] == Constants.TYPE_ROOF_PLAIN) { houseCh[4] = 1; } // plain
                        if (possibleHouses[i][j] == Constants.TYPE_ROOF_ROUNDED) { houseCh[1] = 1; } //  rounded
                        if (possibleHouses[i][j] == Constants.TYPE_ROOF_TYPICAL) { houseCh[3] = 1; } //  typical
                        if (possibleHouses[i][j] == Constants.TYPE_ROOF_GARDEN) { houseCh[4] = 1; } //  garden
                    }
                    if ((j == 3) || (j == 5)) // plant
                    {
                        if (possibleHouses[i][j] == Constants.TYPE_PLANT_BIRD) { houseCh[0] = 1; } // bird
                        if (possibleHouses[i][j] == Constants.TYPE_PLANT_FLOWER) { houseCh[0] = 1; houseCh[4] = 1; } // flower
                        if (possibleHouses[i][j] == Constants.TYPE_PLANT_TREE) { houseCh[4] = 1; } // tree
                        if (possibleHouses[i][j] == Constants.TYPE_PLANT_SLIDE) { houseCh[1] = 1; } // slide
                        if (possibleHouses[i][j] == Constants.TYPE_PLANT_TREEHOUSE) { houseCh[4] = 1; } //  treehouse
                        if (possibleHouses[i][j] == Constants.TYPE_PLANT_PLAIN) { houseCh[4] = 1; } // plain
                    }
                }
                charHouses.Add(houseCh);
            }
        }


        GameObject getCorrespondingPiece(int diff_color, int diff_piece, int color, int i, int j)
        {
            if (diff_color < diff_piece)
            {
                questionhousepiece = 0;
                colorquestion = color;
                GameObject colorPieceGuess = Instantiate(gameController.colorHousePieceOptions[color]);
                return colorPieceGuess;
            }
            else
            {
                questionhousepiece = 1;
                typepiecequestion = i;
                indexpiecequestion = j;
                if (i == 0) // wall
                {
                    GameObject wallPieceGuess = Instantiate(gameController.wallHousePieceOptions[j]);
                    return wallPieceGuess;

                }
                if ((i == 1) || (i == 4)) // door
                {
                    GameObject doorPieceGuess = Instantiate(gameController.doorHousePieceOptions[j]);
                    return doorPieceGuess;


                }
                if (i == 2)  // roof
                {
                    GameObject roofPieceGuess = Instantiate(gameController.roofHousePieceOptions[j]);
                    return roofPieceGuess;

                }
                if ((i == 3) || (i == 5))// plant
                {
                    GameObject plantPieceGuess = Instantiate(gameController.plantHousePieceOptions[j]);
                    return plantPieceGuess;

                }
            }
            return null;

        }


        // Use this for initialization
        void Start()
        {
            //level = 1;

            gameController.SendRobotUtterance("child-answering-questions-start", false, -1,-1,-1);

            if (gameController.tutorial)
            {
                {
                    pieceoptions = 4;
                    houseoptions = 5; //wall,door1,roof,plant1,door2

                    var w1 = GameObject.Find("body1");
                    var w2 = GameObject.Find("body2");
                    var w3 = GameObject.Find("body3");
                    var w4 = GameObject.Find("body4");
                    var d1 = GameObject.Find("door1");
                    var r1 = GameObject.Find("roof1");
                    var d2 = GameObject.Find("door2");

                    GameObject wall1 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                    GameObject wall2 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                    GameObject wall3 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                    GameObject wall4 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                    GameObject door = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door1]);
                    GameObject roof = Instantiate(gameController.roofHousePieceOptions[childbuilder.explainerHouse.roof]);
                    GameObject door2 = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door2]);

                    wall1.transform.SetParent(w1.transform, false);
                    wall2.transform.SetParent(w2.transform, false);
                    wall3.transform.SetParent(w3.transform, false);
                    wall4.transform.SetParent(w4.transform, false);
                    door.transform.SetParent(d1.transform, false);
                    roof.transform.SetParent(r1.transform, false);
                    door2.transform.SetParent(d2.transform, false);
                }
            }

            if ((level == 1) && (!gameController.tutorial))
            {

                pieceoptions = 2;
                houseoptions = 3; //wall,door,roof
                var w1 = GameObject.Find("body1");
                var w2 = GameObject.Find("body2");
                var w3 = GameObject.Find("body3");
                var w4 = GameObject.Find("body4");
                var d1 = GameObject.Find("door1");
                var r1 = GameObject.Find("roof1");

                GameObject wall1 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall2 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall3 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall4 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject door = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door1]);
                GameObject roof = Instantiate(gameController.roofHousePieceOptions[childbuilder.explainerHouse.roof]);

                wall1.transform.SetParent(w1.transform, false);
                wall2.transform.SetParent(w2.transform, false);
                wall3.transform.SetParent(w3.transform, false);
                wall4.transform.SetParent(w4.transform, false);
                door.transform.SetParent(d1.transform,false);
                roof.transform.SetParent(r1.transform, false);


            }
            if ((level == 2) && (!gameController.tutorial))
            {
                pieceoptions = 3;
                houseoptions = 4; //wall,door,roof,plant

                var w1 = GameObject.Find("body1");
                var w2 = GameObject.Find("body2");
                var w3 = GameObject.Find("body3");
                var w4 = GameObject.Find("body4");
                var d1 = GameObject.Find("door1");
                var r1 = GameObject.Find("roof1");
                var p1 = GameObject.Find("plant1");

                GameObject wall1 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall2 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall3 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall4 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject door = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door1]);
                GameObject roof = Instantiate(gameController.roofHousePieceOptions[childbuilder.explainerHouse.roof]);
                GameObject plant = Instantiate(gameController.plantHousePieceOptions[childbuilder.explainerHouse.plant1]);

                wall1.transform.SetParent(w1.transform, false);
                wall2.transform.SetParent(w2.transform, false);
                wall3.transform.SetParent(w3.transform, false);
                wall4.transform.SetParent(w4.transform, false);
                door.transform.SetParent(d1.transform, false);
                roof.transform.SetParent(r1.transform, false);
                plant.transform.SetParent(p1.transform, false);
            }
            if ((level == 3) && (!gameController.tutorial))
            {
                pieceoptions = 4;
                houseoptions = 5; //wall,door1,roof,plant1,door2

                var w1 = GameObject.Find("body1");
                var w2 = GameObject.Find("body2");
                var w3 = GameObject.Find("body3");
                var w4 = GameObject.Find("body4");
                var d1 = GameObject.Find("door1");
                var r1 = GameObject.Find("roof1");
                var p1 = GameObject.Find("plant1");
                var d2 = GameObject.Find("door2");

                GameObject wall1 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall2 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall3 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall4 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject door = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door1]);
                GameObject roof = Instantiate(gameController.roofHousePieceOptions[childbuilder.explainerHouse.roof]);
                GameObject plant = Instantiate(gameController.plantHousePieceOptions[childbuilder.explainerHouse.plant1]);
                GameObject door2 = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door2]);

                wall1.transform.SetParent(w1.transform, false);
                wall2.transform.SetParent(w2.transform, false);
                wall3.transform.SetParent(w3.transform, false);
                wall4.transform.SetParent(w4.transform, false);
                door.transform.SetParent(d1.transform, false);
                roof.transform.SetParent(r1.transform, false);
                plant.transform.SetParent(p1.transform, false);
                door2.transform.SetParent(d2.transform, false);
            }
            if ((level == 4) && (!gameController.tutorial))
            {
                pieceoptions = 6;
                houseoptions = 6; //wall,door1,roof,plant1,door2,plant2

                var w1 = GameObject.Find("body1");
                var w2 = GameObject.Find("body2");
                var w3 = GameObject.Find("body3");
                var w4 = GameObject.Find("body4");
                var d1 = GameObject.Find("door1");
                var r1 = GameObject.Find("roof1");
                var p1 = GameObject.Find("plant1");
                var d2 = GameObject.Find("door2");
                var p2 = GameObject.Find("plant2");

                GameObject wall1 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall2 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall3 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject wall4 = Instantiate(gameController.wallHousePieceOptions[childbuilder.explainerHouse.wall]);
                GameObject door = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door1]);
                GameObject roof = Instantiate(gameController.roofHousePieceOptions[childbuilder.explainerHouse.roof]);
                GameObject plant = Instantiate(gameController.plantHousePieceOptions[childbuilder.explainerHouse.plant1]);
                GameObject door2 = Instantiate(gameController.doorHousePieceOptions[childbuilder.explainerHouse.door2]);
                GameObject plant2 = Instantiate(gameController.plantHousePieceOptions[childbuilder.explainerHouse.plant2]);

                wall1.transform.SetParent(w1.transform, false);
                wall2.transform.SetParent(w2.transform, false);
                wall3.transform.SetParent(w3.transform, false);
                wall4.transform.SetParent(w4.transform, false);
                door.transform.SetParent(d1.transform, false);
                roof.transform.SetParent(r1.transform, false);
                plant.transform.SetParent(p1.transform, false);
                door2.transform.SetParent(d2.transform, false);
                plant2.transform.SetParent(p2.transform, false);
            }

            numbposs = (int)(Math.Pow(pieceoptions, houseoptions));
            CreateAllHouses();
            CreateCharacteristics();
            FindBestGuess();

        }

        void FindBestGuess()
        {

            // check for the best question regardign specific house pieces
            //int numbposs = (int)Math.Pow(pieceoptions, houseoptions);

            int[,] numb = new int[houseoptions, pieceoptions];
            for (int i = 0; i < houseoptions; ++i)
            {
                foreach (int[] house in possibleHouses)
                {
                    int val = house[i];
                    numb[i, val]++;
                }
            }

            //check for the best question regarding colors
            //int numbposs = (int)Math.Pow(pieceoptions, houseoptions);
            int[] numbcolor = new int[5];
            for (int i = 0; i < 5; ++i)
            {
                foreach (int[] col in charHouses)
                {
                    if (col[i] == 1)
                    {
                        numbcolor[i]++;
                    }
                }
            }

            for (int i = 0; i < houseoptions; i++)
            {
                for (int j = 0; j < pieceoptions; ++j)
                {
                    Console.Write(numb[i, j]);

                }
                Console.WriteLine();
            }

            int target = (numbposs / 2);
            int diff_piece = 999999;
            int indexI = 0;
            int indexJ = 0;

            for (int i = 0; i < houseoptions; i++)
            {
                for (int j = 0; j < pieceoptions; ++j)
                {
                    if (Math.Abs(numb[i, j] - target) < diff_piece)
                    {
                        diff_piece = Math.Abs(numb[i, j] - target);
                        indexI = i;
                        indexJ = j;
                    }
                }
            }

            int diff_color = 999999;
            int c = 0;
            for (int i = 0; i < 5; ++i)
            {
                if (Math.Abs(numbcolor[i] - target) < diff_color)
                {
                    diff_color = Math.Abs(numbcolor[i] - target);
                    c = i;
                }
            }

            // get appropriate question
            question = getCorrespondingPiece(diff_color, diff_piece, c, indexI, indexJ);


            // for tutorial if present outline yes, otherwise outline no
            /*if (gameController.tutorial)
            {
                if (diff_color == 1)
                {

                }

                if (diff_piece == 1)
                {
                    if (indexI == 0) // wall
                    {
                        if (indexJ == childbuilder.builtHouse.wall)
                        {
                            GameObject y = GameObject.Find("circle-yes");
                            y.gameObject.GetComponent<Renderer>().enabled = true;
                        }
                        else
                        {
                            GameObject n = GameObject.Find("circle-no");
                            n.gameObject.GetComponent<Renderer>().enabled = true;
                        }

                    }
                    if (indexI == 1) // door1
                    {
                        if (indexJ == childbuilder.builtHouse.door1)
                        {
                            GameObject y = GameObject.Find("circle-yes");
                            y.gameObject.GetComponent<Renderer>().enabled = true;
                        }
                        else
                        {
                            GameObject n = GameObject.Find("circle-no");
                            n.gameObject.GetComponent<Renderer>().enabled = true;
                        }
                    }
                    if (indexI == 4) // door2
                    {
                        if (indexJ == childbuilder.builtHouse.door2)
                        {
                            GameObject y = GameObject.Find("circle-yes");
                            y.gameObject.GetComponent<Renderer>().enabled = true;
                        }
                        else
                        {
                            GameObject n = GameObject.Find("circle-no");
                            n.gameObject.GetComponent<Renderer>().enabled = true;
                        }
                    }
                    if (indexI == 2)  // roof
                    {
                        if (indexJ == childbuilder.builtHouse.roof)
                        {
                            GameObject y = GameObject.Find("circle-yes");
                            y.gameObject.GetComponent<Renderer>().enabled = true;
                        }
                        else
                        {
                            GameObject n = GameObject.Find("circle-no");
                            n.gameObject.GetComponent<Renderer>().enabled = true;
                        }
                    }

                }
         
            }*/


            var canvas = GameObject.Find("Guess_Panel");
            //show on screen question
            question.transform.SetParent(canvas.transform,false);

            //Robots asks question
            if ((questionhousepiece == 1) && ((indexI == 0) || (indexI == 2)))
            {
                gameController.SendRobotUtterance("ask-question-piece", false, indexI + 1, indexJ , -1);
            }
            if ((questionhousepiece == 1) && (indexI == 1) && (level < 3))
            {
                gameController.SendRobotUtterance("ask-question-piece", false, 2, indexJ, -1);
            }
            if ((questionhousepiece == 1) && (indexI == 1) && (level > 2))
            {
                gameController.SendRobotUtterance("ask-question-piece", false, 6, indexJ, -1); // left door
            }
            if ((questionhousepiece == 1) && (indexI == 4) && (level > 2))
            {
                gameController.SendRobotUtterance("ask-question-piece", false, 5, indexJ, -1); // right door
            }
            if ((questionhousepiece == 1) && (indexI == 3) && (level < 4))
            {
                gameController.SendRobotUtterance("ask-question-piece", false, 4, indexJ, -1);
            }
            if ((questionhousepiece == 1) && (indexI == 3) && (level > 3))
            {
                gameController.SendRobotUtterance("ask-question-piece", false, 7, indexJ, -1);
            }
            if ((questionhousepiece == 1) && (indexI == 5) && (level > 3))
            {
                gameController.SendRobotUtterance("ask-question-piece", false, 8, indexJ, -1);
            }

            if (questionhousepiece == 0)
            {
                gameController.SendRobotUtterance("ask-question-color", false, -1, -1, c);
            }

            // if level is 3 or 4 and its a door must show left or right

            if ((level >2) && (indexI == 1) && (questionhousepiece==1))
            {
                GameObject left_arrow = Instantiate(left);
                left_arrow.transform.SetParent(canvas.transform, false);
            }
            if ((level > 2) && (indexI == 4) && (questionhousepiece == 1))
            {
                GameObject right_arrow = Instantiate(right);
                right_arrow.transform.SetParent(canvas.transform, false);
            }

            if ((level > 3) && (indexI == 3) && (questionhousepiece == 1))
            {
                GameObject right_arrow = Instantiate(right);
                right_arrow.transform.SetParent(canvas.transform, false);
            }
            if ((level > 3) && (indexI == 5) && (questionhousepiece == 1))
            {
                GameObject left_arrow = Instantiate(left);
                left_arrow.transform.SetParent(canvas.transform, false);
            }




            Console.WriteLine(diff_piece);
            Console.WriteLine(indexI);
            Console.WriteLine(indexJ);

            int[] best = new int[2];
            best[0] = indexI;
            best[1] = indexJ;

            numberguesses++;

        }

        public void UpdatePossibleHouses(int ispresent)
        {
            //int numbposs = (int)Math.Pow(pieceoptions, houseoptions);
            if ((ispresent == 1) && (questionhousepiece == 1))
            {
                for (int i=0; i<numbposs;++i)
                {
                    if (possibleHouses[i][typepiecequestion] != indexpiecequestion)
                    {
                        charHouses.RemoveAt(i);
                        possibleHouses.RemoveAt(i);
                        --i;
                        numbposs--;
                        //remove that house
                    }
                }

                //remove all houses that dont have it present
            }
            else if ((ispresent == 1) && (questionhousepiece == 0))
            {
                for (int i =0; i< numbposs;++i)
                {
                    if (charHouses[i][colorquestion] == 0)
                    {
                        charHouses.RemoveAt(i);
                        possibleHouses.RemoveAt(i);
                        --i;
                        numbposs--;
                    }
                }
                //remove all houses that have it present
            }
            else if ((ispresent == 0) & (questionhousepiece == 1))
            {
                for (int i = 0; i < numbposs; ++i)
                {
                    if (possibleHouses[i][typepiecequestion] == indexpiecequestion)
                    {
                        charHouses.RemoveAt(i);
                        possibleHouses.RemoveAt(i);
                        --i;
                        numbposs--;
                    }
                }
            }
            else if ((ispresent == 0) & (questionhousepiece == 0))
            {
                for (int i = 0; i < numbposs; ++i)
                {
                    if (charHouses[i][colorquestion] == 1)
                    {
                        charHouses.RemoveAt(i);
                        possibleHouses.RemoveAt(i);
                        --i;
                        numbposs--;
                    }
                }
            }

            CurrentGuessOnPanel(ispresent);
            if (numbposs == 1)
            {
                MakeFinalGuess();
            }
            else
            {
                FindBestGuess();
            }
        }

        public void CurrentGuessOnPanel(int ispresent)
        {
            string panelspot = "g" + numberguesses;
            var canvas = GameObject.Find(panelspot);
            //show on screen question
            GameObject x1 = Instantiate(x);

            question.transform.SetParent(canvas.transform, false);
            if (ispresent == 0)
            {
                x1.transform.SetParent(canvas.transform, false);
            }
            if (level != 4)
            {
                question.transform.localScale = new Vector3(1.5f, 1.5f, 0);
            }

            if ((level > 2) && (typepiecequestion == 1) && (questionhousepiece == 1))
            {
                GameObject left_arrow = Instantiate(left);
                left_arrow.transform.SetParent(canvas.transform, false);
            }
            if ((level > 2) && (typepiecequestion == 4) && (questionhousepiece == 1))
            {
                GameObject right_arrow = Instantiate(right);
                right_arrow.transform.SetParent(canvas.transform, false);
            }

            if ((level > 3) && (typepiecequestion == 3) && (questionhousepiece == 1))
            {
                GameObject right_arrow = Instantiate(right);
                right_arrow.transform.SetParent(canvas.transform, false);
            }
            if ((level > 3) && (typepiecequestion == 5) && (questionhousepiece == 1))
            {
                GameObject left_arrow = Instantiate(left);
                left_arrow.transform.SetParent(canvas.transform, false);
            }

        }

        public void MakeFinalGuess()
        {

            if (gameController.internalGameState == Constants.END_GAME)
            {
                gameController.SendRobotUtterance("end-game", false, -1, -1, -1);
                Logger.Log("GoingToQuit");
                //Application.Quit();
            } 

            finalHouse.wall = possibleHouses[0][0];
            finalHouse.door1 = possibleHouses[0][1];
            finalHouse.roof = possibleHouses[0][2];

            if (level > 1)
            {
                finalHouse.plant1 = possibleHouses[0][3];
            }
            if (level > 2)
            {
                finalHouse.door2 = possibleHouses[0][4];
            }
            if (level > 3)
            {
                finalHouse.plant2 = possibleHouses[0][5];
            }


            if (level == 1)
            {
                SceneManager.LoadScene("Both_houses_L1");
            }
            if (level == 2)
            {
                SceneManager.LoadScene("Both_houses_L2");
            }
            if (level == 3)
            {
                SceneManager.LoadScene("Both_houses_L3");
            }
            if (level == 4)
            {
                SceneManager.LoadScene("Both_houses_L4");
            }


        }

        // Update is called once per frame
        /*void Update()
        {

        }*/
    }
}