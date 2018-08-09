using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System;

namespace HouseGame
{
    public class ChildGuesserTutorial : MonoBehaviour
    {

        public RandomHouse chosenHouse = new RandomHouse();
        public List<RandomHouse> variations = new List<RandomHouse>();
        private MainGameController gameController;
        int level;
        private int chosenIndex;
        public int house_types;
        public int piece_types;
        public int house_numbers;
        public RandomHouse submittedHouse = new RandomHouse();


        public int final_guess;

        public GameObject x;
        public GameObject c;

        public GameObject right;
        public GameObject left;

        public int[] state;
        int tut;
        bool updated = false;



        void Awake()
        {
            DontDestroyOnLoad(gameObject);
            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();
            level = gameController.levelNumber;
            if (level == 1)
            {
                house_types = 3;
                piece_types = 2;
                house_numbers = 4;
                state = new int[house_numbers];
                for (int i = 0; i < house_numbers; ++i)
                {
                    state[i] = 0;
                }
            }
            if (level == 2)
            {
                house_numbers = 6;
                house_types = 4;
                piece_types = 3;
                state = new int[house_numbers];
                for (int i = 0; i < house_numbers; ++i)
                {
                    state[i] = 0;
                }
            }
            if (level == 3)
            {
                house_numbers = 9;
                house_types = 5;
                piece_types = 4;
                state = new int[house_numbers];
                for (int i = 0; i < house_numbers; ++i)
                {
                    state[i] = 0;
                }
            }
            if (level == 4)
            {
                house_numbers = 9;
                house_types = 6;
                piece_types = 6;
                state = new int[house_numbers];
                for (int i = 0; i < house_numbers; ++i)
                {
                    state[i] = 0;
                }
            }

        }

        // Use this for initialization
        void Start()
        {

            chosenHouse.wall = 1;
            chosenHouse.door1 = 0;
            chosenHouse.roof = 1;

            // house 1 ----------------------------------------------------------------------------
            var h1_b1 = GameObject.Find("h1_b1");
            var h1_b2 = GameObject.Find("h1_b2");
            var h1_b3 = GameObject.Find("h1_b3");
            var h1_b4 = GameObject.Find("h1_b4");
            var h1_d1 = GameObject.Find("h1_d1");
            var h1_r1 = GameObject.Find("h1_r1");


            GameObject wall11 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall12 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall13 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall14 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject door11 = Instantiate(gameController.doorHousePieceOptions[0]);
            GameObject roof11 = Instantiate(gameController.roofHousePieceOptions[0]);


            wall11.transform.SetParent(h1_b1.transform, false);
            wall12.transform.SetParent(h1_b2.transform, false);
            wall13.transform.SetParent(h1_b3.transform, false);
            wall14.transform.SetParent(h1_b4.transform, false);
            door11.transform.SetParent(h1_d1.transform, false);
            roof11.transform.SetParent(h1_r1.transform, false);


            // house 2 ----------------------------------------------------------------------------
            var h2_b1 = GameObject.Find("h2_b1");
            var h2_b2 = GameObject.Find("h2_b2");
            var h2_b3 = GameObject.Find("h2_b3");
            var h2_b4 = GameObject.Find("h2_b4");
            var h2_d1 = GameObject.Find("h2_d1");
            var h2_r1 = GameObject.Find("h2_r1");


            GameObject wall21 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall22 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall23 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject wall24 = Instantiate(gameController.wallHousePieceOptions[0]);
            GameObject door21 = Instantiate(gameController.doorHousePieceOptions[1]);
            GameObject roof21 = Instantiate(gameController.roofHousePieceOptions[0]);


            wall21.transform.SetParent(h2_b1.transform, false);
            wall22.transform.SetParent(h2_b2.transform, false);
            wall23.transform.SetParent(h2_b3.transform, false);
            wall24.transform.SetParent(h2_b4.transform, false);
            door21.transform.SetParent(h2_d1.transform, false);
            roof21.transform.SetParent(h2_r1.transform, false);


            // house 3 ----------------------------------------------------------------------------
            var h3_b1 = GameObject.Find("h3_b1");
            var h3_b2 = GameObject.Find("h3_b2");
            var h3_b3 = GameObject.Find("h3_b3");
            var h3_b4 = GameObject.Find("h3_b4");
            var h3_d1 = GameObject.Find("h3_d1");
            var h3_r1 = GameObject.Find("h3_r1");


            GameObject wall31 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject wall32 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject wall33 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject wall34 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject door31 = Instantiate(gameController.doorHousePieceOptions[0]);
            GameObject roof31 = Instantiate(gameController.roofHousePieceOptions[1]);


            wall31.transform.SetParent(h3_b1.transform, false);
            wall32.transform.SetParent(h3_b2.transform, false);
            wall33.transform.SetParent(h3_b3.transform, false);
            wall34.transform.SetParent(h3_b4.transform, false);
            door31.transform.SetParent(h3_d1.transform, false);
            roof31.transform.SetParent(h3_r1.transform, false);


            // house 4 ----------------------------------------------------------------------------
            var h4_b1 = GameObject.Find("h4_b1");
            var h4_b2 = GameObject.Find("h4_b2");
            var h4_b3 = GameObject.Find("h4_b3");
            var h4_b4 = GameObject.Find("h4_b4");
            var h4_d1 = GameObject.Find("h4_d1");
            var h4_r1 = GameObject.Find("h4_r1");


            GameObject wall41 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject wall42 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject wall43 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject wall44 = Instantiate(gameController.wallHousePieceOptions[1]);
            GameObject door41 = Instantiate(gameController.doorHousePieceOptions[1]);
            GameObject roof41 = Instantiate(gameController.roofHousePieceOptions[1]);


            wall41.transform.SetParent(h4_b1.transform, false);
            wall42.transform.SetParent(h4_b2.transform, false);
            wall43.transform.SetParent(h4_b3.transform, false);
            wall44.transform.SetParent(h4_b4.transform, false);
            door41.transform.SetParent(h4_d1.transform, false);
            roof41.transform.SetParent(h4_r1.transform, false);


            CreateQuestionPieces();
            gameController.SendRobotUtterance("child-guesser", true, -1, -1, -1);
        }

        // Update is called once per frame
        void Update()
        {
            
            if ((tut == 0) && (!updated))
            {
                GameObject a1 = GameObject.Find("arrow");
                a1.gameObject.GetComponent<Renderer>().enabled = true;
                updated = true;


            }
            if ((tut == 1)&&  (!updated))
            {
                gameController.SendRobotUtterance("child-guesser-1", true, -1, -1, -1);
                // circle ones to check off
                GameObject a1 = GameObject.Find("arrow");
                a1.gameObject.GetComponent<Renderer>().enabled = false;
                GameObject h1 = GameObject.Find("circle-h1-no");
                h1.gameObject.GetComponent<Renderer>().enabled = true;
                GameObject h2 = GameObject.Find("circle-h2-no");
                h2.gameObject.GetComponent<Renderer>().enabled = true;

                updated = true;
            }
            if ((tut == 1))
            {
                if ((state[0] == 1) && (state[1] == 1))
                {
                    tut++;
                    updated = false;
                }
            }
            if ((tut == 2) && (!updated))
            {
                gameController.SendRobotUtterance("child-guesser-2", true, -1, -1, -1);

                GameObject a2 = GameObject.Find("arrow2");
                a2.gameObject.GetComponent<Renderer>().enabled = true;
                GameObject h1 = GameObject.Find("circle-h1-no");
                h1.gameObject.GetComponent<Renderer>().enabled = false;
                GameObject h2 = GameObject.Find("circle-h2-no");
                h2.gameObject.GetComponent<Renderer>().enabled = false;
                updated = true;
            }
            if ((tut == 3) && (!updated))
            {
                gameController.SendRobotUtterance("child-guesser-3", true, -1, -1, -1);

                GameObject a2 = GameObject.Find("arrow2");
                a2.gameObject.GetComponent<Renderer>().enabled = false;
                GameObject h4 = GameObject.Find("circle-h4-no");
                h4.gameObject.GetComponent<Renderer>().enabled = true;
                updated = true;
            }
            if ((tut == 3))
            {
                if ((state[3] == 1) )
                {
                    tut++;
                    updated = false;
                }
            }
            if ((tut == 4) && (!updated))
            {
                gameController.SendRobotUtterance("child-guesser-4", true, -1, -1, -1);
                GameObject h4 = GameObject.Find("circle-h4-no");
                h4.gameObject.GetComponent<Renderer>().enabled = false;
                GameObject h3 = GameObject.Find("circle-h3-yes");
                h3.gameObject.GetComponent<Renderer>().enabled = true;
                //tut++;
                updated = true;
            }
            if ((tut == 4))
            {
                if (state[2] == 2)
                {
                    tut++;
                    updated = false;
                }
            }
            if ((tut == 5) && (!updated))
            {
                gameController.SendRobotUtterance("robot-both-houses-tutorial-2", true, -1, -1, -1);
                GameObject a3 = GameObject.Find("arrow3");
                a3.gameObject.GetComponent<Renderer>().enabled = true;
                updated = true;
            }
        }


       

        void CreateQuestionPieces()
        {
            int g_index = 1;
            for (int i = 0; i < piece_types; ++i)
            {
                string currentpanel_wall = "g" + g_index;
                string currentpanel_door = "g" + (g_index + piece_types);
                string currentpanel_roof = "g" + (g_index + 2 * piece_types);

                var canvas_w = GameObject.Find(currentpanel_wall);
                var canvas_d = GameObject.Find(currentpanel_door);
                var canvas_r = GameObject.Find(currentpanel_roof);

                GameObject wall = Instantiate(gameController.wallHousePieceOptions[i]);
                GameObject door = Instantiate(gameController.doorHousePieceOptions[i]);
                GameObject roof = Instantiate(gameController.roofHousePieceOptions[i]);

                wall.transform.SetParent(canvas_w.transform, false);
                door.transform.SetParent(canvas_d.transform, false);
                roof.transform.SetParent(canvas_r.transform, false);

                if (level > 1)
                {
                    string currentpanel_plant = "g" + (g_index + 3 * piece_types);
                    var canvas_p = GameObject.Find(currentpanel_plant);
                    GameObject plant = Instantiate(gameController.plantHousePieceOptions[i]);
                    plant.transform.SetParent(canvas_p.transform, false);
                }
                if (level > 2)
                {
                    string currentpanel_door2 = "g" + (g_index + 4 * piece_types);
                    var canvas_d2 = GameObject.Find(currentpanel_door2);
                    GameObject door2 = Instantiate(gameController.doorHousePieceOptions[i]);
                    door2.transform.SetParent(canvas_d2.transform, false);
                    GameObject r = Instantiate(right);
                    GameObject l = Instantiate(left);
                    r.transform.SetParent(canvas_d2.transform, false);
                    l.transform.SetParent(canvas_d.transform, false);
                }
                if (level > 3)
                {
                    string currentpanel_plant2 = "g" + (g_index + 5 * piece_types);
                    var canvas_p2 = GameObject.Find(currentpanel_plant2);
                    GameObject plant2 = Instantiate(gameController.plantHousePieceOptions[i]);
                    plant2.transform.SetParent(canvas_p2.transform, false);
                    GameObject r = Instantiate(right);
                    GameObject l = Instantiate(left);
                    l.transform.SetParent(canvas_p2.transform, false);
                    string currentpanel_plant = "g" + (g_index + 3 * piece_types);
                    var canvas_p = GameObject.Find(currentpanel_plant);
                    r.transform.SetParent(canvas_p.transform, false);

                }

                if (level == 1)
                {
                    wall.transform.localScale = new Vector3(2.0f, 2.0f, 0);
                    door.transform.localScale = new Vector3(2.0f, 2.0f, 0);
                    roof.transform.localScale = new Vector3(2.0f, 2.0f, 0);
                }

                g_index++;
            }
        }

        public void AnswerQuestion(int id)
        {

            id = id + piece_types;
            int type_piece = id / piece_types;
            int piece_id = id % piece_types;

            int type = 0;

            int isPresent = 0;

            var q_panel = GameObject.Find("Guess_Panel");
            GameObject q;
            if (type_piece == Constants.WALL)
            {
                type = 1;
                q = Instantiate(gameController.wallHousePieceOptions[piece_id]);
                q.transform.SetParent(q_panel.transform, false);

                if (piece_id == 1)
                {
                    isPresent = 1;

                }
                else
                {
                    tut++;
                    updated = false;
                }
            }
            else if (type_piece == Constants.DOOR)
            {
                if (level < 3)
                {
                    type = 2;
                }
                else
                {
                    type = 5;
                }
                q = Instantiate(gameController.doorHousePieceOptions[piece_id]);
                q.transform.SetParent(q_panel.transform, false);

                if (piece_id == 0)
                {
                    isPresent = 1;
                    tut++;
                    updated = false;
                }
                if (level > 2)
                {
                    GameObject l = Instantiate(left);
                    l.transform.SetParent(q_panel.transform, false);
                }
            }
            else if (type_piece == Constants.DOOR + 3)
            {
                type = 6;
                q = Instantiate(gameController.doorHousePieceOptions[piece_id]);
                q.transform.SetParent(q_panel.transform, false);

                if (piece_id == chosenHouse.door2)
                {
                    isPresent = 1;
                }
                GameObject r = Instantiate(right);
                r.transform.SetParent(q_panel.transform, false);
            }
            else if (type_piece == Constants.ROOF)
            {
                type = 3;
                q = Instantiate(gameController.roofHousePieceOptions[piece_id]);
                q.transform.SetParent(q_panel.transform, false);

                if (piece_id == 1)
                {
                    isPresent = 1;
                }
            }

            else if (type_piece == Constants.PLANT)
            {
                if (level < 4)
                {
                    type = 4;
                }
                else
                {
                    type = 7;
                }
                q = Instantiate(gameController.plantHousePieceOptions[piece_id]);
                q.transform.SetParent(q_panel.transform, false);

                if (piece_id == chosenHouse.plant1)
                {
                    isPresent = 1;
                }
                if (level > 3)
                {
                    GameObject r = Instantiate(right);
                    r.transform.SetParent(q_panel.transform, false);
                }
            }

            else if (type_piece == Constants.PLANT + 2)
            {
                type = 8;
                q = Instantiate(gameController.plantHousePieceOptions[piece_id]);
                q.transform.SetParent(q_panel.transform, false);

                if (piece_id == chosenHouse.plant2)
                {
                    isPresent = 1;
                }
                GameObject l = Instantiate(left);
                l.transform.SetParent(q_panel.transform, false);
            }


            if (isPresent == 1)
            {
                gameController.SendRobotUtterance("child-guess-piece-yes", false, type, piece_id, -1);
                GameObject ch = Instantiate(c);
                ch.transform.SetParent(q_panel.transform, false);
            }
            else
            {
                gameController.SendRobotUtterance("child-guess-piece-no", false, type, piece_id, -1);
                GameObject x1 = Instantiate(x);
                x1.transform.SetParent(q_panel.transform, false);
            }

            //////////////////////////////////

        }

        public void xOrCheckHouse(string button)
        {
            //Logger.Log("Clicked");
            int panelId = (int)Char.GetNumericValue(button[1]);

            string pic_id = "xh" + panelId;
            string check_id = "ch" + panelId;

            GameObject x = GameObject.Find(pic_id);
            GameObject c = GameObject.Find(check_id);
            GameObject c2 = GameObject.Find("ch" + final_guess);
            if ((state[panelId - 1] == 1) && (button[0] == 'x'))
            {
                x.transform.localScale = new Vector3(0f, 0f, 0f);
                state[panelId - 1] = 0;
            }
            else if ((state[panelId - 1] == 0) && (button[0] == 'x'))
            {
                x.transform.localScale = new Vector3(3f, 3f, 3f);
                state[panelId - 1] = 1;
            }
            else if ((state[panelId - 1] == 2) && (button[0] == 'x'))
            {
                final_guess = 0;
                state[panelId - 1] = 1;
                c.transform.localScale = new Vector3(0f, 0f, 0f);
                x.transform.localScale = new Vector3(3f, 3f, 3f);
            }
            else if ((state[panelId - 1] == 1) && (button[0] == 'c'))
            {
                if (final_guess != 0)
                {
                    c2.transform.localScale = new Vector3(0f, 0f, 0f);
                    state[final_guess - 1] = 0;
                }
                c.transform.localScale = new Vector3(3f, 3f, 3f);
                x.transform.localScale = new Vector3(0f, 0f, 0f);
                state[panelId - 1] = 2;
                final_guess = panelId;
            }
            else if ((state[panelId - 1] == 0) && (button[0] == 'c'))
            {
                if (final_guess != 0)
                {
                    c2.transform.localScale = new Vector3(0f, 0f, 0f);
                    state[final_guess - 1] = 0;
                }
                c.transform.localScale = new Vector3(3f, 3f, 3f);
                final_guess = panelId;
                state[panelId - 1] = 2;
            }
            else if ((state[panelId - 1] == 2) && (button[0] == 'c'))
            {
                final_guess = 0;
                state[panelId - 1] = 0;
                c.transform.localScale = new Vector3(0f, 0f, 0f);
            }

        }

        public void SubmitFinalAnswer()
        {
            if (final_guess != 0)
            {
                submittedHouse.wall = 1;
                submittedHouse.door1 = 0;
                submittedHouse.roof = 1;
                gameController.scene = Constants.CHILD_GUESSING_SCENE;
                SceneManager.LoadScene("Both_houses_L1");
            }

        }


        void CreateVariations()
        {
            System.Random rnd = new System.Random();

            if (level == 1)
            {
                List<int> takenPos = new List<int>();
                takenPos.Add(chosenIndex);
                RandomHouse var1 = new RandomHouse();
                RandomHouse var2 = new RandomHouse();
                RandomHouse var3 = new RandomHouse();
                //var1 changes wall and roof
                //var2 changes wall and door
                //var3 changes roof and door
                if (chosenHouse.wall == 0)
                {
                    var1.wall = 1;
                    var2.wall = 1;
                    var3.wall = 0;
                }
                else
                {
                    var1.wall = 0;
                    var2.wall = 0;
                    var3.wall = 1;
                }
                if (chosenHouse.roof == 0)
                {
                    var1.roof = 1;
                    var3.roof = 1;
                    var2.roof = 0;
                }
                else
                {
                    var1.roof = 0;
                    var3.roof = 0;
                    var2.roof = 1;
                }
                if (chosenHouse.door1 == 0)
                {
                    var2.door1 = 1;
                    var3.door1 = 1;
                    var1.door1 = 0;
                }
                else
                {
                    var2.door1 = 0;
                    var3.door1 = 0;
                    var1.door1 = 1;
                }


                for (int i = 0; i < 3; i++)
                {
                    int newPos = rnd.Next(1, house_numbers + 1);
                    while (takenPos.IndexOf(newPos) != -1)
                    {
                        newPos = rnd.Next(1, house_numbers + 1);
                    }

                    takenPos.Add(newPos);
                    if (i == 0)
                    {
                        var1.position = newPos;
                    }
                    if (i == 1)
                    {
                        var2.position = newPos;
                    }
                    if (i == 2)
                    {
                        var3.position = newPos;
                    }
                }

                variations.Add(var1);
                variations.Add(var2);
                variations.Add(var3);

            }

            if (level == 2)
            {
                List<int> takenPos = new List<int>();
                takenPos.Add(chosenIndex);
                for (int i = 0; i < 5; i++)
                {
                    RandomHouse variation = new RandomHouse();
                    int diff = 0;
                    while (diff != 1)
                    {
                        int w = rnd.Next(0, 3);
                        int r = rnd.Next(0, 3);
                        int d = rnd.Next(0, 3);
                        int p = rnd.Next(0, 3);
                        if ((w != chosenHouse.wall) && (r != chosenHouse.roof) && (d != chosenHouse.door1) && (p != chosenHouse.plant1))
                        {
                            diff = 1;
                            variation.wall = w;
                            variation.door1 = d;
                            variation.plant1 = p;
                            variation.roof = r;
                        }
                    }
                    int newPos = rnd.Next(1, house_numbers + 1);
                    while (takenPos.IndexOf(newPos) != -1)
                    {
                        newPos = rnd.Next(1, house_numbers + 1);
                    }

                    takenPos.Add(newPos);
                    variation.position = newPos;
                    variations.Add(variation);

                }

            }

            if (level == 3)
            {
                int newPos = 1;
                for (int i = 0; i < 9; i++)
                {
                    RandomHouse variation = new RandomHouse();
                    int diff = 0;
                    while (diff != 1)
                    {
                        int w = rnd.Next(0, 4);
                        int r = rnd.Next(0, 4);
                        int d = rnd.Next(0, 4);
                        int p = rnd.Next(0, 4);
                        int d2 = rnd.Next(0, 4);
                        if ((w != chosenHouse.wall) && (r != chosenHouse.roof) && (d != chosenHouse.door1) && (p != chosenHouse.plant1) && (d2 != chosenHouse.door2))
                        {
                            diff = 1;
                            variation.wall = w;
                            variation.door1 = d;
                            variation.plant1 = p;
                            variation.roof = r;
                            variation.door2 = d2;
                        }
                    }
                    if (newPos == chosenIndex)
                    {
                        newPos++;
                    }

                    variation.position = newPos;
                    variations.Add(variation);
                    newPos++;

                }

            }
            if (level == 4)
            {
                int newPos = 1;
                for (int i = 0; i < 9; i++)
                {
                    RandomHouse variation = new RandomHouse();
                    int diff = 0;
                    while (diff != 1)
                    {
                        int w = rnd.Next(0, 6);
                        int r = rnd.Next(0, 6);
                        int d = rnd.Next(0, 6);
                        int p = rnd.Next(0, 6);
                        int d2 = rnd.Next(0, 6);
                        int p2 = rnd.Next(0, 6);
                        if ((w != chosenHouse.wall) && (r != chosenHouse.roof) && (d != chosenHouse.door1) && (p != chosenHouse.plant1) && (d2 != chosenHouse.door2) && (p2 != chosenHouse.plant2))
                        {
                            diff = 1;
                            variation.wall = w;
                            variation.door1 = d;
                            variation.plant1 = p;
                            variation.roof = r;
                            variation.door2 = d2;
                            variation.plant2 = p2;
                        }
                    }
                    if (newPos == chosenIndex)
                    {
                        newPos++;
                    }

                    variation.position = newPos;
                    variations.Add(variation);
                    newPos++;

                }

            }
        }
    }
}
