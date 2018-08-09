using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System;


namespace TrainGame {

    public class ConstructedTrain
    {

        public List<GameObject> frontPieces;
        public List<GameObject> bodyPieces;
        public List<GameObject> cabinPieces;
        public List<GameObject> backPieces;
        public List<GameObject> wheelPieces;
        public List<GameObject> smokePieces;

        public int front;
        public int body;
        public int cabin;
        public int back;
        public int wheel1;
        public int wheel2;
        public int wheel3;
        public int wheel4;
        public int smoke1;
        public int smoke2;
        public int smoke3;

        public int n_front_pieces = 0;
        public int n_body_pieces = 0;
        public int n_cabin_pieces = 0;
        public int n_back_pieces = 0;
        public int n_smoke_pieces = 0;
        public int n_wheel_pieces = 0;

        public int order_front;
        public int order_body;
        public int order_cabin;
        public int order_back;
        public int order_wheel1;
        public int order_wheel2;
        public int order_wheel3;
        public int order_wheel4;
        public int order_smoke1;
        public int order_smoke2;
        public int order_smoke3;

        
    }


    public class ChildBuilder : MonoBehaviour {

        int levelNumber;
        private int correct = 1;

        public int select_type = -1;
        public int select_id = -1;
        public int step = 1;
        private int rightFirst = 0;
        public int score;
        

        // panels where the outline and rocket pieces will be placed
        private Transform DashedOutlinesPanel;
        private Transform SelectedOutlinesPanel;
        private Transform TrainPiecesLeftPanel;
        private Transform TrainPiecesRightPanel;

        // outline slots
        private List<GameObject> dashedFrontOutlineSlots = new List<GameObject>();
        private List<GameObject> dashedBodyOutlineSlot = new List<GameObject>();
        private List<GameObject> dashedCabinOutlineSlots = new List<GameObject>();
        private List<GameObject> dashedBackOutlineSlots = new List<GameObject>();
        private List<GameObject> dashedWheelOutlineSlots = new List<GameObject>();
        private List<GameObject> dashedSmokeOutlineSlots = new List<GameObject>();

        // panel body pieces
        public List<GameObject> frontPanelTrainPieces = new List<GameObject>();
        private List<GameObject> bodyPanelTrainPieces = new List<GameObject>();
        private List<GameObject> cabinPanelTrainPieces = new List<GameObject>();
        private List<GameObject> backPanelTrainPieces = new List<GameObject>();
        private List<GameObject> wheelPanelTrainPieces = new List<GameObject>();
        private List<GameObject> smokePanelTrainPieces = new List<GameObject>();

        // keep track of the 2 rockets and all of their pieces
        public ConstructedTrain childTrain;
        public ConstructedTrain robotTrain;
        public int order = 1;

        // animator and related variables
        public Animator panelsAnimator;
        private bool firstStateChangeOccured = false;
        public bool firstTimeIn = true;

        //public House builtHouse = new House();

        // type of piece selected
        private int currentPieceTypeSelected;
        public int lastPieceTypeSelected;


        // buttons
        public GameObject explainerCompleteButton;
        public GameObject NewGameButton;

        // game controller
        private MainGameController gameController;


        void Start()
        {
            
            DontDestroyOnLoad(gameObject);
            // initialize pieceTypeSelected
            currentPieceTypeSelected = Constants.NONE_SELECTED;
            lastPieceTypeSelected = Constants.NONE_SELECTED;

            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();
            gameController.currentScene = Constants.EXPLAINER_SCENE;
            levelNumber = gameController.levelNumber;

            if (levelNumber == 1)
                childTrain = new ConstructedTrain();
            if (levelNumber == 2)
                childTrain = new ConstructedTrain();
            if (levelNumber == 3)
                childTrain = new ConstructedTrain();
            if (levelNumber == 4)
                childTrain = new ConstructedTrain();

            robotTrain = new ConstructedTrain();

            // set up the scene
            SetUpExplainerScene();

            // enable drag and drop functionality
            EnableDragAndDropGameplay();

            // turn off the buttons
            explainerCompleteButton.SetActive(false);
            NewGameButton.SetActive(false);

            if (!gameController.tutorial)
            {
                CreateRandomTrain();
                TrainInCorner();
            }
            else
            {
                HideBuilderPieces();
                ShowTutorialPieces();
                TrainInCornerTutorial();
            }

            // subscribe to appropriate events
            Slot.OnPieceAddedToTrain += ExplainerPieceAddedToTrain;
            DragHandler.OnPieceRemovedByTrash += ExplainerPieceRemoved;
            //lastPieceTypeSelected = Constants.NONE_SELECTED;
            //gameController.previousLevel = Application.loadedLevel;
            firstStateChangeOccured = false;
            lastPieceTypeSelected = Constants.NONE_SELECTED;
        }

        void Update()
        {
            /*if (firstTimeIn)
            {
                Logger.Log("HJERE");
                firstTimeIn = false;
                step = 1;
                lastPieceTypeSelected = Constants.NONE_SELECTED;
            }*/
            if ((levelNumber == 1) && (!gameController.tutorial))
            {
                if ((step == 1) &&  (correct == 1)) //front
                {
                    lastPieceTypeSelected = Constants.NONE_SELECTED;
                    //EnableDragAndDropGameplay();
                    //PlaceOutlineAndHousePieces();
                    gameController.SendRobotUtterance("level-1-intro", false,-1,-1,-1,-1);
                    gameController.SendRobotUtterance("level-1", false, Constants.FRONT, -1,-1,-1);
                    rightFirst = 1;
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_front");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.FRONT;
                    select_id = 0;


                }
                /*if ((step == 1) && (correct == 0) && (rightFirst == 0))
                {
                    gameController.SendRobotUtterance("level-1-wrong", false, -1, -1, -1, -1);
                }*/
                if ((step == 2) && (correct == 1)) //body
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.BODY,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_body");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.BODY;
                    select_id = 0;
                }
                if ((step == 3) && (correct == 1)) //cabin
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.CABIN,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_cabin");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.CABIN;
                    select_id = 0;
                }
                if ((step == 4) && (correct == 1)) //back
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.BACK,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_back");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.BACK;
                    select_id = 0;
                }
                if ((step == 5) && (correct == 1)) // smoke1
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.SMOKE1,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_smoke1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.SMOKE;
                    select_id = 0;
                }
                if ((step == 6) && (correct == 1)) //smoke2
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.SMOKE2,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_smoke2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.SMOKE;
                    select_id = 1;
                }
                if ((step == 7) && (correct == 1)) //smoke3
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.SMOKE3,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_smoke3");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.SMOKE;
                    select_id = 2;
                }
                if ((step == 8) && (correct == 1)) //wheel1
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.WHEEL1,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_wheel1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.WHEEL;
                    select_id = 0;
                }
                if ((step == 9) && (correct == 1)) //wheel2
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.WHEEL2,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_wheel2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.WHEEL;
                    select_id = 1;
                }
                if ((step == 10) && (correct == 1)) //wheel3
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.WHEEL3,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_wheel3");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.WHEEL;
                    select_id = 2;
                }
                if ((step == 11) && (correct == 1)) //wheel4
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.WHEEL4,-1,-1,-1);
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
                    correct = 0;
                    GameObject sc = GameObject.Find("sel_wheel4");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    select_type = Constants.WHEEL;
                    select_id = 3;
                    Logger.Log(score);
                }
            }
            if ((levelNumber == 2) && (!gameController.tutorial))
            {
                if ((step == 1)  && (correct == 1)) //body and cabin    
                {
                    gameController.SendRobotUtterance("level-2-intro", false, -1,-1,-1,-1);
                    gameController.SendRobotUtterance("level-2", false, Constants.BODY, Constants.CABIN,-1,-1);
                    correct = 0;
                    rightFirst = 1;
                    GameObject sc1 = GameObject.Find("sel_body");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_cabin");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 3)  && (correct == 1)) // wheel 1 and 2
                {
                    gameController.SendRobotUtterance("level-2", false, Constants.WHEEL1, Constants.WHEEL2, -1, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1);
                    GameObject sc1 = GameObject.Find("sel_wheel1");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_wheel2");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 5)  && (correct == 1)) //wheel 3 and 4
                {
                    gameController.SendRobotUtterance("level-2", false, Constants.WHEEL3, Constants.WHEEL4, -1, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1);
                    GameObject sc1 = GameObject.Find("sel_wheel3");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_wheel4");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 7)  && (correct == 1)) // front and back
                {
                    gameController.SendRobotUtterance("level-2", false, Constants.FRONT, Constants.BACK, -1, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0);
                    GameObject sc1 = GameObject.Find("sel_front");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_back");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 9) && (correct == 1)) // smoke 1 and 2
                {
                    gameController.SendRobotUtterance("level-2", false, Constants.SMOKE1, Constants.SMOKE2, -1, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0);
                    GameObject sc1 = GameObject.Find("sel_smoke1");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_smoke2");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 11) && (correct == 1)) // smoke3
                {
                    gameController.SendRobotUtterance("level-1", false, Constants.SMOKE3, -1, -1, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0);
                    GameObject sc1 = GameObject.Find("sel_smoke3");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    Logger.Log(score);
                }
            }
            if ((levelNumber == 3) && (!gameController.tutorial))
            {
                if ((step == 1) && (correct == 1)) //  wheel 1,2,3         
                {
                    gameController.SendRobotUtterance("level-3-intro", false, -1, -1, -1, -1);
                    gameController.SendRobotUtterance("level-3", false, Constants.WHEEL1, Constants.WHEEL2, Constants.WHEEL3, -1);
                    correct = 0;
                    rightFirst = 1;
                    GameObject sc1 = GameObject.Find("sel_wheel1");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_wheel2");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc3 = GameObject.Find("sel_wheel3");
                    sc3.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 4) && (correct == 1)) // wheel 4, body, cabin
                {
                    gameController.SendRobotUtterance("level-3", false, Constants.WHEEL4, Constants.BODY, Constants.CABIN, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1);
                    GameObject sc1 = GameObject.Find("sel_wheel4");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_body");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc3 = GameObject.Find("sel_cabin");
                    sc3.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 7) && (correct == 1)) // smoke 1,2,3
                {
                    gameController.SendRobotUtterance("level-3", false, Constants.SMOKE1, Constants.SMOKE2, Constants.SMOKE3, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0);
                    GameObject sc1 = GameObject.Find("sel_smoke1");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_smoke2");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc3 = GameObject.Find("sel_smoke3");
                    sc3.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 10) && (correct == 1)) // front,back
                {
                    gameController.SendRobotUtterance("level-2", false, Constants.FRONT, Constants.BACK, -1, -1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0);
                    GameObject sc1 = GameObject.Find("sel_front");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_back");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                }
            }
            if ((levelNumber == 4) && (!gameController.tutorial)) 
            {
                if ((step == 1) && (correct == 1)) // front, body, smoke1, wheel1
                {
                    gameController.SendRobotUtterance("level-4-intro", false, -1, -1, -1, -1);
                    gameController.SendRobotUtterance("level-4", false, Constants.FRONT, Constants.BODY, Constants.SMOKE1, Constants.WHEEL1);
                    correct = 0;
                    rightFirst = 1;
                    GameObject sc1 = GameObject.Find("sel_front");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_body");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc3 = GameObject.Find("sel_smoke1");
                    sc3.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc4 = GameObject.Find("sel_wheel1");
                    sc4.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 5) && (correct == 1)) // smoke2, wheel2, smoke 3, cabin
                {
                    gameController.SendRobotUtterance("level-4", false, Constants.SMOKE2, Constants.WHEEL2, Constants.SMOKE3, Constants.CABIN);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1);
                    GameObject sc1 = GameObject.Find("sel_smoke2");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_wheel2");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc3 = GameObject.Find("sel_smoke3");
                    sc3.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc4 = GameObject.Find("sel_cabin");
                    sc4.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 9) && (correct == 1)) // wheel3, wheel4, back
                {
                    gameController.SendRobotUtterance("level-3", false, Constants.WHEEL3, Constants.WHEEL4, Constants.BACK,-1);
                    correct = 0;
                    rightFirst = 1;
                    deleteWrongChildren(0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1);
                    GameObject sc1 = GameObject.Find("sel_wheel3");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("sel_wheel4");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc3 = GameObject.Find("sel_back");
                    sc3.gameObject.GetComponent<Image>().enabled = true;
                }
            }
            if (gameController.tutorial)
            {
                if ((step == 1) && (correct == 1)) 
                {
                    // teach them to click
                    gameController.SendRobotUtterance("tutorial-1", false, -1,-1,-1,-1);
                    gameController.SendRobotUtterance("tutorial-2", false, -1,-1,-1,-1);
                    correct = 0;
                    GameObject sc1 = GameObject.Find("arrow1");
                    sc1.gameObject.GetComponent<Renderer>().enabled = true;
                    GameObject sc2 = GameObject.Find("arrow2");
                    sc2.gameObject.GetComponent<Renderer>().enabled = true;
                }
                if ((step == 2) && (correct == 1)) 
                {
                    // teach them to drag
                    gameController.SendRobotUtterance("tutorial-3", false, -1,-1,-1,-1);
                    correct = 0;
                    GameObject sc1 = GameObject.Find("arrow1");
                    sc1.gameObject.GetComponent<Renderer>().enabled = false;
                    GameObject sc2 = GameObject.Find("arrow2");
                    sc2.gameObject.GetComponent<Renderer>().enabled = false;

                    if (gameController.saveLevel == 1)
                    {
                        GameObject sc3 = GameObject.Find("arrow_drag");
                        sc3.gameObject.GetComponent<Renderer>().enabled = true;
                    }
                    else if ((gameController.saveLevel == 2) || (gameController.saveLevel == 3) || (gameController.saveLevel == 4))
                    {
                        GameObject sc3 = GameObject.Find("arrow_drag2");
                        sc3.gameObject.GetComponent<Renderer>().enabled = true;
                    }
                }
                if ((step == 3) && (correct == 1)) 
                {
                    //teach them about the trash
                    gameController.SendRobotUtterance("tutorial-4", false, -1,-1,-1,-1);
                    correct = 0;
                    GameObject sc3 = GameObject.Find("arrow_drag");
                    sc3.gameObject.GetComponent<Renderer>().enabled = false;
                    GameObject sc4 = GameObject.Find("arrow_drag2");
                    sc4.gameObject.GetComponent<Renderer>().enabled = false;
                    GameObject sc1 = GameObject.Find("arrow_trash");
                    sc1.gameObject.GetComponent<Renderer>().enabled = true;
                }
                if ((step == 4) && (correct == 1)) // body and cabin
                {
                    gameController.SendRobotUtterance("tutorial-5", false, -1,-1,-1,-1);
                    correct = 0;
                    deleteWrongChildren(0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0);
                    GameObject sc1 = GameObject.Find("arrow_trash");
                    sc1.gameObject.GetComponent<Renderer>().enabled = false;
                    GameObject sc2 = GameObject.Find("t_sel_body");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc3 = GameObject.Find("t_sel_cabin");
                    sc3.gameObject.GetComponent<Image>().enabled = true;

                }
                if ((step == 5) && (correct == 1)) // body and cabin
                {
                    correct = 0;

                }
                if ((step == 6) && (correct == 1)) // wheel1 and wheel2
                {
                    gameController.SendRobotUtterance("tutorial-6", false, -1,-1,-1,-1);
                    correct = 0;
                    deleteWrongChildren(0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0);
                    GameObject sc1 = GameObject.Find("t_sel_wheel1");
                    sc1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject sc2 = GameObject.Find("t_sel_wheel2");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                }
                if ((step == 7) && (correct == 1)) // wheel1 and wheel2
                {
                    GameObject sc2 = GameObject.Find("t_sel_wheel2");
                    sc2.gameObject.GetComponent<Image>().enabled = true;
                    correct = 0;

                }
                if ((step == 8) && (correct == 1)) // smoke1
                {
                    gameController.SendRobotUtterance("tutorial-7", false, -1,-1,-1,-1);
                    correct = 0;
                    deleteWrongChildren(0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
                    GameObject sc1 = GameObject.Find("t_sel_smoke1");
                    sc1.gameObject.GetComponent<Image>().enabled = true;

                }
                if ((step == 9) && (correct == 1))
                {
                    correct = 0;
                    gameController.SendRobotUtterance("tutorial-8", false, -1,-1,-1,-1);

                }
            }

        }

        void deleteWrongChildren(int front, int body, int cabin, int back, int s1, int s2, int s3, int w1, int w2, int w3, int w4)
        {
            if (front == 1)
            {
                GameObject o = GameObject.Find("s_front");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_front_pieces--;
                    GameObject.Destroy(child.gameObject);
                }

            }
            if (body == 1)
            {
                GameObject o = GameObject.Find("s_body");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_body_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
                GameObject ot = GameObject.Find("t_s_body");
                foreach (Transform child in ot.transform)
                {
                    childTrain.n_body_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (cabin == 1)
            {
                GameObject o = GameObject.Find("s_cabin");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_cabin_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
                GameObject ot = GameObject.Find("t_s_cabin");
                foreach (Transform child in ot.transform)
                {
                    childTrain.n_cabin_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (back == 1)
            {
                GameObject o = GameObject.Find("s_back");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_back_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (s1 == 1)
            {
                GameObject o = GameObject.Find("s_smoke1");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_smoke_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
                GameObject ot = GameObject.Find("t_s_smoke1");
                foreach (Transform child in ot.transform)
                {
                    childTrain.n_smoke_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (s2 == 1)
            {
                GameObject o = GameObject.Find("s_smoke2");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_smoke_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (s3 == 1)
            {
                GameObject o = GameObject.Find("s_smoke3");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_smoke_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (w1 == 1)
            {
                GameObject o = GameObject.Find("s_wheel1");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_wheel_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
                GameObject ot = GameObject.Find("t_s_wheel1");
                foreach (Transform child in ot.transform)
                {
                    childTrain.n_wheel_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (w2 == 1)
            {
                GameObject o = GameObject.Find("s_wheel2");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_wheel_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
                GameObject ot = GameObject.Find("t_s_wheel2");
                foreach (Transform child in ot.transform)
                {
                    childTrain.n_wheel_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (w3 == 1)
            {
                GameObject o = GameObject.Find("s_wheel3");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_wheel_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
            if (w4 == 1)
            {
                GameObject o = GameObject.Find("s_wheel4");
                foreach (Transform child in o.transform)
                {
                    childTrain.n_wheel_pieces--;
                    GameObject.Destroy(child.gameObject);
                }
            }
        }


        void CreateRandomTrain()
        {
            int f = 0; int b = 0; int c = 0; int bd = 0; int s1 = 0; int s2 = 0;int s3 = 0;int w1 = 0;int w2 = 0;int w3 = 0;int w4=0;
            System.Random rnd = new System.Random();
            if (levelNumber == 1)
            {
                f = rnd.Next(0, 1);
                b = rnd.Next(0, 1);
                c = rnd.Next(0, 1);
                bd = rnd.Next(0, 1);
                s1 = rnd.Next(0, 2);
                s2 = rnd.Next(0, 2);
                s3 = rnd.Next(0, 2);
                w1 = rnd.Next(0, 2);
                w2 = rnd.Next(0, 2);
                w3 = rnd.Next(0, 2);
                w4 = rnd.Next(0, 2);
            }
            if (levelNumber == 2)
            {
                f = rnd.Next(0, 1);
                b = rnd.Next(0, 1);
                c = rnd.Next(0, 1);
                bd = rnd.Next(0, 1);
                s1 = rnd.Next(0, 4);
                s2 = rnd.Next(0, 4);
                s3 = rnd.Next(0, 4);
                w1 = rnd.Next(0, 3);
                w2 = rnd.Next(0, 3);
                w3 = rnd.Next(0, 3);
                w4 = rnd.Next(0, 3);
            }
            if (levelNumber == 3)
            {
                f = rnd.Next(0, 2);
                b = rnd.Next(0, 2);
                c = rnd.Next(0, 2);
                bd = rnd.Next(0, 2);
                s1 = rnd.Next(0, 5);
                s2 = rnd.Next(0, 5);
                s3 = rnd.Next(0, 5);
                w1 = rnd.Next(0, 4);
                w2 = rnd.Next(0, 4);
                w3 = rnd.Next(0, 4);
                w4 = rnd.Next(0, 4);
            }
            if (levelNumber == 4)
            {
                f = rnd.Next(0, 2);
                b = rnd.Next(0, 2);
                c = rnd.Next(0, 2);
                bd = rnd.Next(0, 2);
                s1 = rnd.Next(0, 7);
                s2 = rnd.Next(0, 7);
                s3 = rnd.Next(0, 7);
                w1 = rnd.Next(0, 5);
                w2 = rnd.Next(0, 5);
                w3 = rnd.Next(0, 5);
                w4 = rnd.Next(0, 5);
            }
            robotTrain.front = f;
            robotTrain.back = b;
            robotTrain.cabin = c;
            robotTrain.body = bd;
            robotTrain.smoke1 = s1;
            robotTrain.smoke2 = s2;
            robotTrain.smoke3 = s3;
            robotTrain.wheel1 = w1;
            robotTrain.wheel2 = w2;
            robotTrain.wheel3 = w3;
            robotTrain.wheel4 = w4;
        }

        void TrainInCorner()
        {
            var f = GameObject.Find("o_front");
            var bd = GameObject.Find("o_body");
            var c = GameObject.Find("o_cabin");
            var b = GameObject.Find("o_back");
            var w1 = GameObject.Find("o_wheel1");
            var w2 = GameObject.Find("o_wheel2");
            var w3 = GameObject.Find("o_wheel3");
            var w4 = GameObject.Find("o_wheel4");
            var s1 = GameObject.Find("o_smoke1");
            var s2 = GameObject.Find("o_smoke2");
            var s3 = GameObject.Find("o_smoke3");

            GameObject front = Instantiate(gameController.frontPieceOptions[robotTrain.front]);
            GameObject body = Instantiate(gameController.bodyPieceOptions[robotTrain.body]);
            GameObject cabin = Instantiate(gameController.cabinPieceOptions[robotTrain.cabin]);
            GameObject back = Instantiate(gameController.backPieceOptions[robotTrain.back]);
            GameObject wheel1 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel1]);
            GameObject wheel2 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel2]);
            GameObject wheel3 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel3]);
            GameObject wheel4 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel4]);
            GameObject smoke1 = Instantiate(gameController.smokePieceOptions[robotTrain.smoke1]);
            GameObject smoke2 = Instantiate(gameController.smokePieceOptions[robotTrain.smoke2]);
            GameObject smoke3 = Instantiate(gameController.smokePieceOptions[robotTrain.smoke3]);

            front.transform.SetParent(f.transform, false);
            body.transform.SetParent(bd.transform, false);
            cabin.transform.SetParent(c.transform, false);
            back.transform.SetParent(b.transform, false);
            wheel1.transform.SetParent(w1.transform, false);
            wheel2.transform.SetParent(w2.transform, false);
            wheel3.transform.SetParent(w3.transform, false);
            wheel4.transform.SetParent(w4.transform, false);
            smoke1.transform.SetParent(s1.transform, false);
            smoke2.transform.SetParent(s2.transform, false);
            smoke3.transform.SetParent(s3.transform, false);

        }

        void TrainInCornerTutorial()
        {
            var bd = GameObject.Find("t_o_body");
            var c = GameObject.Find("t_o_cabin");
            var w1 = GameObject.Find("t_o_wheel1");
            var w2 = GameObject.Find("t_o_wheel2");        
            var s1 = GameObject.Find("t_o_smoke1");
          
            GameObject body = Instantiate(gameController.bodyPieceOptions[0]);
            GameObject cabin = Instantiate(gameController.cabinPieceOptions[0]); 
            GameObject wheel1 = Instantiate(gameController.wheelPieceOptions[0]);
            GameObject wheel2 = Instantiate(gameController.wheelPieceOptions[1]);
            GameObject smoke1 = Instantiate(gameController.smokePieceOptions[1]);

            body.transform.SetParent(bd.transform, false);
            cabin.transform.SetParent(c.transform, false);
            wheel1.transform.SetParent(w1.transform, false);
            wheel2.transform.SetParent(w2.transform, false);
            smoke1.transform.SetParent(s1.transform, false);

        }

        public void ExplainerFinishedBuilding () {

			// switch the explainer mode
			gameController.explainerMode = Constants.EXPLAINER_EXPLAINING;

			// disable the drag and drop functionality
			DisableDragAndDropGameplay ();

			// clear scene variables
			ClearSceneVariables ();

			//unsbuscribe from appropriate events
			Slot.OnPieceAddedToTrain -= ExplainerPieceAddedToTrain;
			DragHandler.OnPieceRemovedByTrash -= ExplainerPieceRemoved;

            FinishBuilding();

        }

		void ExplainerPieceAddedToTrain (GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex) {
			if (IsHouseComplete ()) {
				explainerCompleteButton.SetActive (true);
			} else {
				explainerCompleteButton.SetActive (false);
			}
		}

		void ExplainerPieceRemoved (GameObject pieceToRemove, int pieceType, int oldParentType, int oldParentOutlineIndex) {
			if (IsHouseComplete ()) {
				explainerCompleteButton.SetActive (true);
			} else {
				explainerCompleteButton.SetActive (false);
			}
		}

		public void FinishBuilding () {
            firstTimeIn = true;
            if (gameController.internalGameState == Constants.END_GAME)
            {
                gameController.SendRobotUtterance("end-game", false, -1,-1,-1,-1);
                //Logger.Log("GoingToQuit");
                Application.Quit();
            }

            if (!gameController.tutorial)
            {
                EraseAllBuilder();
                ShowBothTrains();
                showOnScreen();
                CorrectTrain();
                NewGameButton.SetActive(true);
            }
            else
            {
                gameController.levelNumber = gameController.saveLevel;
                levelNumber = gameController.saveLevel;
                gameController.tutorial = false;
                HideTutorialPieces();
                ShowBuilderPieces();
                BothTrainsNewGame();
                //Destroy(GameObject.Find("ExplainerSceneManager"));
                //SceneManager.LoadScene("Start_menu");
            }

            /*if (!gameController.tutorial)
            {
                SceneManager.LoadScene("Both_houses_L1");
            }
            else
            {
                gameController.levelNumber = gameController.saveLevel;
                gameController.tutorial = false;
                Destroy(GameObject.Find("ExplainerSceneManager"));
                SceneManager.LoadScene("Start_menu");
            }*/
        }




        public void ClearSceneVariables()
        {

            // dashed outline slots
            dashedFrontOutlineSlots.Clear();
            dashedBodyOutlineSlot.Clear();
            dashedCabinOutlineSlots.Clear();
            dashedBackOutlineSlots.Clear();
            dashedSmokeOutlineSlots.Clear();
            dashedWheelOutlineSlots.Clear();

            // panel pieces
            frontPanelTrainPieces.Clear();
            bodyPanelTrainPieces.Clear();
            cabinPanelTrainPieces.Clear();
            backPanelTrainPieces.Clear();
            smokePanelTrainPieces.Clear();
            wheelPanelTrainPieces.Clear();

        }


        void HidePieces(List<GameObject> pieces)
        {
            foreach (GameObject piece in pieces)
            {
                piece.GetComponent<Image>().enabled = false;
            }
        }

        public bool IsHouseComplete()
        {
            //Logger.Log(childTrain.n_back_pieces);
            //Logger.Log(childTrain.n_body_pieces);
            if ((childTrain.n_back_pieces == 1) && (childTrain.n_body_pieces == 1) && (childTrain.n_front_pieces == 1) && (childTrain.n_cabin_pieces == 1) && (childTrain.n_smoke_pieces == 3) && (childTrain.n_wheel_pieces == 4) && (step==12))
            {
                if (gameController.internalGameState == Constants.END_GAME)
                {
                    // send goobye message
                    gameController.SendRobotUtterance("end", true, -1, -1, -1, -1);
                    Application.Quit();
                }
                return true;
            }
            else if (gameController.tutorial)
            {
                if ((childTrain.n_body_pieces == 1) && (childTrain.n_cabin_pieces == 1) && (childTrain.n_wheel_pieces == 2) && (childTrain.n_smoke_pieces == 1))
                {
                    gameController.SendRobotUtterance("tutorial-8", false, -1, -1, -1, -1);
                    if (gameController.internalGameState == Constants.END_GAME)
                    {
                        // send goobye message
                        gameController.SendRobotUtterance("end", true, -1, -1, -1, -1);
                        Application.Quit();
                    }
                    return true;
                }
            }
            return false;

        }
        void HideSelectedFrontPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject f = GameObject.Find("s_front");
                f.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedFrontPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject f = GameObject.Find("s_front");
                f.gameObject.GetComponent<Image>().enabled = true;
            }
        }

        void HideSelectedBodyPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject b = GameObject.Find("s_body");
                b.gameObject.GetComponent<Image>().enabled = false;
            }
            else
            {
                GameObject b = GameObject.Find("t_s_body");
                b.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedBodyPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject b = GameObject.Find("s_body");
                b.gameObject.GetComponent<Image>().enabled = true;
            }
            else
            {
                GameObject b = GameObject.Find("t_s_body");
                b.gameObject.GetComponent<Image>().enabled = true;
            }
        }

        void HideSelectedCabinPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject c = GameObject.Find("s_cabin");
                c.gameObject.GetComponent<Image>().enabled = false;
            }
            else
            {
                GameObject c = GameObject.Find("t_s_cabin");
                c.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedCabinPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject c = GameObject.Find("s_cabin");
                c.gameObject.GetComponent<Image>().enabled = true;
            }
            else
            {
                GameObject c = GameObject.Find("t_s_cabin");
                c.gameObject.GetComponent<Image>().enabled = true;
            }
        }

        void HideSelectedBackPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject b = GameObject.Find("s_back");
                b.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedBackPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject b = GameObject.Find("s_back");
                b.gameObject.GetComponent<Image>().enabled = true;
            }
        }

        void HideSelectedSmokePieces()
        {
            if (!gameController.tutorial)
            {
                GameObject s1 = GameObject.Find("s_smoke1");
                s1.gameObject.GetComponent<Image>().enabled = false;
                    GameObject s2 = GameObject.Find("s_smoke2");
                    GameObject s3 = GameObject.Find("s_smoke3");
                    s2.gameObject.GetComponent<Image>().enabled = false;
                    s3.gameObject.GetComponent<Image>().enabled = false;
            }
            else
            {
                GameObject s1 = GameObject.Find("t_s_smoke1");
                s1.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedSmokePieces()
        {
            if (!gameController.tutorial)
            {
                GameObject s1 = GameObject.Find("s_smoke1");
                s1.gameObject.GetComponent<Image>().enabled = true;
                    GameObject s2 = GameObject.Find("s_smoke2");
                    GameObject s3 = GameObject.Find("s_smoke3");
                    s2.gameObject.GetComponent<Image>().enabled = true;
                    s3.gameObject.GetComponent<Image>().enabled = true;
            }
            else
            {
                GameObject s1 = GameObject.Find("t_s_smoke1");
                s1.gameObject.GetComponent<Image>().enabled = true;
            }
        }

        void HideSelectedWheelPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject w1 = GameObject.Find("s_wheel1");
                GameObject w2 = GameObject.Find("s_wheel2");
                w1.gameObject.GetComponent<Image>().enabled = false;
                w2.gameObject.GetComponent<Image>().enabled = false;
                    GameObject w3 = GameObject.Find("s_wheel3");
                    GameObject w4 = GameObject.Find("s_wheel4");
                    w3.gameObject.GetComponent<Image>().enabled = false;
                    w4.gameObject.GetComponent<Image>().enabled = false;
            }
            else
            {
                GameObject w1 = GameObject.Find("t_s_wheel1");
                GameObject w2 = GameObject.Find("t_s_wheel2");
                w1.gameObject.GetComponent<Image>().enabled = false;
                w2.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedWheelPieces()
        {
            if (!gameController.tutorial)
            {
                GameObject w1 = GameObject.Find("s_wheel1");
                GameObject w2 = GameObject.Find("s_wheel2");
                w1.gameObject.GetComponent<Image>().enabled = true;
                w2.gameObject.GetComponent<Image>().enabled = true;
                GameObject w3 = GameObject.Find("s_wheel3");
                GameObject w4 = GameObject.Find("s_wheel4");
                w3.gameObject.GetComponent<Image>().enabled = true;
                w4.gameObject.GetComponent<Image>().enabled = true;
            }
            else
            {
                GameObject w1 = GameObject.Find("t_s_wheel1");
                GameObject w2 = GameObject.Find("t_s_wheel2");
                w1.gameObject.GetComponent<Image>().enabled = true;
                w2.gameObject.GetComponent<Image>().enabled = true;
            }
        }


        // This is our cue to hide the old pieces on the panels and show the new ones
        void PanelIn()
        {
            //lastPieceTypeSelected = Constants.NONE_SELECTED;
            // hide the rocket pieces of the old selected piece type
            if (lastPieceTypeSelected == Constants.FRONT)
            {
                HideSelectedFrontPieces();
                HidePieces(frontPanelTrainPieces);
            }
            else if (lastPieceTypeSelected == Constants.BODY)
            {
                HideSelectedBodyPieces();
                HidePieces(bodyPanelTrainPieces);
            }
            else if (lastPieceTypeSelected == Constants.CABIN)
            {
                HideSelectedCabinPieces();
                HidePieces(cabinPanelTrainPieces);
            }
            else if (lastPieceTypeSelected == Constants.BACK)
            {
                HideSelectedBackPieces();
                HidePieces(backPanelTrainPieces);
            }
            else if (lastPieceTypeSelected == Constants.WHEEL)
            {
                HideSelectedWheelPieces();
                HidePieces(wheelPanelTrainPieces);
            }
            else if (lastPieceTypeSelected == Constants.SMOKE)
            {
                HideSelectedSmokePieces();
                HidePieces(smokePanelTrainPieces);
            }

            // show the rocket pieces of the new selected piece type
            if (currentPieceTypeSelected == Constants.FRONT)
            {
                ShowSelectedFrontPieces();
                ShowPieces(frontPanelTrainPieces);
            }
            else if (currentPieceTypeSelected == Constants.BODY)
            {
                ShowSelectedBodyPieces();
                ShowPieces(bodyPanelTrainPieces);
            }
            else if (currentPieceTypeSelected == Constants.CABIN)
            {
                ShowSelectedCabinPieces();
                ShowPieces(cabinPanelTrainPieces);
            }
            else if (currentPieceTypeSelected == Constants.BACK)
            {
                ShowSelectedBackPieces();
                ShowPieces(backPanelTrainPieces);
            }
            else if (currentPieceTypeSelected == Constants.WHEEL)
            {
                if ((gameController.tutorial) && (step == 1))
                {
                    correct = 1;
                    step++;
                }
                ShowSelectedWheelPieces();
                ShowPieces(wheelPanelTrainPieces);
            }
            else if (currentPieceTypeSelected == Constants.SMOKE)
            {
                ShowSelectedSmokePieces();
                ShowPieces(smokePanelTrainPieces);
            }

            // indicate activity
            gameController.recentScreenInteraction = true;
        }

        void PieceAddedToPanel(GameObject pieceAdded)
        {
            if (pieceAdded.tag == "Front")
            {
                frontPanelTrainPieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Body")
            {
                bodyPanelTrainPieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Cabin")
            {
                cabinPanelTrainPieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Back")
            {
                backPanelTrainPieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Smoke")
            {
                smokePanelTrainPieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Wheel")
            {
                wheelPanelTrainPieces.Add(pieceAdded);
            }

            // indicate activity
            gameController.recentScreenInteraction = true;
        }

        void PieceAddedToTrain(GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex)
        {

            // if we've moved a piece within the rocket, we must remove it from its previously stored location
            if (oldParentType == Constants.PARENT_TRAIN_OUTLINE)
            {
                if (pieceType == Constants.FRONT)
                {
                    childTrain.frontPieces[oldParentOutlineIndex] = null;
                }
                else if (pieceType == Constants.BODY)
                {
                    childTrain.bodyPieces[oldParentOutlineIndex] = null;
                }
                else if (pieceType == Constants.CABIN)
                {
                    childTrain.cabinPieces[oldParentOutlineIndex] = null;
                }
                else if (pieceType == Constants.BACK)
                {
                    childTrain.backPieces[oldParentOutlineIndex] = null;
                }
                /*else if (pieceType == Constants.SMOKE)
                {
                    childTrain.smokePieces[oldParentOutlineIndex] = null;
                }
                else if (pieceType == Constants.WHEEL)
                {
                    childTrain.wheelPieces[oldParentOutlineIndex] = null;
                }*/
            }
            // if the piece added to the rocket was taken from the panel, remove it from the panel pieces list
            // and remove the piece from the current lists of rocket pieces
            else
            {
                int removalIndex = -1;
                if (pieceType == Constants.FRONT)
                {
                    for (int i = 0; i < frontPanelTrainPieces.Count; i++)
                    {
                        if (pieceAdded.name == frontPanelTrainPieces[i].name)
                        {
                            childTrain.n_front_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    frontPanelTrainPieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.BODY)
                {
                    for (int i = 0; i < bodyPanelTrainPieces.Count; i++)
                    {
                        if (pieceAdded.name == bodyPanelTrainPieces[i].name)
                        {
                            childTrain.n_body_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    bodyPanelTrainPieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.CABIN)
                {
                    for (int i = 0; i < cabinPanelTrainPieces.Count; i++)
                    {
                        if (pieceAdded.name == cabinPanelTrainPieces[i].name)
                        {
                            childTrain.n_cabin_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    cabinPanelTrainPieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.BACK)
                {
                    for (int i = 0; i < backPanelTrainPieces.Count; i++)
                    {
                        if (pieceAdded.name == backPanelTrainPieces[i].name)
                        {
                            childTrain.n_back_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    backPanelTrainPieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.WHEEL)
                {
                    for (int i = 0; i < wheelPanelTrainPieces.Count; i++)
                    {
                        if (pieceAdded.name == wheelPanelTrainPieces[i].name)
                        {
                            childTrain.n_wheel_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    wheelPanelTrainPieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.SMOKE)
                {
                    for (int i = 0; i < smokePanelTrainPieces.Count; i++)
                    {
                        if (pieceAdded.name == smokePanelTrainPieces[i].name)
                        {
                            childTrain.n_smoke_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    smokePanelTrainPieces.RemoveAt(removalIndex);
                }
            }

            // add the piece added to the new location in the appropriate rocket pieces list

            ////////////////////////////////////////////////////////////
            //////////////     LEVEL 1 /////////////////////////////////
            ////////////////////////////////////////////////////////////
            if (levelNumber ==1)
            {
                if (pieceType == Constants.FRONT)
                {

                    GameObject frontPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject frontPrefab in gameController.frontPieceOptions)
                    {
                        if (pieceAdded.name.Contains(frontPrefab.name))
                        {
                            if (!gameController.tutorial)
                            {
                                if (step == 1)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                            }
                            childTrain.order_front = order;
                            order++;
                            childTrain.front = i;
                            frontPrefabAdded = frontPrefab;
                            GameObject sc = GameObject.Find("sel_front");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        i++;
                    }
                    //childTrain.frontPieces[newParentOutlineIndex] = frontPrefabAdded;
                }
                else if (pieceType == Constants.BODY)
                {
                    GameObject bodyPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject bodyPrefab in gameController.bodyPieceOptions)
                    {
                        if (pieceAdded.name.Contains(bodyPrefab.name))
                        {
                            if (!gameController.tutorial)
                            {
                                if (step == 2)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                            }
                            else
                            {
                                if (step == 4)
                                {
                                    correct = 1;
                                    step++;
                                }
                                else if (step == 5)
                                {
                                    // say that it was done on the wrong step
                                    gameController.SendRobotUtterance("tutorial-5-wrong", false, -1,-1,-1,-1);
                                    correct = 1;
                                    step++;

                                }
                                else
                                {
                                    // say that it was done on the wrong step
                                }
                            }
                            childTrain.order_body = order;
                            order++;
                            childTrain.body = i;
                            bodyPrefabAdded = bodyPrefab;
                            if (!gameController.tutorial)
                            {
                                GameObject sc = GameObject.Find("sel_body");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            else
                            {
                                GameObject sc = GameObject.Find("t_sel_body");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            
                            break;
                        }
                        i++;
                    }
                    //childTrain.bodyPieces[newParentOutlineIndex] = bodyPrefabAdded;
                }
                else if (pieceType == Constants.CABIN)
                {
                    GameObject cabinPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject cabinPrefab in gameController.cabinPieceOptions)
                    {
                        if (pieceAdded.name.Contains(cabinPrefab.name))
                        {
                            if (!gameController.tutorial)
                            {
                                if (step == 3)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                            }
                            else
                            {
                                if (step == 5)
                                {
                                    correct = 1;
                                    step++;
                                }
                                else if (step == 4)
                                {
                                    // say that it was done on the wrong step
                                    gameController.SendRobotUtterance("tutorial-5-wrong", false, -1,-1,-1,-1);
                                    correct = 1;
                                    step++;

                                }
                                else
                                {
                                    // say that it was done on the wrong step
                                }
                            }
                            childTrain.order_cabin = order;
                            order++;
                            childTrain.cabin = i;
                            cabinPrefabAdded = cabinPrefab;
                            if (!gameController.tutorial)
                            {
                                GameObject sc = GameObject.Find("sel_cabin");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            else
                            {
                                GameObject sc = GameObject.Find("t_sel_cabin");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            break;
                        }


                        i++;
                    }
                    //childTrain.cabinPieces[newParentOutlineIndex] = cabinPrefabAdded;
                }
                else if (pieceType == Constants.BACK)
                {
                    int i = 0;
                    GameObject backPrefabAdded = null;
                    foreach (GameObject backPrefab in gameController.backPieceOptions)
                    {
                        if (pieceAdded.name.Contains(backPrefab.name))
                        {
                            if (step == 4)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_back = order;
                            order++;
                            childTrain.back = i;
                            backPrefabAdded = backPrefab;
                            GameObject sc = GameObject.Find("sel_back");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.backPieces[newParentOutlineIndex] = backPrefabAdded;
                }

                else if (pieceType == Constants.WHEEL)
                {
                    int i = 0;
                    GameObject wheelPrefabAdded = null;
                    foreach (GameObject wheelPrefab in gameController.wheelPieceOptions)
                    {
                        if (pieceAdded.name.Contains(wheelPrefab.name))
                        {
                            if ((newParentOutlineIndex == 0))
                            {
                                if (!gameController.tutorial)
                                {
                                    if (step == 8)
                                    {
                                        if (rightFirst == 1)
                                        {
                                            score += 1;
                                        }
                                        correct = 1;
                                        step = step + 1;
                                    }
                                    else
                                    {
                                        rightFirst = -1;
                                    }
                                }
                                else
                                {
                                    if (step == 2)
                                    {
                                        correct = 1;
                                        step++;
                                    }
                                    if (step == 6)
                                    {
                                        correct = 1;
                                        step++;
                                    }
                                    else if (step == 7)
                                    {
                                        // say that it was done on the wrong step
                                        //gameController.SendRobotUtterance("tutorial-6-wrong", false, -1,-1,-1,-1);
                                        correct = 1;
                                        step++;

                                    }
                                    else
                                    {
                                        // say that it was done on the wrong step
                                    }
                                }
                                childTrain.order_wheel1 = order;
                                order++;
                                childTrain.wheel1 = i;
                                if (!gameController.tutorial)
                                {
                                    GameObject sc = GameObject.Find("sel_wheel1");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                                else
                                {
                                    GameObject sc = GameObject.Find("t_sel_wheel1");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                            }
                            if ((newParentOutlineIndex == 1))
                            {
                                if (!gameController.tutorial)
                                {
                                    if (step == 9)
                                    {
                                        if (rightFirst == 1)
                                        {
                                            score += 1;
                                        }
                                        correct = 1;
                                        step = step + 1;
                                    }
                                    else
                                    {
                                        rightFirst = -1;
                                    }
                                }
                                else
                                {
                                    if (step == 7)
                                    {
                                        correct = 1;
                                        step++;
                                    }
                                    else if (step == 6)
                                    {
                                        // say that it was done on the wrong step
                                        //gameController.SendRobotUtterance("tutorial-6-wrong", false, -1,-1,-1,-1);
                                        correct = 1;
                                        step++;

                                    }
                                    else
                                    {
                                        // say that it was done on the wrong step
                                    }
                                }
                                childTrain.order_wheel2 = order;
                                order++;
                                childTrain.wheel2 = i;
                                GameObject sc = GameObject.Find("sel_wheel2");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 10)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel3 = order;
                                order++;
                                childTrain.wheel3 = i;
                                GameObject sc = GameObject.Find("sel_wheel3");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 3)
                            {
                                if (!gameController.tutorial)
                                {
                                    if (step == 11)
                                    {
                                        if (rightFirst == 1)
                                        {
                                            score += 1;
                                        }
                                        correct = 1;
                                        step = step + 1;
                                    }
                                    else
                                    {
                                        rightFirst = -1;
                                    }
                                }
                                else
                                {
                                    if (step == 2)
                                    {
                                        correct = 1;
                                        step++;
                                    }
                                    if (step == 6)
                                    {
                                        correct = 1;
                                        step++;
                                    }
                                    else if (step == 7)
                                    {
                                        // say that it was done on the wrong step
                                        //gameController.SendRobotUtterance("tutorial-6-wrong", false, -1, -1, -1, -1);
                                        correct = 1;
                                        step++;

                                    }
                                }
                                childTrain.order_wheel4 = order;
                                order++;
                                childTrain.wheel4 = i;
                                if (!gameController.tutorial)
                                {
                                    GameObject sc = GameObject.Find("sel_wheel4");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                                else
                                {
                                    GameObject sc = GameObject.Find("t_sel_wheel1");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                    GameObject sc2 = GameObject.Find("t_sel_wheel2");
                                    sc2.gameObject.GetComponent<Image>().enabled = false;
                                }
                            }
                            wheelPrefabAdded = wheelPrefab;
                            break;
                        }
                        ++i;
                    }
                    // childTrain.wheelPieces[newParentOutlineIndex] = wheelPrefabAdded;
                }

                else if (pieceType == Constants.SMOKE)
                {
                    int i = 0;
                    GameObject smokePrefabAdded = null;
                    foreach (GameObject smokePrefab in gameController.smokePieceOptions)
                    {
                        if (pieceAdded.name.Contains(smokePrefab.name))
                        {
                            if (newParentOutlineIndex == 0)
                            {
                                if (!gameController.tutorial)
                                {
                                    if (step == 5)
                                    {
                                        if (rightFirst == 1)
                                        {
                                            score += 1;
                                        }
                                        correct = 1;
                                        step = step + 1;
                                    }
                                }
                                else
                                {
                                    if (step == 8)
                                    {
                                        correct = 1;
                                        step++;
                                    }
                                }
                                childTrain.order_smoke1 = order;
                                order++;
                                childTrain.smoke1 = i;
                                if (!gameController.tutorial)
                                {
                                    GameObject sc = GameObject.Find("sel_smoke1");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                                else
                                {
                                    GameObject sc = GameObject.Find("t_sel_smoke1");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                            }
                            if (newParentOutlineIndex == 1)
                            {
                                if (step == 6)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                childTrain.order_smoke2 = order;
                                order++;
                                childTrain.smoke2 = i;
                                if (!gameController.tutorial)
                                {
                                    GameObject sc = GameObject.Find("sel_smoke2");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                                else
                                {
                                    GameObject sc = GameObject.Find("t_sel_smoke2");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 7)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                childTrain.order_smoke3 = order;
                                order++;
                                childTrain.smoke3 = i;
                                if (!gameController.tutorial)
                                {
                                    GameObject sc = GameObject.Find("sel_smoke3");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                                else
                                {
                                    GameObject sc = GameObject.Find("t_sel_smoke1");
                                    sc.gameObject.GetComponent<Image>().enabled = false;
                                }
                            }
                            smokePrefabAdded = smokePrefab;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.smokePieces[newParentOutlineIndex] = smokePrefabAdded;

                }
            }

            ////////////////////////////////////////////////////////////
            //////////////     LEVEL 2 /////////////////////////////////
            ////////////////////////////////////////////////////////////
            if (levelNumber == 2)
            {
                if (pieceType == Constants.FRONT)
                {

                    GameObject frontPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject frontPrefab in gameController.frontPieceOptions)
                    {
                        if (pieceAdded.name.Contains(frontPrefab.name))
                        {
                            if (step == 7)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if (step == 8)
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_front = order;
                            order++;
                            childTrain.front = i;
                            frontPrefabAdded = frontPrefab;
                            GameObject sc = GameObject.Find("sel_front");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        i++;
                    }
                    //childTrain.frontPieces[newParentOutlineIndex] = frontPrefabAdded;
                }
                else if (pieceType == Constants.BODY)
                {
                    GameObject bodyPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject bodyPrefab in gameController.bodyPieceOptions)
                    {
                        if (pieceAdded.name.Contains(bodyPrefab.name))
                        {
                            if (step == 1)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if (step == 2)
                            {
                                step++;
                                correct = 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_body = order;
                            order++;
                            childTrain.body = i;
                            bodyPrefabAdded = bodyPrefab;
                            GameObject sc = GameObject.Find("sel_body");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        i++;
                    }
                    //childTrain.bodyPieces[newParentOutlineIndex] = bodyPrefabAdded;
                }
                else if (pieceType == Constants.CABIN)
                {
                    GameObject cabinPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject cabinPrefab in gameController.cabinPieceOptions)
                    {
                        if (pieceAdded.name.Contains(cabinPrefab.name))
                        {
                            if (step == 1)
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else if (step == 2)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_cabin = order;
                            order++;
                            childTrain.cabin = i;
                            cabinPrefabAdded = cabinPrefab;
                            GameObject sc = GameObject.Find("sel_cabin");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }


                        i++;
                    }
                    //childTrain.cabinPieces[newParentOutlineIndex] = cabinPrefabAdded;
                }
                else if (pieceType == Constants.BACK)
                {
                    int i = 0;
                    GameObject backPrefabAdded = null;
                    foreach (GameObject backPrefab in gameController.backPieceOptions)
                    {
                        if (pieceAdded.name.Contains(backPrefab.name))
                        {
                            if (step == 8)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if (step == 7)
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_back = order;
                            order++;
                            childTrain.back = i;
                            backPrefabAdded = backPrefab;
                            GameObject sc = GameObject.Find("sel_back");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.backPieces[newParentOutlineIndex] = backPrefabAdded;
                }

                else if (pieceType == Constants.WHEEL)
                {
                    int i = 0;
                    GameObject wheelPrefabAdded = null;
                    foreach (GameObject wheelPrefab in gameController.wheelPieceOptions)
                    {
                        if (pieceAdded.name.Contains(wheelPrefab.name))
                        {
                            if (newParentOutlineIndex == 0)
                            {
                                if (step == 3)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if (step == 4)
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel1 = order;
                                order++;
                                childTrain.wheel1 = i;
                                GameObject sc = GameObject.Find("sel_wheel1");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 1)
                            {
                                if (step == 4)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if (step == 3)
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel2 = order;
                                order++;
                                childTrain.wheel2 = i;
                                GameObject sc = GameObject.Find("sel_wheel2");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 5)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if (step == 6)
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel3 = order;
                                order++;
                                childTrain.wheel3 = i;
                                GameObject sc = GameObject.Find("sel_wheel3");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 3)
                            {
                                if (step == 6)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if (step == 5)
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel4 = order;
                                order++;
                                childTrain.wheel4 = i;
                                GameObject sc = GameObject.Find("sel_wheel4");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            wheelPrefabAdded = wheelPrefab;
                            break;
                        }
                        ++i;
                    }
                    // childTrain.wheelPieces[newParentOutlineIndex] = wheelPrefabAdded;
                }

                else if (pieceType == Constants.SMOKE)
                {
                    int i = 0;
                    GameObject smokePrefabAdded = null;
                    foreach (GameObject smokePrefab in gameController.smokePieceOptions)
                    {
                        if (pieceAdded.name.Contains(smokePrefab.name))
                        {
                            if (newParentOutlineIndex == 0)
                            {
                                if (step == 9)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if (step == 10)
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke1 = order;
                                order++;
                                childTrain.smoke1 = i;
                                GameObject sc = GameObject.Find("sel_smoke1");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 1)
                            {
                                if (step == 10)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if (step == 9)
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke2 = order;
                                order++;
                                childTrain.smoke2 = i;
                                GameObject sc = GameObject.Find("sel_smoke2");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 11)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                childTrain.order_smoke3 = order;
                                order++;
                                childTrain.smoke3 = i;
                                GameObject sc = GameObject.Find("sel_smoke3");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            smokePrefabAdded = smokePrefab;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.smokePieces[newParentOutlineIndex] = smokePrefabAdded;

                }
            }

            ////////////////////////////////////////////////////////////
            //////////////     LEVEL 3 /////////////////////////////////
            ////////////////////////////////////////////////////////////

            if (levelNumber ==3)
            {
                if (pieceType == Constants.FRONT)
                {

                    GameObject frontPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject frontPrefab in gameController.frontPieceOptions)
                    {
                        if (pieceAdded.name.Contains(frontPrefab.name))
                        {
                            if (step == 10)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if (step == 11)
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_front = order;
                            order++;
                            childTrain.front = i;
                            frontPrefabAdded = frontPrefab;
                            GameObject sc = GameObject.Find("sel_front");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        i++;
                    }
                    //childTrain.frontPieces[newParentOutlineIndex] = frontPrefabAdded;
                }
                else if (pieceType == Constants.BODY)
                {
                    GameObject bodyPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject bodyPrefab in gameController.bodyPieceOptions)
                    {
                        if (pieceAdded.name.Contains(bodyPrefab.name))
                        {
                            if (step == 5)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if ((step == 4) || (step == 6))
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_body = order;
                            order++;
                            childTrain.body = i;
                            bodyPrefabAdded = bodyPrefab;
                            GameObject sc = GameObject.Find("sel_body");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        i++;
                    }
                    //childTrain.bodyPieces[newParentOutlineIndex] = bodyPrefabAdded;
                }
                else if (pieceType == Constants.CABIN)
                {
                    GameObject cabinPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject cabinPrefab in gameController.cabinPieceOptions)
                    {
                        if (pieceAdded.name.Contains(cabinPrefab.name))
                        {
                            if (step == 6)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if ((step == 4) || (step == 5))
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_cabin = order;
                            order++;
                            childTrain.cabin = i;
                            cabinPrefabAdded = cabinPrefab;
                            GameObject sc = GameObject.Find("sel_cabin");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }


                        i++;
                    }
                    //childTrain.cabinPieces[newParentOutlineIndex] = cabinPrefabAdded;
                }
                else if (pieceType == Constants.BACK)
                {
                    int i = 0;
                    GameObject backPrefabAdded = null;
                    foreach (GameObject backPrefab in gameController.backPieceOptions)
                    {
                        if (pieceAdded.name.Contains(backPrefab.name))
                        {
                            if (step == 11)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if ((step == 10))
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_back = order;
                            order++;
                            childTrain.back = i;
                            backPrefabAdded = backPrefab;
                            GameObject sc = GameObject.Find("sel_back");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.backPieces[newParentOutlineIndex] = backPrefabAdded;
                }

                else if (pieceType == Constants.WHEEL)
                {
                    int i = 0;
                    GameObject wheelPrefabAdded = null;
                    foreach (GameObject wheelPrefab in gameController.wheelPieceOptions)
                    {
                        if (pieceAdded.name.Contains(wheelPrefab.name))
                        {
                            if (newParentOutlineIndex == 0)
                            {
                                if (step == 1)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 2) || (step == 3))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel1 = order;
                                order++;
                                childTrain.wheel1 = i;
                                GameObject sc = GameObject.Find("sel_wheel1");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 1)
                            {
                                if (step == 2)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 1) || (step == 3))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel2 = order;
                                order++;
                                childTrain.wheel2 = i;
                                GameObject sc = GameObject.Find("sel_wheel2");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 3)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 1) || (step == 2))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel3 = order;
                                order++;
                                childTrain.wheel3 = i;
                                GameObject sc = GameObject.Find("sel_wheel3");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 3)
                            {
                                if (step == 4)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 5) || (step == 6))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel4 = order;
                                order++;
                                childTrain.wheel4 = i;
                                GameObject sc = GameObject.Find("sel_wheel4");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            wheelPrefabAdded = wheelPrefab;
                            break;
                        }
                        ++i;
                    }
                    // childTrain.wheelPieces[newParentOutlineIndex] = wheelPrefabAdded;
                }

                else if (pieceType == Constants.SMOKE)
                {
                    int i = 0;
                    GameObject smokePrefabAdded = null;
                    foreach (GameObject smokePrefab in gameController.smokePieceOptions)
                    {
                        if (pieceAdded.name.Contains(smokePrefab.name))
                        {
                            if (newParentOutlineIndex == 0)
                            {
                                if (step == 7)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 8) || (step == 9))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke1 = order;
                                order++;
                                childTrain.smoke1 = i;
                                GameObject sc = GameObject.Find("sel_smoke1");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 1)
                            {
                                if (step == 8)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 7) || (step == 9))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke2 = order;
                                order++;
                                childTrain.smoke2 = i;
                                GameObject sc = GameObject.Find("sel_smoke2");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 9)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 7) || (step == 8))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke3 = order;
                                order++;
                                childTrain.smoke3 = i;
                                GameObject sc = GameObject.Find("sel_smoke3");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            smokePrefabAdded = smokePrefab;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.smokePieces[newParentOutlineIndex] = smokePrefabAdded;

                }
            }

            ////////////////////////////////////////////////////////////
            //////////////     LEVEL 4 /////////////////////////////////
            ////////////////////////////////////////////////////////////
            if (levelNumber == 4)
            {
                if (pieceType == Constants.FRONT)
                {

                    GameObject frontPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject frontPrefab in gameController.frontPieceOptions)
                    {
                        if (pieceAdded.name.Contains(frontPrefab.name))
                        {
                            if (step == 1)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if ((step == 2) || (step == 3) || (step == 4))
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_front = order;
                            order++;
                            childTrain.front = i;
                            frontPrefabAdded = frontPrefab;
                            GameObject sc = GameObject.Find("sel_front");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        i++;
                    }
                    //childTrain.frontPieces[newParentOutlineIndex] = frontPrefabAdded;
                }
                else if (pieceType == Constants.BODY)
                {
                    GameObject bodyPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject bodyPrefab in gameController.bodyPieceOptions)
                    {
                        if (pieceAdded.name.Contains(bodyPrefab.name))
                        {
                            if (step == 2)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if ((step == 1) || (step == 3) || (step == 4))
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_body = order;
                            order++;
                            childTrain.body = i;
                            bodyPrefabAdded = bodyPrefab;
                            GameObject sc = GameObject.Find("sel_body");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        i++;
                    }
                    //childTrain.bodyPieces[newParentOutlineIndex] = bodyPrefabAdded;
                }
                else if (pieceType == Constants.CABIN)
                {
                    GameObject cabinPrefabAdded = null;
                    int i = 0;
                    foreach (GameObject cabinPrefab in gameController.cabinPieceOptions)
                    {
                        if (pieceAdded.name.Contains(cabinPrefab.name))
                        {
                            if (step == 8)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if ((step == 5) || (step == 6) || (step == 7))
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_cabin = order;
                            order++;
                            childTrain.cabin = i;
                            cabinPrefabAdded = cabinPrefab;
                            GameObject sc = GameObject.Find("sel_cabin");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }


                        i++;
                    }
                    //childTrain.cabinPieces[newParentOutlineIndex] = cabinPrefabAdded;
                }
                else if (pieceType == Constants.BACK)
                {
                    int i = 0;
                    GameObject backPrefabAdded = null;
                    foreach (GameObject backPrefab in gameController.backPieceOptions)
                    {
                        if (pieceAdded.name.Contains(backPrefab.name))
                        {
                            if (step == 11)
                            {
                                if (rightFirst == 1)
                                {
                                    score += 1;
                                }
                                correct = 1;
                                step = step + 1;
                            }
                            else if ((step == 9) ||  (step == 10))
                            {
                                correct = 1;
                                step = step + 1;
                            }
                            else
                            {
                                rightFirst = -1;
                            }
                            childTrain.order_back = order;
                            order++;
                            childTrain.back = i;
                            backPrefabAdded = backPrefab;
                            GameObject sc = GameObject.Find("sel_back");
                            sc.gameObject.GetComponent<Image>().enabled = false;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.backPieces[newParentOutlineIndex] = backPrefabAdded;
                }

                else if (pieceType == Constants.WHEEL)
                {
                    int i = 0;
                    GameObject wheelPrefabAdded = null;
                    foreach (GameObject wheelPrefab in gameController.wheelPieceOptions)
                    {
                        if (pieceAdded.name.Contains(wheelPrefab.name))
                        {
                            if (newParentOutlineIndex == 0)
                            {
                                if (step == 4)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 2) || (step == 3) || (step == 1))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel1 = order;
                                order++;
                                childTrain.wheel1 = i;
                                GameObject sc = GameObject.Find("sel_wheel1");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 1)
                            {
                                if (step == 6)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 5) || (step == 7) || (step == 8))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel2 = order;
                                order++;
                                childTrain.wheel2 = i;
                                GameObject sc = GameObject.Find("sel_wheel2");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 9)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 10) || (step == 11))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel3 = order;
                                order++;
                                childTrain.wheel3 = i;
                                GameObject sc = GameObject.Find("sel_wheel3");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 3)
                            {
                                if (step == 10)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 9) || (step == 11))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_wheel4 = order;
                                order++;
                                childTrain.wheel4 = i;
                                GameObject sc = GameObject.Find("sel_wheel4");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            wheelPrefabAdded = wheelPrefab;
                            break;
                        }
                        ++i;
                    }
                    // childTrain.wheelPieces[newParentOutlineIndex] = wheelPrefabAdded;
                }

                else if (pieceType == Constants.SMOKE)
                {
                    int i = 0;
                    GameObject smokePrefabAdded = null;
                    foreach (GameObject smokePrefab in gameController.smokePieceOptions)
                    {
                        if (pieceAdded.name.Contains(smokePrefab.name))
                        {
                            if (newParentOutlineIndex == 0)
                            {
                                if (step == 3)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 1) || (step == 2) || (step == 4))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke1 = order;
                                order++;
                                childTrain.smoke1 = i;
                                GameObject sc = GameObject.Find("sel_smoke1");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 1)
                            {
                                if (step == 5)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 6) || (step == 7) || (step == 8))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke2 = order;
                                order++;
                                childTrain.smoke2 = i;
                                GameObject sc = GameObject.Find("sel_smoke2");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            if (newParentOutlineIndex == 2)
                            {
                                if (step == 7)
                                {
                                    if (rightFirst == 1)
                                    {
                                        score += 1;
                                    }
                                    correct = 1;
                                    step = step + 1;
                                }
                                else if ((step == 5) || (step == 6) || (step == 8))
                                {
                                    correct = 1;
                                    step = step + 1;
                                }
                                else
                                {
                                    rightFirst = -1;
                                }
                                childTrain.order_smoke3 = order;
                                order++;
                                childTrain.smoke3 = i;
                                GameObject sc = GameObject.Find("sel_smoke3");
                                sc.gameObject.GetComponent<Image>().enabled = false;
                            }
                            smokePrefabAdded = smokePrefab;
                            break;
                        }
                        ++i;
                    }
                    //childTrain.smokePieces[newParentOutlineIndex] = smokePrefabAdded;

                }
            }

            if ((!gameController.tutorial) && (rightFirst != 1) && (correct == 0))
            {
                System.Random rnd = new System.Random();
                int r = rnd.Next(0, 3);
                if (r == 1)
                {
                    if (levelNumber == 1)
                    {
                        if (step == 1)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.FRONT, -1, -1, -1);
                        if (step == 2)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.BODY, -1, -1, -1);
                        if (step == 3)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.CABIN, -1, -1, -1);
                        if (step == 4)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.BACK, -1, -1, -1);
                        if (step == 5)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.SMOKE1, -1, -1, -1);
                        if (step == 6)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.SMOKE2, -1, -1, -1);
                        if (step == 7)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.SMOKE3, -1, -1, -1);
                        if (step == 8)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.WHEEL1, -1, -1, -1);
                        if (step == 9)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.WHEEL2, -1, -1, -1);
                        if (step == 10)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.WHEEL3, -1, -1, -1);
                        if (step == 11)
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.WHEEL4, -1, -1, -1);

                    }
                    if (levelNumber == 2)
                    {
                        if ((step == 1) || (step == 2))
                            gameController.SendRobotUtterance("level-2-wrong", false, Constants.BODY, Constants.CABIN, -1, -1);
                        if ((step == 3) || (step == 4))
                            gameController.SendRobotUtterance("level-2-wrong", false, Constants.WHEEL1, Constants.WHEEL2, -1, -1);
                        if ((step == 5) || (step == 6))
                            gameController.SendRobotUtterance("level-2-wrong", false, Constants.WHEEL3, Constants.WHEEL4, -1, -1);
                        if ((step == 7) || (step == 8))
                            gameController.SendRobotUtterance("level-2-wrong", false, Constants.FRONT, Constants.BACK, -1, -1);
                        if ((step == 9) || (step == 10))
                            gameController.SendRobotUtterance("level-2-wrong", false, Constants.SMOKE1, Constants.SMOKE2, -1, -1);
                        if (step == 11) 
                            gameController.SendRobotUtterance("level-1-wrong", false, Constants.SMOKE3, -1, -1, -1);
                    }
                    if (levelNumber == 3)
                    {
                        if ((step == 1) || (step == 2) || (step == 3))
                            gameController.SendRobotUtterance("level-3-wrong", false, Constants.WHEEL1, Constants.WHEEL2, Constants.WHEEL3, -1);
                        if ((step == 4) || (step == 5) || (step == 6))
                            gameController.SendRobotUtterance("level-3-wrong", false, Constants.WHEEL4, Constants.BODY, Constants.CABIN, -1);
                        if ((step == 7) || (step == 8) || (step == 9))
                            gameController.SendRobotUtterance("level-3-wrong", false, Constants.SMOKE1, Constants.SMOKE2, Constants.SMOKE3, -1);
                        if ((step == 10) || (step == 11))
                            gameController.SendRobotUtterance("level-2-wrong", false, Constants.FRONT, Constants.BACK, -1, -1);

                    }
                    if (levelNumber == 4)
                    {
                        if ((step == 1) || (step == 2) || (step == 3) || (step == 4))
                            gameController.SendRobotUtterance("level-4-wrong", false, Constants.FRONT, Constants.BODY, Constants.SMOKE1, Constants.WHEEL1);
                        if ((step == 5) || (step == 6) || (step == 7) || (step == 8))
                            gameController.SendRobotUtterance("level-4-wrong", false, Constants.SMOKE2, Constants.WHEEL2, Constants.SMOKE3, Constants.CABIN);
                        if ((step == 9) || (step == 10) || (step == 11))
                            gameController.SendRobotUtterance("level-3-wrong", false, Constants.WHEEL3, Constants.WHEEL4, Constants.BODY, -1);
                    }
                }
            }


            // indicate activity
            gameController.recentScreenInteraction = true;

        }

        void PieceRemoved(GameObject pieceToRemove, int pieceType, int oldParentType, int oldParentOutlineIndex)
        {
            if (pieceType == Constants.FRONT)
            {
                childTrain.n_front_pieces--;
                if ((levelNumber == 2) && ((step == 7) || (step == 8)))
                {
                    GameObject sc = GameObject.Find("sel_front");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 10) || (step == 11)))
                {
                    GameObject sc = GameObject.Find("sel_front");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 1) || (step == 2) || (step == 3 ) || (step == 4)))
                {
                    GameObject sc = GameObject.Find("sel_front");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }

            }
            if (pieceType == Constants.BODY)
            {
                if ((levelNumber == 1) && (gameController.tutorial) && ((step == 4) || (step == 5)))
                {
                    GameObject sc = GameObject.Find("sel_body");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 2) && ((step == 1) || (step == 2)))
                {
                    GameObject sc = GameObject.Find("sel_body");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 4) || (step == 5) || (step == 6)))
                {
                    GameObject sc = GameObject.Find("sel_body");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 1) || (step == 2) || (step == 3) || (step == 4)))
                {
                    GameObject sc = GameObject.Find("sel_body");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                childTrain.n_body_pieces--;
            }
            if (pieceType == Constants.CABIN)
            {
                if ((levelNumber == 1) && (gameController.tutorial) && ((step == 4) || (step == 5)))
                {
                    GameObject sc = GameObject.Find("sel_cabin");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 2) && ((step == 1) || (step == 2)))
                {
                    GameObject sc = GameObject.Find("sel_cabin");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                    //step--;
                }
                if ((levelNumber == 3) && ((step == 4) || (step == 5) || (step == 6)))
                {
                    GameObject sc = GameObject.Find("sel_cabin");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                    //step--;
                }
                if ((levelNumber == 4) && ((step == 5) || (step == 6) || (step == 7) || (step == 8)))
                {
                    GameObject sc = GameObject.Find("sel_cabin");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                childTrain.n_cabin_pieces--;
            }
            if (pieceType == Constants.BACK)
            {
                childTrain.n_back_pieces--;
                if ((levelNumber == 2) && ((step == 7)||(step == 8)))
                {
                    GameObject sc = GameObject.Find("sel_back");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 10) || (step == 11)))
                {
                    GameObject sc = GameObject.Find("sel_back");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 9) || (step == 10) || (step == 11)))
                {
                    GameObject sc = GameObject.Find("sel_back");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
            }
            if (pieceType == Constants.WHEEL)
            {
                if ((levelNumber ==1) && (gameController.tutorial) && (step == 3))
                {
                    correct = 1;
                    step++;
                }
                if ((levelNumber == 1) && (gameController.tutorial) && ((step == 6) || (step == 7)) && (oldParentOutlineIndex == 3))
                {
                    GameObject sc = GameObject.Find("sel_wheel1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 1) && (gameController.tutorial) && ((step == 6) || (step == 7)) && (oldParentOutlineIndex == 1))
                {
                    GameObject sc = GameObject.Find("sel_wheel2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 2) && ((step == 3) || (step == 4)) && (oldParentOutlineIndex == 4))
                {
                    GameObject sc = GameObject.Find("sel_wheel1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 2) && ((step == 3) || (step == 4)) && (oldParentOutlineIndex == 2))
                {
                    GameObject sc = GameObject.Find("sel_wheel2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 2) && ((step == 5) || (step == 6)) && (oldParentOutlineIndex == 0))
                {
                    GameObject sc = GameObject.Find("sel_wheel3");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 2) && ((step == 5) || (step == 6)) && (oldParentOutlineIndex == 5))
                {
                    GameObject sc = GameObject.Find("sel_wheel4");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }

                // LEVEL 3
                if ((levelNumber == 3) && ((step == 1) || (step == 2) || (step == 3)) && (oldParentOutlineIndex == 4))
                {
                    GameObject sc = GameObject.Find("sel_wheel1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 1) || (step == 2) || (step == 3)) && (oldParentOutlineIndex == 2))
                {
                    GameObject sc = GameObject.Find("sel_wheel2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 1) || (step == 2) || (step == 3)) && (oldParentOutlineIndex == 0))
                {
                    GameObject sc = GameObject.Find("sel_wheel3");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 4) || (step == 5) || (step == 6)) && (oldParentOutlineIndex == 5))
                {
                    GameObject sc = GameObject.Find("sel_wheel4");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }

                // LEVEL 4
                if ((levelNumber == 4) && ((step == 1) || (step == 2) || (step == 3) || (step == 4)) && (oldParentOutlineIndex == 4))
                {
                    GameObject sc = GameObject.Find("sel_wheel1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 5) || (step == 6) || (step == 7) || (step == 8)) && (oldParentOutlineIndex == 2))
                {
                    GameObject sc = GameObject.Find("sel_wheel2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 9) || (step == 10) || (step == 11)) && (oldParentOutlineIndex == 0))
                {
                    GameObject sc = GameObject.Find("sel_wheel3");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 9) || (step == 10) || (step == 11)) && (oldParentOutlineIndex == 5))
                {
                    GameObject sc = GameObject.Find("sel_wheel4");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }

                childTrain.n_wheel_pieces--;
            }
            if (pieceType == Constants.SMOKE)
            {
                if ((levelNumber == 2) && ((step == 9) || (step == 10)) && (oldParentOutlineIndex == 9))
                {
                    GameObject sc = GameObject.Find("sel_smoke1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 2) && ((step == 9) || (step == 10)) && (oldParentOutlineIndex == 8))
                {
                    GameObject sc = GameObject.Find("sel_smoke2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }

                //LEVEL 3
                if ((levelNumber == 3) && ((step == 7) || (step == 8) || (step == 9)) && (oldParentOutlineIndex == 9))
                {
                    GameObject sc = GameObject.Find("sel_smoke1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 7) || (step == 8) || (step == 9)) && (oldParentOutlineIndex == 8))
                {
                    GameObject sc = GameObject.Find("sel_smoke2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 3) && ((step == 7) || (step == 8) || (step == 9)) && (oldParentOutlineIndex == 7))
                {
                    GameObject sc = GameObject.Find("sel_smoke3");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }

                //LEVEL 4
                if ((levelNumber == 4) && ((step == 1) || (step == 2) || (step == 3) || (step == 4)) && (oldParentOutlineIndex == 9))
                {
                    GameObject sc = GameObject.Find("sel_smoke1");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 5) || (step == 6) || (step == 7) || (step == 8)) && (oldParentOutlineIndex == 8))
                {
                    GameObject sc = GameObject.Find("sel_smoke2");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }
                if ((levelNumber == 4) && ((step == 5) || (step == 6) || (step == 7) || (step == 8)) && (oldParentOutlineIndex == 7))
                {
                    GameObject sc = GameObject.Find("sel_smoke3");
                    sc.gameObject.GetComponent<Image>().enabled = true;
                    step--;
                }

                childTrain.n_smoke_pieces--;
            }


        }

        void RemovePanelPieces()
        {
            Transform frontPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftFrontPieces");
            Transform frontPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightFrontPieces");
            foreach (var comp in frontPieceParentPanelLeft.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }
            foreach (var comp in frontPieceParentPanelRight.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }

            Transform backPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftBackPieces");
            Transform backPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightBackPieces");
            foreach (var comp in backPieceParentPanelLeft.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }
            foreach (var comp in backPieceParentPanelRight.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }

            Transform bodyPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftBodyPieces");
            Transform bodyPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightBodyPieces");
            foreach (var comp in bodyPieceParentPanelLeft.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }
            foreach (var comp in bodyPieceParentPanelRight.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }

            Transform cabinPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftCabinPieces");
            Transform cabinPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightCabinPieces");
            foreach (var comp in cabinPieceParentPanelLeft.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }
            foreach (var comp in cabinPieceParentPanelRight.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }

            Transform wheelPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftWheelPieces");
            Transform wheelPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightWheelPieces");
            foreach (var comp in wheelPieceParentPanelLeft.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }
            foreach (var comp in wheelPieceParentPanelRight.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }

            Transform smokePieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftSmokePieces");
            Transform smokePieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightSmokePieces");
            foreach (var comp in smokePieceParentPanelLeft.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }
            foreach (var comp in smokePieceParentPanelRight.GetComponents<Component>())
            {
                if (!(comp is Transform))
                {
                    Destroy(comp);
                }
            }

        }

        void PlaceOutlineAndHousePieces()
        {
            int levelNumber = gameController.saveLevel;
            //-------------------------------------- Rocket Pieces on the Panels --------------------------------------

            // front pieces
            Transform frontPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftFrontPieces");
            Transform frontPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightFrontPieces");
            int totalFrontPieceOptions = Constants.LEVEL_INFO[levelNumber].frontPieceOptionNames.Count;
            int currentFrontPieceOptions = 0;
            for (int i = 0; i < totalFrontPieceOptions; i++)
            {
                for (int j = 0; j < gameController.frontPieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].frontPieceOptionNames[i] == gameController.frontPieceOptions[j].name)
                    {
                        GameObject panelFrontPieceClone = Instantiate(gameController.frontPieceOptions[j]);
                        if (currentFrontPieceOptions * 2 < totalFrontPieceOptions)
                        {
                            panelFrontPieceClone.transform.SetParent(frontPieceParentPanelLeft);
                        }
                        else
                        {
                            panelFrontPieceClone.transform.SetParent(frontPieceParentPanelRight);
                        }
                        panelFrontPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        frontPanelTrainPieces.Add(panelFrontPieceClone);
                        currentFrontPieceOptions++;
                    }
                }
            }

            // body pieces
            Transform bodyPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftBodyPieces");
            Transform bodyPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightBodyPieces");
            int totalBodyPieceOptions = Constants.LEVEL_INFO[levelNumber].bodyPieceOptionNames.Count;
            int currentBodyPieceOptions = 0;
            for (int i = 0; i < totalBodyPieceOptions; i++)
            {
                for (int j = 0; j < gameController.bodyPieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].bodyPieceOptionNames[i] == gameController.bodyPieceOptions[j].name)
                    {
                        GameObject panelBodyPieceClone = Instantiate(gameController.bodyPieceOptions[j]);
                        if (currentBodyPieceOptions * 2 < totalBodyPieceOptions)
                        {
                            panelBodyPieceClone.transform.SetParent(bodyPieceParentPanelLeft);
                        }
                        else
                        {
                            panelBodyPieceClone.transform.SetParent(bodyPieceParentPanelRight);
                        }
                        panelBodyPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        bodyPanelTrainPieces.Add(panelBodyPieceClone);
                        currentBodyPieceOptions++;
                    }
                }
            }

            // cabin pieces
            Transform cabinPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftCabinPieces"); 
            Transform cabinPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightCabinPieces");
            int totalCabinPieceOptions = Constants.LEVEL_INFO[levelNumber].cabinPieceOptionNames.Count; 
            int currentCabinPieceOptions = 0;
            for (int i = 0; i < totalCabinPieceOptions; i++)
            {
                for (int j = 0; j < gameController.cabinPieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].cabinPieceOptionNames[i] == gameController.cabinPieceOptions[j].name)
                    {
                        GameObject panelCabinPieceClone = Instantiate(gameController.cabinPieceOptions[j]);
                        if (currentCabinPieceOptions * 2 < totalCabinPieceOptions)
                        {
                            panelCabinPieceClone.transform.SetParent(cabinPieceParentPanelLeft);
                        }
                        else
                        {
                            panelCabinPieceClone.transform.SetParent(cabinPieceParentPanelRight);
                        }
                        panelCabinPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        cabinPanelTrainPieces.Add(panelCabinPieceClone);
                        currentCabinPieceOptions++;
                    }
                }
            }

            // back pieces
            Transform backPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftBackPieces"); 
            Transform backPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightBackPieces");
            int totalBackPieceOptions = Constants.LEVEL_INFO[levelNumber].backPieceOptionNames.Count;
            int currentBackPieceOptions = 0;
            for (int i = 0; i < totalBackPieceOptions; i++)
            {
                for (int j = 0; j < gameController.backPieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].backPieceOptionNames[i] == gameController.backPieceOptions[j].name) 
                    {
                        GameObject panelBackPieceClone = Instantiate(gameController.backPieceOptions[j]);
                        if (currentBackPieceOptions * 2 < totalBackPieceOptions)
                        {
                            panelBackPieceClone.transform.SetParent(backPieceParentPanelLeft);
                        }
                        else
                        {
                            panelBackPieceClone.transform.SetParent(backPieceParentPanelRight);
                        }
                        panelBackPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        backPanelTrainPieces.Add(panelBackPieceClone);
                        currentBackPieceOptions++;
                    }
                }
            }

            // wheel pieces
            Transform wheelPieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftWheelPieces"); 
            Transform wheelPieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightWheelPieces");
            int totalWheelPieceOptions = Constants.LEVEL_INFO[levelNumber].wheelPieceOptionNames.Count;
            int currentWheelPieceOptions = 0;
            for (int i = 0; i < totalWheelPieceOptions; i++)
            {
                for (int j = 0; j < gameController.wheelPieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].wheelPieceOptionNames[i] == gameController.wheelPieceOptions[j].name)
                    {
                        GameObject panelWheelPieceClone = Instantiate(gameController.wheelPieceOptions[j]); 
                        if (currentWheelPieceOptions * 2 < totalWheelPieceOptions)
                        {
                            panelWheelPieceClone.transform.SetParent(wheelPieceParentPanelLeft);
                        }
                        else
                        {
                            panelWheelPieceClone.transform.SetParent(wheelPieceParentPanelRight);
                        }
                        panelWheelPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        wheelPanelTrainPieces.Add(panelWheelPieceClone);
                        currentWheelPieceOptions++;
                    }
                }
            }

            // smoke pieces
            Transform smokePieceParentPanelLeft = TrainPiecesLeftPanel.FindChild("LeftSmokePieces");
            Transform smokePieceParentPanelRight = TrainPiecesRightPanel.FindChild("RightSmokePieces");
            int totalSmokePieceOptions = Constants.LEVEL_INFO[levelNumber].smokePieceOptionNames.Count;
            int currentSmokePieceOptions = 0;
            for (int i = 0; i < totalSmokePieceOptions; i++)
            {
                for (int j = 0; j < gameController.smokePieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].smokePieceOptionNames[i] == gameController.smokePieceOptions[j].name)
                    {
                        GameObject panelSmokePieceClone = Instantiate(gameController.smokePieceOptions[j]); 
                        if (currentSmokePieceOptions * 2 < totalSmokePieceOptions)
                        {
                            panelSmokePieceClone.transform.SetParent(smokePieceParentPanelLeft);
                        }
                        else
                        {
                            panelSmokePieceClone.transform.SetParent(smokePieceParentPanelRight);
                        }
                        panelSmokePieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        smokePanelTrainPieces.Add(panelSmokePieceClone);
                        currentSmokePieceOptions++;
                    }
                }
            }


        }



        public void SetUpExplainerScene()
        {

            // initialize pieceTypeSelected
            currentPieceTypeSelected = Constants.NONE_SELECTED;
            lastPieceTypeSelected = Constants.NONE_SELECTED;

            // initialize firstStateChangeOccured
            firstStateChangeOccured = false;

            // find all of the objects we need access to
            DashedOutlinesPanel = GameObject.Find("DashedOutlines").transform;
            //SelectedOutlinesPanel = GameObject.Find ("SelectedOutlines").transform;
            TrainPiecesLeftPanel = GameObject.Find("TrainPiecesLeft").transform;
            TrainPiecesRightPanel = GameObject.Find("TrainPiecesRight").transform;
            panelsAnimator = GameObject.Find("TrainField").GetComponent<Animator>();
            //panelsAnimator = GameObject.Find("Builder").GetComponent<Animator>();

            PlaceOutlineAndHousePieces();

            UpdateOutlineAndHousePanelPieces();

        }

        void ShowPieces(List<GameObject> pieces)
        {
            foreach (GameObject piece in pieces)
            {
                piece.GetComponent<Image>().enabled = true;
            }
        }

        void TriggerPanelChange(int selectedOutlineType)
        {
            if (selectedOutlineType != currentPieceTypeSelected)
            {
                if (firstStateChangeOccured == true)
                {
                    panelsAnimator.SetTrigger("movePanelsInOut");
                }
                else
                {
                    panelsAnimator.SetTrigger("movePanelsIn");
                    firstStateChangeOccured = true;
                }

                // update the current and last piece type selected varaibles
                lastPieceTypeSelected = currentPieceTypeSelected;
                currentPieceTypeSelected = selectedOutlineType;

                // show/hide the appropriate ouline and rocket panel pieces
                UpdateOutlineAndHousePanelPieces();

                // indicate activity
                gameController.recentScreenInteraction = true;
            }
        }

        void UpdateOutlineAndHousePanelPieces()
        {

            // hide the selected outlines and show the dashed outlines of the old selected piece type
            if (lastPieceTypeSelected == Constants.FRONT)
            {
                ShowPieces(dashedFrontOutlineSlots);
            }
            else if (lastPieceTypeSelected == Constants.BODY)
                ShowPieces(dashedBodyOutlineSlot);
            else if (lastPieceTypeSelected == Constants.CABIN)
                ShowPieces(dashedCabinOutlineSlots);
            else if (lastPieceTypeSelected == Constants.BACK)
                ShowPieces(dashedBackOutlineSlots);
            else if (lastPieceTypeSelected == Constants.SMOKE)
                ShowPieces(dashedSmokeOutlineSlots);
            else if (lastPieceTypeSelected == Constants.WHEEL)
                ShowPieces(dashedWheelOutlineSlots);

            // hide the dashed outlines and show the selected outlines of the new selected piece type
            if (currentPieceTypeSelected == Constants.NONE_SELECTED)
            {

                // show all dashed pieces 
                ShowPieces(dashedFrontOutlineSlots);
                ShowPieces(dashedBodyOutlineSlot);
                ShowPieces(dashedCabinOutlineSlots);
                ShowPieces(dashedBackOutlineSlots);
                ShowPieces(dashedSmokeOutlineSlots);
                ShowPieces(dashedWheelOutlineSlots);

                // hide all the pieces
                HidePieces(frontPanelTrainPieces);
                HidePieces(bodyPanelTrainPieces);
                HidePieces(cabinPanelTrainPieces);
                HidePieces(smokePanelTrainPieces);
                HidePieces(backPanelTrainPieces);
                HidePieces(wheelPanelTrainPieces);

            }
            else if (currentPieceTypeSelected == Constants.FRONT)
                HidePieces(dashedFrontOutlineSlots);
            else if (currentPieceTypeSelected == Constants.BODY)
                HidePieces(dashedBodyOutlineSlot);
            else if (currentPieceTypeSelected == Constants.CABIN)
                HidePieces(dashedCabinOutlineSlots);
            else if (currentPieceTypeSelected == Constants.BACK)
                HidePieces(dashedBackOutlineSlots);
            else if (currentPieceTypeSelected == Constants.SMOKE)
                HidePieces(dashedSmokeOutlineSlots);
            else if (currentPieceTypeSelected == Constants.WHEEL)
                HidePieces(dashedWheelOutlineSlots);

        }

        public void DisableDragAndDropGameplay()
        {

            // subscribe to the events that indicate clicks on outline pieces
            Slot.OnClickForPanelChangeOutlinePiece -= TriggerPanelChange;

            // subscribe to the event that alerts the game manager of pieces added to the rocket
            Slot.OnPieceAddedToTrain -= PieceAddedToTrain;


            // subscribe to the event that alerts the game manager of the panel going in
            PanelAnimationEventHandler.OnTriggerPanelIn -= PanelIn;
            DragHandler.OnClickForPanelChangeTrainPiece -= TriggerPanelChange;

            // subscribe to the event that alerts the game manager of cloned pieces added to the panel
            DragHandler.OnPieceClonedToPanel -= PieceAddedToPanel;

            // subscribe to the event that alerts the game manager of a deleted piece (via trash)
            DragHandler.OnPieceRemovedByTrash -= PieceRemoved;

        }

        public void EnableDragAndDropGameplay()
        {

            // subscribe to the events that indicate clicks on outline pieces
            Slot.OnClickForPanelChangeOutlinePiece += TriggerPanelChange;

            // subscribe to the event that alerts the game manager of pieces added to the rocket
            Slot.OnPieceAddedToTrain += PieceAddedToTrain;

            // subscribe to the event that alerts the game manager of the panel going in
            PanelAnimationEventHandler.OnTriggerPanelIn += PanelIn;
            DragHandler.OnClickForPanelChangeTrainPiece += TriggerPanelChange;

            // subscribe to the event that alerts the game manager of cloned pieces added to the panel
            DragHandler.OnPieceClonedToPanel += PieceAddedToPanel;

            // subscribe to the event that alerts the game manager of a deleted piece (via trash)
            DragHandler.OnPieceRemovedByTrash += PieceRemoved;

        }


        //--------------------------------------------------------------------------------------------------------------

        public void EraseAllBuilder()
        {
                GameObject a = GameObject.Find("s_front");
                foreach (Transform child in a.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject b = GameObject.Find("s_body");
                foreach (Transform child in b.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject c = GameObject.Find("s_cabin");
                foreach (Transform child in c.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject d = GameObject.Find("s_back");
                foreach (Transform child in d.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject e = GameObject.Find("s_smoke1");
                foreach (Transform child in e.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject f = GameObject.Find("s_smoke2");
                foreach (Transform child in f.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject g = GameObject.Find("s_smoke3");
                foreach (Transform child in g.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject h = GameObject.Find("s_wheel1");
                foreach (Transform child in h.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject i = GameObject.Find("s_wheel2");
                foreach (Transform child in i.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                GameObject j = GameObject.Find("s_wheel3");
                foreach (Transform child in j.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }


                GameObject k = GameObject.Find("s_wheel4");
                foreach (Transform child in k.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

            GameObject ass = GameObject.Find("o_front");
            foreach (Transform child in ass.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject bs = GameObject.Find("o_body");
            foreach (Transform child in bs.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject cs = GameObject.Find("o_cabin");
            foreach (Transform child in cs.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject ds = GameObject.Find("o_back");
            foreach (Transform child in ds.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject es = GameObject.Find("o_smoke1");
            foreach (Transform child in es.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject fs = GameObject.Find("o_smoke2");
            foreach (Transform child in fs.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject gs = GameObject.Find("o_smoke3");
            foreach (Transform child in gs.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject hs = GameObject.Find("o_wheel1");
            foreach (Transform child in hs.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject iss = GameObject.Find("o_wheel2");
            foreach (Transform child in iss.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject js = GameObject.Find("o_wheel3");
            foreach (Transform child in js.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject ks = GameObject.Find("o_wheel4");
            foreach (Transform child in ks.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject l = GameObject.Find("front");
            l.gameObject.GetComponent<Image>().enabled = false;
            GameObject m = GameObject.Find("body");
            m.gameObject.GetComponent<Image>().enabled = false;
            GameObject n = GameObject.Find("cabin");
            n.gameObject.GetComponent<Image>().enabled = false;
            GameObject o = GameObject.Find("back");
            o.gameObject.GetComponent<Image>().enabled = false;
            GameObject p = GameObject.Find("smoke1");
            p.gameObject.GetComponent<Image>().enabled = false;
            GameObject q = GameObject.Find("smoke2");
            q.gameObject.GetComponent<Image>().enabled = false;
            GameObject r = GameObject.Find("smoke3");
            r.gameObject.GetComponent<Image>().enabled = false;
            GameObject s = GameObject.Find("wheel1");
            s.gameObject.GetComponent<Image>().enabled = false;
            GameObject t = GameObject.Find("wheel2");
            t.gameObject.GetComponent<Image>().enabled = false;
            GameObject u = GameObject.Find("wheel3");
            u.gameObject.GetComponent<Image>().enabled = false;
            GameObject v = GameObject.Find("wheel4");
            v.gameObject.GetComponent<Image>().enabled = false;


            GameObject w = GameObject.Find("o_body");
            w.gameObject.GetComponent<Image>().enabled = false;
            GameObject y = GameObject.Find("o_front");
            y.gameObject.GetComponent<Image>().enabled = false;
            GameObject x = GameObject.Find("o_cabin");
            x.gameObject.GetComponent<Image>().enabled = false;
            GameObject z = GameObject.Find("o_back");
            z.gameObject.GetComponent<Image>().enabled = false;
            GameObject aa = GameObject.Find("o_smoke1");
            aa.gameObject.GetComponent<Image>().enabled = false;
            GameObject bb = GameObject.Find("o_smoke2");
            bb.gameObject.GetComponent<Image>().enabled = false;
            GameObject cc = GameObject.Find("o_smoke3");
            cc.gameObject.GetComponent<Image>().enabled = false;
            GameObject dd = GameObject.Find("o_wheel1");
            dd.gameObject.GetComponent<Image>().enabled = false;
            GameObject ee = GameObject.Find("o_wheel2");
            ee.gameObject.GetComponent<Image>().enabled = false;
            GameObject ff = GameObject.Find("o_wheel3");
            ff.gameObject.GetComponent<Image>().enabled = false;
            GameObject gg = GameObject.Find("o_wheel4");
            gg.gameObject.GetComponent<Image>().enabled = false;

            GameObject hh = GameObject.Find("s_front");
            hh.gameObject.GetComponent<Image>().enabled = false;
            GameObject ii = GameObject.Find("s_body");
            ii.gameObject.GetComponent<Image>().enabled = false;
            GameObject jj = GameObject.Find("s_cabin");
            jj.gameObject.GetComponent<Image>().enabled = false;
            GameObject kk = GameObject.Find("s_back");
            kk.gameObject.GetComponent<Image>().enabled = false;
            GameObject ll = GameObject.Find("s_smoke1");
            ll.gameObject.GetComponent<Image>().enabled = false;
            GameObject mm = GameObject.Find("s_smoke2");
            mm.gameObject.GetComponent<Image>().enabled = false;
            GameObject nn = GameObject.Find("s_smoke3");
            nn.gameObject.GetComponent<Image>().enabled = false;
            GameObject oo = GameObject.Find("s_wheel1");
            oo.gameObject.GetComponent<Image>().enabled = false;
            GameObject pp = GameObject.Find("s_wheel2");
            pp.gameObject.GetComponent<Image>().enabled = false;
            GameObject qq = GameObject.Find("s_wheel3");
            qq.gameObject.GetComponent<Image>().enabled = false;
            GameObject rr = GameObject.Find("s_wheel4");
            rr.gameObject.GetComponent<Image>().enabled = false;

            GameObject  ss = GameObject.Find("LeftPiecePanel");
            ss.gameObject.GetComponent<Image>().enabled = false;
            GameObject tt = GameObject.Find("RightPiecePanel");
            tt.gameObject.GetComponent<Image>().enabled = false;

            GameObject uu = GameObject.Find("trashcanLeft");
            uu.gameObject.GetComponent<Image>().enabled = false;
            GameObject vv = GameObject.Find("trashcanRight");
            vv.gameObject.GetComponent<Image>().enabled = false;

            HidePieces(frontPanelTrainPieces);
            HidePieces(bodyPanelTrainPieces);
            HidePieces(cabinPanelTrainPieces);
            HidePieces(smokePanelTrainPieces);
            HidePieces(backPanelTrainPieces);
            HidePieces(wheelPanelTrainPieces);


            explainerCompleteButton.SetActive(false);
            


        }

        public void ShowBothTrains()
        {
            GameObject l = GameObject.Find("c_front");
            l.gameObject.GetComponent<Image>().enabled = true;
            GameObject m = GameObject.Find("c_body");
            m.gameObject.GetComponent<Image>().enabled = true;
            GameObject n = GameObject.Find("c_cabin");
            n.gameObject.GetComponent<Image>().enabled = true;
            GameObject o = GameObject.Find("c_back");
            o.gameObject.GetComponent<Image>().enabled = true;
            GameObject p = GameObject.Find("c_smoke1");
            p.gameObject.GetComponent<Image>().enabled = true;
            GameObject q = GameObject.Find("c_smoke2");
            q.gameObject.GetComponent<Image>().enabled = true;
            GameObject r = GameObject.Find("c_smoke3");
            r.gameObject.GetComponent<Image>().enabled = true;
            GameObject s = GameObject.Find("c_wheel1");
            s.gameObject.GetComponent<Image>().enabled = true;
            GameObject t = GameObject.Find("c_wheel2");
            t.gameObject.GetComponent<Image>().enabled = true;
            GameObject u = GameObject.Find("c_wheel3");
            u.gameObject.GetComponent<Image>().enabled = true;
            GameObject v = GameObject.Find("c_wheel4");
            v.gameObject.GetComponent<Image>().enabled = true;


            GameObject w = GameObject.Find("r_body");
            w.gameObject.GetComponent<Image>().enabled = true;
            GameObject y = GameObject.Find("r_front");
            y.gameObject.GetComponent<Image>().enabled = true;
            GameObject x = GameObject.Find("r_cabin");
            x.gameObject.GetComponent<Image>().enabled = true;
            GameObject z = GameObject.Find("r_back");
            z.gameObject.GetComponent<Image>().enabled = true;
            GameObject aa = GameObject.Find("r_smoke1");
            aa.gameObject.GetComponent<Image>().enabled = true;
            GameObject bb = GameObject.Find("r_smoke2");
            aa.gameObject.GetComponent<Image>().enabled = true;
            GameObject cc = GameObject.Find("r_smoke3");
            bb.gameObject.GetComponent<Image>().enabled = true;
            GameObject dd = GameObject.Find("r_wheel1");
            cc.gameObject.GetComponent<Image>().enabled = true;
            GameObject ee = GameObject.Find("r_wheel2");
            dd.gameObject.GetComponent<Image>().enabled = true;
            GameObject ff = GameObject.Find("r_wheel3");
            ee.gameObject.GetComponent<Image>().enabled = true;
            GameObject gg = GameObject.Find("r_wheel4");
            ff.gameObject.GetComponent<Image>().enabled = true;

        }

        public void NewTrain()
        {
            childTrain.front = -1;
            childTrain.body = - 1;
            childTrain.cabin = -1;
            childTrain.back = -1;
            childTrain.wheel1 = -1;
            childTrain.wheel2 = -1;
            childTrain.wheel3 = -1;
            childTrain.wheel4 = -1;
            childTrain.smoke1 = -1;
            childTrain.smoke2 = -1;
            childTrain.smoke3 = -1;

            childTrain.n_front_pieces = 0;
            childTrain.n_body_pieces = 0;
            childTrain.n_cabin_pieces = 0;
            childTrain.n_back_pieces = 0;
            childTrain.n_smoke_pieces = 0;
            childTrain.n_wheel_pieces = 0;

            robotTrain.front = -1;
            robotTrain.body = -1;
            robotTrain.cabin = -1;
            robotTrain.back = -1;
            robotTrain.wheel1 = -1;
            robotTrain.wheel2 = -1;
            robotTrain.wheel3 = -1;
            robotTrain.wheel4 = -1;
            robotTrain.smoke1 = -1;
            robotTrain.smoke2 = -1;
            robotTrain.smoke3 = -1;

            robotTrain.n_front_pieces = 0;
            robotTrain.n_body_pieces = 0;
            robotTrain.n_cabin_pieces = 0;
            robotTrain.n_back_pieces = 0;
            robotTrain.n_smoke_pieces = 0;
            robotTrain.n_wheel_pieces = 0;
        }

        private void CorrectTrain()
        {
            int train_errors = 0;

            if (childTrain.front != robotTrain.front)
            {
                train_errors++;
            }
            if (childTrain.body != robotTrain.body)
            {
                train_errors++;
            }
            if (childTrain.cabin != robotTrain.cabin)
            {
                train_errors++;
            }
            if (childTrain.back != robotTrain.back)
            {
                train_errors++;
            }
            if (childTrain.wheel1 != robotTrain.wheel1)
            {
                train_errors++;
            }
            if (childTrain.wheel2 != robotTrain.wheel2)
            {
                train_errors++;
            }
            if (childTrain.wheel3 != robotTrain.wheel3)
            {
                train_errors++;
            }
            if (childTrain.wheel4 != robotTrain.wheel4)
            {
                train_errors++;
            }
            if (childTrain.smoke1 != robotTrain.smoke1)
            {
                train_errors++;
            }
            if (childTrain.smoke2 != robotTrain.smoke2)
            {
                train_errors++;
            }
            if (childTrain.smoke3 != robotTrain.smoke3)
            {
                train_errors++;
            }

            if (train_errors == 0)
            {
                gameController.SendRobotUtterance("both-trains-right", false, -1, -1, -1, -1);
            }
            else
            {
                gameController.SendRobotUtterance("both-trains-wrong", false, -1, -1, -1, -1);
            }

            int order_errors = 11 - score;


            //gameController.SendRobotUtterance("child-builder-inverted-pieces", true, 2, -1, -1);NewGameButton

            performanceMetrics(train_errors, order_errors);
            if (gameController.internalGameState == Constants.END_GAME)
            {
                // send goobye message
                gameController.SendRobotUtterance("end", false, -1, -1, -1, -1);
                Application.Quit();
            }
            gameController.SendRobotUtterance("another-game", false, -1, -1, -1, -1);
        }

        /*private void performanceMetrics(int train_errors, int order_errors)
        {
            Dictionary<string, float> averagePerformanceMetrics = new Dictionary<string, float>();
            averagePerformanceMetrics.Add("correct-pieces-errors", train_errors);
            averagePerformanceMetrics.Add("correct-order-errors", order_errors);
            Logger.Log(train_errors);
            Logger.Log(order_errors);
            Logger.Log(averagePerformanceMetrics);
            // send to ROS the percentage of correct pieces 
            if (gameController.sendReceiveROSMessages)
            {
                gameController.clientSocket.SendMessage(RosbridgeUtilities.GetROSJsonPublishGameStateMsg(
                    Constants.STATE_ROSTOPIC, Constants.GAME_STATE_END, averagePerformanceMetrics));
            }

        }*/

        private void performanceMetrics(int train_errors, int order_errors)
        {

            if (gameController.performanceMetricsTrainErrors == 0 && gameController.performanceMetricsOrderErrors == 0)
            {
                gameController.performanceMetricsOrderErrors = order_errors;
                gameController.performanceMetricsTrainErrors = train_errors;
            }
            else
            {
                gameController.performanceMetricsTrainErrors = (gameController.performanceMetricsTrainErrors + train_errors) / 2;
                gameController.performanceMetricsOrderErrors = (gameController.performanceMetricsOrderErrors + train_errors) / 2;
            }

        }

        private void showOnScreen()
        {
            // child built train ----------------------------------------------------------------------------
            var cf = GameObject.Find("c_front");
            var cbd = GameObject.Find("c_body");
            var cc = GameObject.Find("c_cabin");
            var cb = GameObject.Find("c_back");
            var cw1 = GameObject.Find("c_wheel1");
            var cw2 = GameObject.Find("c_wheel2");
            var cw3 = GameObject.Find("c_wheel3");
            var cw4 = GameObject.Find("c_wheel4");
            var cs1 = GameObject.Find("c_smoke1");
            var cs2 = GameObject.Find("c_smoke2");
            var cs3 = GameObject.Find("c_smoke3");


            GameObject c_front = Instantiate(gameController.frontPieceOptions[childTrain.front]);
            GameObject c_body = Instantiate(gameController.bodyPieceOptions[childTrain.body]);
            GameObject c_cabin = Instantiate(gameController.cabinPieceOptions[childTrain.cabin]);
            GameObject c_back = Instantiate(gameController.backPieceOptions[childTrain.back]);
            GameObject c_wheel1 = Instantiate(gameController.wheelPieceOptions[childTrain.wheel1]);
            GameObject c_wheel2 = Instantiate(gameController.wheelPieceOptions[childTrain.wheel2]);
            GameObject c_wheel3 = Instantiate(gameController.wheelPieceOptions[childTrain.wheel3]);
            GameObject c_wheel4 = Instantiate(gameController.wheelPieceOptions[childTrain.wheel4]);
            GameObject c_smoke1 = Instantiate(gameController.smokePieceOptions[childTrain.smoke1]);
            GameObject c_smoke2 = Instantiate(gameController.smokePieceOptions[childTrain.smoke2]);
            GameObject c_smoke3 = Instantiate(gameController.smokePieceOptions[childTrain.smoke3]);

            c_front.transform.SetParent(cf.transform, false);
            c_body.transform.SetParent(cbd.transform, false);
            c_cabin.transform.SetParent(cc.transform, false);
            c_back.transform.SetParent(cb.transform, false);
            c_wheel1.transform.SetParent(cw1.transform, false);
            c_wheel2.transform.SetParent(cw2.transform, false);
            c_wheel3.transform.SetParent(cw3.transform, false);
            c_wheel4.transform.SetParent(cw4.transform, false);
            c_smoke1.transform.SetParent(cs1.transform, false);
            c_smoke2.transform.SetParent(cs2.transform, false);
            c_smoke3.transform.SetParent(cs3.transform, false);

            // robot train ----------------------------------------------------------------------------
            var rf = GameObject.Find("r_front");
            var rbd = GameObject.Find("r_body");
            var rc = GameObject.Find("r_cabin");
            var rb = GameObject.Find("r_back");
            var rw1 = GameObject.Find("r_wheel1");
            var rw2 = GameObject.Find("r_wheel2");
            var rw3 = GameObject.Find("r_wheel3");
            var rw4 = GameObject.Find("r_wheel4");
            var rs1 = GameObject.Find("r_smoke1");
            var rs2 = GameObject.Find("r_smoke2");
            var rs3 = GameObject.Find("r_smoke3");


            GameObject r_front = Instantiate(gameController.frontPieceOptions[robotTrain.front]);
            GameObject r_body = Instantiate(gameController.bodyPieceOptions[robotTrain.body]);
            GameObject r_cabin = Instantiate(gameController.cabinPieceOptions[robotTrain.cabin]);
            GameObject r_back = Instantiate(gameController.backPieceOptions[robotTrain.back]);
            GameObject r_wheel1 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel1]);
            GameObject r_wheel2 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel2]);
            GameObject r_wheel3 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel3]);
            GameObject r_wheel4 = Instantiate(gameController.wheelPieceOptions[robotTrain.wheel4]);
            GameObject r_smoke1 = Instantiate(gameController.smokePieceOptions[robotTrain.smoke1]);
            GameObject r_smoke2 = Instantiate(gameController.smokePieceOptions[robotTrain.smoke2]);
            GameObject r_smoke3 = Instantiate(gameController.smokePieceOptions[robotTrain.smoke3]);

            r_front.transform.SetParent(rf.transform, false);
            r_body.transform.SetParent(rbd.transform, false);
            r_cabin.transform.SetParent(rc.transform, false);
            r_back.transform.SetParent(rb.transform, false);
            r_wheel1.transform.SetParent(rw1.transform, false);
            r_wheel2.transform.SetParent(rw2.transform, false);
            r_wheel3.transform.SetParent(rw3.transform, false);
            r_wheel4.transform.SetParent(rw4.transform, false);
            r_smoke1.transform.SetParent(rs1.transform, false);
            r_smoke2.transform.SetParent(rs2.transform, false);
            r_smoke3.transform.SetParent(rs3.transform, false);



            //checkIfEqual(gw, bw, gr, br, gp1, gp2, bp1, bp2, gd1, gd2, bd1, bd2);
        }

        public void ShowBuilderPieces()
        {
            GameObject l = GameObject.Find("front");
            l.gameObject.GetComponent<Image>().enabled = true;
            GameObject m = GameObject.Find("body");
            m.gameObject.GetComponent<Image>().enabled = true;
            GameObject n = GameObject.Find("cabin");
            n.gameObject.GetComponent<Image>().enabled = true;
            GameObject o = GameObject.Find("back");
            o.gameObject.GetComponent<Image>().enabled = true;
            GameObject p = GameObject.Find("smoke1");
            p.gameObject.GetComponent<Image>().enabled = true;
            GameObject q = GameObject.Find("smoke2");
            q.gameObject.GetComponent<Image>().enabled = true;
            GameObject r = GameObject.Find("smoke3");
            r.gameObject.GetComponent<Image>().enabled = true;
            GameObject s = GameObject.Find("wheel1");
            s.gameObject.GetComponent<Image>().enabled = true;
            GameObject t = GameObject.Find("wheel2");
            t.gameObject.GetComponent<Image>().enabled = true;
            GameObject u = GameObject.Find("wheel3");
            u.gameObject.GetComponent<Image>().enabled = true;
            GameObject v = GameObject.Find("wheel4");
            v.gameObject.GetComponent<Image>().enabled = true;


            GameObject w = GameObject.Find("o_body");
            w.gameObject.GetComponent<Image>().enabled = true;
            GameObject y = GameObject.Find("o_front");
            y.gameObject.GetComponent<Image>().enabled = true;
            GameObject x = GameObject.Find("o_cabin");
            x.gameObject.GetComponent<Image>().enabled = true;
            GameObject z = GameObject.Find("o_back");
            z.gameObject.GetComponent<Image>().enabled = true;
            GameObject aa = GameObject.Find("o_smoke1");
            aa.gameObject.GetComponent<Image>().enabled = true;
            GameObject bb = GameObject.Find("o_smoke2");
            bb.gameObject.GetComponent<Image>().enabled = true;
            GameObject cc = GameObject.Find("o_smoke3");
            cc.gameObject.GetComponent<Image>().enabled = true;
            GameObject dd = GameObject.Find("o_wheel1");
            dd.gameObject.GetComponent<Image>().enabled = true;
            GameObject ee = GameObject.Find("o_wheel2");
            ee.gameObject.GetComponent<Image>().enabled = true;
            GameObject ff = GameObject.Find("o_wheel3");
            ff.gameObject.GetComponent<Image>().enabled = true;
            GameObject gg = GameObject.Find("o_wheel4");
            gg.gameObject.GetComponent<Image>().enabled = true;

            GameObject ss = GameObject.Find("LeftPiecePanel");
            ss.gameObject.GetComponent<Image>().enabled = true;
            GameObject tt = GameObject.Find("RightPiecePanel");
            tt.gameObject.GetComponent<Image>().enabled = true;

            GameObject uu = GameObject.Find("trashcanLeft");
            uu.gameObject.GetComponent<Image>().enabled = true;
            GameObject vv = GameObject.Find("trashcanRight");
            vv.gameObject.GetComponent<Image>().enabled = true;
        }


        public void HideBuilderPieces()
        {
            GameObject l = GameObject.Find("front");
            l.gameObject.GetComponent<Image>().enabled = false;
            GameObject m = GameObject.Find("body");
            m.gameObject.GetComponent<Image>().enabled = false;
            GameObject n = GameObject.Find("cabin");
            n.gameObject.GetComponent<Image>().enabled = false;
            GameObject o = GameObject.Find("back");
            o.gameObject.GetComponent<Image>().enabled = false;
            GameObject p = GameObject.Find("smoke1");
            p.gameObject.GetComponent<Image>().enabled = false;
            GameObject q = GameObject.Find("smoke2");
            q.gameObject.GetComponent<Image>().enabled = false;
            GameObject r = GameObject.Find("smoke3");
            r.gameObject.GetComponent<Image>().enabled = false;
            GameObject s = GameObject.Find("wheel1");
            s.gameObject.GetComponent<Image>().enabled = false;
            GameObject t = GameObject.Find("wheel2");
            t.gameObject.GetComponent<Image>().enabled = false;
            GameObject u = GameObject.Find("wheel3");
            u.gameObject.GetComponent<Image>().enabled = false;
            GameObject v = GameObject.Find("wheel4");
            v.gameObject.GetComponent<Image>().enabled = false;


            GameObject w = GameObject.Find("o_body");
            w.gameObject.GetComponent<Image>().enabled = false;
            GameObject y = GameObject.Find("o_front");
            y.gameObject.GetComponent<Image>().enabled = false;
            GameObject x = GameObject.Find("o_cabin");
            x.gameObject.GetComponent<Image>().enabled = false;
            GameObject z = GameObject.Find("o_back");
            z.gameObject.GetComponent<Image>().enabled = false;
            GameObject aa = GameObject.Find("o_smoke1");
            aa.gameObject.GetComponent<Image>().enabled = false;
            GameObject bb = GameObject.Find("o_smoke2");
            bb.gameObject.GetComponent<Image>().enabled = false;
            GameObject cc = GameObject.Find("o_smoke3");
            cc.gameObject.GetComponent<Image>().enabled = false;
            GameObject dd = GameObject.Find("o_wheel1");
            dd.gameObject.GetComponent<Image>().enabled = false;
            GameObject ee = GameObject.Find("o_wheel2");
            ee.gameObject.GetComponent<Image>().enabled = false;
            GameObject ff = GameObject.Find("o_wheel3");
            ff.gameObject.GetComponent<Image>().enabled = false;
            GameObject gg = GameObject.Find("o_wheel4");
            gg.gameObject.GetComponent<Image>().enabled = false;

        }

        public void ShowTutorialPieces()
        {
            GameObject a = GameObject.Find("t_body");
            a.gameObject.GetComponent<Image>().enabled = true;
            GameObject b = GameObject.Find("t_cabin");
            b.gameObject.GetComponent<Image>().enabled = true;
            GameObject c = GameObject.Find("t_smoke1");
            c.gameObject.GetComponent<Image>().enabled = true;
            GameObject d = GameObject.Find("t_wheel1");
            d.gameObject.GetComponent<Image>().enabled = true;
            GameObject e = GameObject.Find("t_wheel2");
            e.gameObject.GetComponent<Image>().enabled = true;

            /*GameObject f = GameObject.Find("t_s_body");
            f.gameObject.GetComponent<Image>().enabled = true;
            GameObject g = GameObject.Find("t_s_cabin");
            g.gameObject.GetComponent<Image>().enabled = true;
            GameObject h = GameObject.Find("t_s_smoke1");
            h.gameObject.GetComponent<Image>().enabled = true;
            GameObject i = GameObject.Find("t_s_wheel1");
            i.gameObject.GetComponent<Image>().enabled = true;
            GameObject j = GameObject.Find("t_s_wheel2");
            j.gameObject.GetComponent<Image>().enabled = true;*/

            GameObject k = GameObject.Find("t_o_body");
            k.gameObject.GetComponent<Image>().enabled = true;
            GameObject l = GameObject.Find("t_o_cabin");
            l.gameObject.GetComponent<Image>().enabled = true;
            GameObject m = GameObject.Find("t_o_smoke1");
            m.gameObject.GetComponent<Image>().enabled = true;
            GameObject n = GameObject.Find("t_o_wheel1");
            n.gameObject.GetComponent<Image>().enabled = true;
            GameObject o = GameObject.Find("t_o_wheel2");
            o.gameObject.GetComponent<Image>().enabled = true;

        }

        public void HideTutorialPieces()
        {
            GameObject p = GameObject.Find("t_s_body");
            foreach (Transform child in p.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject q = GameObject.Find("t_s_cabin");
            foreach (Transform child in q.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject r = GameObject.Find("t_s_smoke1");
            foreach (Transform child in r.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject s = GameObject.Find("t_s_wheel1");
            foreach (Transform child in s.transform)
            {
                GameObject.Destroy(child.gameObject);
            }
            GameObject t = GameObject.Find("t_s_wheel2");
            foreach (Transform child in t.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject u = GameObject.Find("t_o_body");
            foreach (Transform child in u.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject v = GameObject.Find("t_o_cabin");
            foreach (Transform child in v.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject x = GameObject.Find("t_o_smoke1");
            foreach (Transform child in x.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject w = GameObject.Find("t_o_wheel1");
            foreach (Transform child in w.transform)
            {
                GameObject.Destroy(child.gameObject);
            }
            GameObject y = GameObject.Find("t_o_wheel2");
            foreach (Transform child in y.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject a = GameObject.Find("t_body");
            a.gameObject.GetComponent<Image>().enabled = false;
            GameObject b = GameObject.Find("t_cabin");
            b.gameObject.GetComponent<Image>().enabled = false;
            GameObject c = GameObject.Find("t_smoke1");
            c.gameObject.GetComponent<Image>().enabled = false;
            GameObject d = GameObject.Find("t_wheel1");
            d.gameObject.GetComponent<Image>().enabled = false;
            GameObject e = GameObject.Find("t_wheel2");
            e.gameObject.GetComponent<Image>().enabled = false;

            GameObject f = GameObject.Find("t_s_body");
            f.gameObject.GetComponent<Image>().enabled = false;
            GameObject g = GameObject.Find("t_s_cabin");
            g.gameObject.GetComponent<Image>().enabled = false;
            GameObject h = GameObject.Find("t_s_smoke1");
            h.gameObject.GetComponent<Image>().enabled = false;
            GameObject i = GameObject.Find("t_s_wheel1");
            i.gameObject.GetComponent<Image>().enabled = false;
            GameObject j = GameObject.Find("t_s_wheel2");
            j.gameObject.GetComponent<Image>().enabled = false;

            GameObject k = GameObject.Find("t_o_body");
            k.gameObject.GetComponent<Image>().enabled = false;
            GameObject l = GameObject.Find("t_o_cabin");
            l.gameObject.GetComponent<Image>().enabled = false;
            GameObject m = GameObject.Find("t_o_smoke1");
            m.gameObject.GetComponent<Image>().enabled = false;
            GameObject n = GameObject.Find("t_o_wheel1");
            n.gameObject.GetComponent<Image>().enabled = false;
            GameObject o = GameObject.Find("t_o_wheel2");
            o.gameObject.GetComponent<Image>().enabled = false;

            GameObject sc1 = GameObject.Find("arrow1");
            sc1.gameObject.GetComponent<Renderer>().enabled = false;
            GameObject sc2 = GameObject.Find("arrow2");
            sc2.gameObject.GetComponent<Renderer>().enabled = false;
            GameObject sc3 = GameObject.Find("arrow_drag");
            sc3.gameObject.GetComponent<Renderer>().enabled = false;
            GameObject sc4 = GameObject.Find("arrow_drag2");
            sc4.gameObject.GetComponent<Renderer>().enabled = false;
            GameObject sc5 = GameObject.Find("t_sel_body");
            sc5.gameObject.GetComponent<Image>().enabled = false;
            GameObject sc6 = GameObject.Find("t_sel_cabin");
            sc6.gameObject.GetComponent<Image>().enabled = false;
            GameObject sc7 = GameObject.Find("arrow_trash");
            sc7.gameObject.GetComponent<Renderer>().enabled = false;
            GameObject sc8 = GameObject.Find("t_sel_smoke1");
            sc8.gameObject.GetComponent<Image>().enabled = false;
            GameObject sc9 = GameObject.Find("t_sel_wheel1");
            sc9.gameObject.GetComponent<Image>().enabled = false;
            GameObject sc10 = GameObject.Find("t_sel_wheel2");
            sc10.gameObject.GetComponent<Image>().enabled = false;

            HidePieces(frontPanelTrainPieces);
            HidePieces(bodyPanelTrainPieces);
            HidePieces(cabinPanelTrainPieces);
            HidePieces(smokePanelTrainPieces);
            HidePieces(backPanelTrainPieces);
            HidePieces(wheelPanelTrainPieces);
        }


        public void HideBothTrainsPieces()
        {

            GameObject a = GameObject.Find("c_front");
            foreach (Transform child in a.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject b = GameObject.Find("c_body");
            foreach (Transform child in b.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject c = GameObject.Find("c_cabin");
            foreach (Transform child in c.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject d = GameObject.Find("c_back");
            foreach (Transform child in d.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject e = GameObject.Find("c_smoke1");
            foreach (Transform child in e.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject f = GameObject.Find("c_smoke2");
            foreach (Transform child in f.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject g = GameObject.Find("c_smoke3");
            foreach (Transform child in g.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject h = GameObject.Find("c_wheel1");
            foreach (Transform child in h.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject i = GameObject.Find("c_wheel2");
            foreach (Transform child in i.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject j = GameObject.Find("c_wheel3");
            foreach (Transform child in j.transform)
            {
                GameObject.Destroy(child.gameObject);
            }


            GameObject k = GameObject.Find("c_wheel4");
            foreach (Transform child in k.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject aa = GameObject.Find("r_front");
            foreach (Transform child in aa.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject bb = GameObject.Find("r_body");
            foreach (Transform child in bb.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject cc = GameObject.Find("r_cabin");
            foreach (Transform child in cc.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject dd = GameObject.Find("r_back");
            foreach (Transform child in dd.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject ee = GameObject.Find("r_smoke1");
            foreach (Transform child in ee.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject ff = GameObject.Find("r_smoke2");
            foreach (Transform child in ff.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject gg = GameObject.Find("r_smoke3");
            foreach (Transform child in gg.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject hh = GameObject.Find("r_wheel1");
            foreach (Transform child in hh.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject ii = GameObject.Find("r_wheel2");
            foreach (Transform child in ii.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject jj = GameObject.Find("r_wheel3");
            foreach (Transform child in jj.transform)
            {
                GameObject.Destroy(child.gameObject);
            }


            GameObject kk = GameObject.Find("r_wheel4");
            foreach (Transform child in kk.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject l = GameObject.Find("c_front");
            l.gameObject.GetComponent<Image>().enabled = false;
            GameObject m = GameObject.Find("c_body");
            m.gameObject.GetComponent<Image>().enabled = false;
            GameObject n = GameObject.Find("c_cabin");
            n.gameObject.GetComponent<Image>().enabled = false;
            GameObject o = GameObject.Find("c_back");
            o.gameObject.GetComponent<Image>().enabled = false;
            GameObject p = GameObject.Find("c_smoke1");
            p.gameObject.GetComponent<Image>().enabled = false;
            GameObject q = GameObject.Find("c_smoke2");
            q.gameObject.GetComponent<Image>().enabled = false;
            GameObject r = GameObject.Find("c_smoke3");
            r.gameObject.GetComponent<Image>().enabled = false;
            GameObject s = GameObject.Find("c_wheel1");
            s.gameObject.GetComponent<Image>().enabled = false;
            GameObject t = GameObject.Find("c_wheel2");
            t.gameObject.GetComponent<Image>().enabled = false;
            GameObject u = GameObject.Find("c_wheel3");
            u.gameObject.GetComponent<Image>().enabled = false;
            GameObject v = GameObject.Find("c_wheel4");
            v.gameObject.GetComponent<Image>().enabled = false;


            GameObject w = GameObject.Find("r_body");
            w.gameObject.GetComponent<Image>().enabled = false;
            GameObject y = GameObject.Find("r_front");
            y.gameObject.GetComponent<Image>().enabled = false;
            GameObject x = GameObject.Find("r_cabin");
            x.gameObject.GetComponent<Image>().enabled = false;
            GameObject z = GameObject.Find("r_back");
            z.gameObject.GetComponent<Image>().enabled = false;
            GameObject aaa = GameObject.Find("r_smoke1");
            aaa.gameObject.GetComponent<Image>().enabled = false;
            GameObject bbb = GameObject.Find("r_smoke2");
            bbb.gameObject.GetComponent<Image>().enabled = false;
            GameObject ccc = GameObject.Find("r_smoke3");
            ccc.gameObject.GetComponent<Image>().enabled = false;
            GameObject ddd = GameObject.Find("r_wheel1");
            ddd.gameObject.GetComponent<Image>().enabled = false;
            GameObject eee = GameObject.Find("r_wheel2");
            eee.gameObject.GetComponent<Image>().enabled = false;
            GameObject fff = GameObject.Find("r_wheel3");
            fff.gameObject.GetComponent<Image>().enabled = false;
            GameObject ggg = GameObject.Find("r_wheel4");
            ggg.gameObject.GetComponent<Image>().enabled = false;
        }

        public void BothTrainsNewGame()
        {
            NewGameButton.SetActive(false);
            explainerCompleteButton.SetActive(false);
            NewTrain();

            HideBothTrainsPieces();
            ShowBuilderPieces();


            correct = 1;
            step = 1;
            rightFirst = 0;
            score = 0;
            select_type = -1;
            select_id = -1;

            currentPieceTypeSelected = Constants.NONE_SELECTED;
            lastPieceTypeSelected = Constants.NONE_SELECTED;

            CreateRandomTrain();
            TrainInCorner();

            firstStateChangeOccured = true;
            levelNumber = gameController.levelNumber;

           

        }

    }
}
