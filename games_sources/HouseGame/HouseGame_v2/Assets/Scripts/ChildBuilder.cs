using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using System.Collections;
using System.Collections.Generic;
namespace HouseGame {

    public class ConstructedHouse
    {

        public List<GameObject> wallPieces;
        public List<GameObject> roofPieces;
        public List<GameObject> plantPieces;
        public List<GameObject> doorPieces;

        public int wall;
        public int roof;
        public int door1;
        public int door2;
        public int plant1;
        public int plant2;

        public int n_wall_pieces = 0;
        public int n_roof_pieces = 0;
        public int n_door_pieces = 0;
        public int n_plant_pieces = 0;

        public ConstructedHouse()
        {

        }

        public ConstructedHouse(int numWallPieces, int numRoofPieces, int numPlantPieces, int numDoorPieces)
        {
            // initialize the wallPieces list
            wallPieces = new List<GameObject>();
            // initialize the roofPieces list
            roofPieces = new List<GameObject>();
            // initialize the plantPieces list
            plantPieces = new List<GameObject>();
            // initialize the doorPieces list
            doorPieces = new List<GameObject>();

            for (int i = 0; i < numWallPieces; i++)
            {
                wallPieces.Add(null);
            }
            for (int i = 0; i < numRoofPieces; i++)
            {
                roofPieces.Add(null);
            }
            for (int i = 0; i < numPlantPieces; i++)
            {
                plantPieces.Add(null);
            }
            for (int i = 0; i < numDoorPieces; i++)
            {
                doorPieces.Add(null);
            }
        }
    }


    public class ChildBuilder : MonoBehaviour {

        int levelNumber;
        // panels where the outline and rocket pieces will be placed
        private Transform DashedOutlinesPanel;
        private Transform SelectedOutlinesPanel;
        private Transform HousePiecesLeftPanel;
        private Transform HousePiecesRightPanel;

        // keeps track of the current wall piece (only one type allowed)
        public static int wall_piece_type = Constants.TYPE_WALL_NONE;


        // outline slots
        private List<GameObject> dashedWallOutlineSlots = new List<GameObject>();
        private List<GameObject> dashedRoofOutlineSlot = new List<GameObject>();
        private List<GameObject> dashedPlantOutlineSlots = new List<GameObject>();
        private List<GameObject> dashedDoorOutlineSlots = new List<GameObject>();

        // panel body pieces
        private List<GameObject> wallPanelHousePieces = new List<GameObject>();
        private List<GameObject> roofPanelHousePieces = new List<GameObject>();
        private List<GameObject> plantPanelHousePieces = new List<GameObject>();
        private List<GameObject> doorPanelHousePieces = new List<GameObject>();

        // keep track of the 2 rockets and all of their pieces
        public ConstructedHouse builderHouse;
        public ConstructedHouse explainerHouse;

        // animator and related variables
        private Animator panelsAnimator;
        private bool firstStateChangeOccured = false;

        public House builtHouse = new House();

        // type of piece selected
        private int currentPieceTypeSelected;
        private int lastPieceTypeSelected;


        // buttons
        public GameObject explainerCompleteButton;
		public GameObject switchToBuilderButton;

		// game controller
		private MainGameController gameController;


		void Start ()
        {
            //gameController.SendRobotUtterance("child-builder-start", false, -1, -1, -1);
            Logger.Log("Entering Here");
            DontDestroyOnLoad(gameObject);
            // initialize pieceTypeSelected
            currentPieceTypeSelected = Constants.NONE_SELECTED;
            lastPieceTypeSelected = Constants.NONE_SELECTED;

            gameController = GameObject.Find("GameManager").GetComponent<MainGameController>();
            gameController.currentScene = Constants.EXPLAINER_SCENE;
            levelNumber = gameController.levelNumber;

            if (levelNumber == 1)
                explainerHouse = new ConstructedHouse(4,1,0,1);
            if (levelNumber == 2)
                explainerHouse = new ConstructedHouse(4, 1, 1, 1);
            if (levelNumber == 3)
                explainerHouse = new ConstructedHouse(4, 1, 1, 2);
            if (levelNumber == 4)
                explainerHouse = new ConstructedHouse(4, 1, 2, 2);

			// set up the scene
			SetUpExplainerScene ();

			// enable drag and drop functionality
			EnableDragAndDropGameplay ();

			// turn off the buttons
			explainerCompleteButton.SetActive (false);
			//switchToBuilderButton.SetActive (false);

			// subscribe to appropriate events
			Slot.OnPieceAddedToHouse += ExplainerPieceAddedToHouse;
			DragHandler.OnPieceRemovedByTrash += ExplainerPieceRemoved;
            wall_piece_type = Constants.TYPE_WALL_NONE;

            gameController.SendRobotUtterance("child-builder-start", false, -1, -1, -1);

        }


		public void ExplainerFinishedBuilding () {

			// switch the explainer mode
			gameController.explainerMode = Constants.EXPLAINER_EXPLAINING;

			// disable the drag and drop functionality
			DisableDragAndDropGameplay ();

			// clear scene variables
			ClearSceneVariables ();

			//unsbuscribe from appropriate events
			Slot.OnPieceAddedToHouse -= ExplainerPieceAddedToHouse;
			DragHandler.OnPieceRemovedByTrash -= ExplainerPieceRemoved;

            FinishBuilding();

        }

		void ExplainerPieceAddedToHouse (GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex) {
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

            if (gameController.internalGameState == Constants.END_GAME)
            {
                gameController.SendRobotUtterance("end-game", false, -1, -1, -1);
                Logger.Log("GoingToQuit");
                //Application.Quit();
            }

            // load the builder scene
            if (gameController.levelNumber == 1)
            {
                    SceneManager.LoadScene("Robot_guessing_L1");
            }
            if (gameController.levelNumber == 2)
            {
                    SceneManager.LoadScene("Robot_guessing_L2");
            }
            if (gameController.levelNumber == 3)
            {
                    SceneManager.LoadScene("Robot_guessing_L3");
            }
            if (gameController.levelNumber == 4)
            {
                    SceneManager.LoadScene("Robot_guessing_L4");
            }
        }




        public void ClearSceneVariables()
        {

            // dashed outline slots
            dashedWallOutlineSlots.Clear();
            dashedRoofOutlineSlot.Clear();
            dashedPlantOutlineSlots.Clear();
            dashedDoorOutlineSlots.Clear();

            // panel pieces
            wallPanelHousePieces.Clear();
            doorPanelHousePieces.Clear();
            roofPanelHousePieces.Clear();
            plantPanelHousePieces.Clear();

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

            if (levelNumber == 1)
            {
                if ((explainerHouse.n_wall_pieces == 4) && (explainerHouse.n_roof_pieces == 1) && (explainerHouse.n_door_pieces == 1))
                {
                    if (gameController.internalGameState == Constants.END_GAME)
                    {
                        // send goobye message

                        //Application.Quit();
                    }
                    return true;
                }
            }
            if (levelNumber == 2)
            {

                if ((explainerHouse.n_wall_pieces == 4) && (explainerHouse.n_roof_pieces == 1) && (explainerHouse.n_door_pieces == 1) && (explainerHouse.n_plant_pieces == 1))
                {
                    if (gameController.internalGameState == Constants.END_GAME)
                    {
                        // send goobye message

                        //Application.Quit();
                    }
                    return true;
                }
            }
            if (levelNumber == 3)
            {
                if ((explainerHouse.n_wall_pieces == 4) && (explainerHouse.n_roof_pieces == 1) && (explainerHouse.n_door_pieces == 2) && (explainerHouse.n_plant_pieces == 1))
                {
                    if (gameController.internalGameState == Constants.END_GAME)
                    {
                        // send goobye message

                        //Application.Quit();
                    }
                    return true;
                }
            }
            if (levelNumber == 4)
            {
                if ((explainerHouse.n_wall_pieces == 4) && (explainerHouse.n_roof_pieces == 1) && (explainerHouse.n_door_pieces == 2) && (explainerHouse.n_plant_pieces == 2))
                {
                    if (gameController.internalGameState == Constants.END_GAME)
                    {
                        // send goobye message

                        //Application.Quit();
                    }
                    return true;
                }
            }
            return false;

        }
        void HideSelectedWallPieces()
        {
            GameObject b0 = GameObject.Find("sb0");
            GameObject b1 = GameObject.Find("sb1");
            GameObject b2 = GameObject.Find("sb2");
            GameObject b3 = GameObject.Find("sb3");
            b0.gameObject.GetComponent<Image>().enabled = false;
            b1.gameObject.GetComponent<Image>().enabled = false;
            b2.gameObject.GetComponent<Image>().enabled = false;
            b3.gameObject.GetComponent<Image>().enabled = false;
        }

        void ShowSelectedWallPieces()
        {
            GameObject b0 = GameObject.Find("sb0");
            GameObject b1 = GameObject.Find("sb1");
            GameObject b2 = GameObject.Find("sb2");
            GameObject b3 = GameObject.Find("sb3");
            b0.gameObject.GetComponent<Image>().enabled = true;
            b1.gameObject.GetComponent<Image>().enabled = true;
            b2.gameObject.GetComponent<Image>().enabled = true;
            b3.gameObject.GetComponent<Image>().enabled = true;
        }

        void HideSelectedDoorPieces()
        {
            GameObject d0 = GameObject.Find("sd0");
            d0.gameObject.GetComponent<Image>().enabled = false;
            if (levelNumber > 2)
            {
                GameObject d1 = GameObject.Find("sd1");
                d1.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedDoorPieces()
        {
            GameObject d0 = GameObject.Find("sd0");
            d0.gameObject.GetComponent<Image>().enabled = true;
            if (levelNumber > 2)
            {
                GameObject d1 = GameObject.Find("sd1");
                d1.gameObject.GetComponent<Image>().enabled = true;
            }
        }

        void HideSelectedRoofPieces()
        {
            GameObject r0 = GameObject.Find("sr0");
            r0.gameObject.GetComponent<Image>().enabled = false;
        }

        void ShowSelectedRoofPieces()
        {
            GameObject r0 = GameObject.Find("sr0");
            r0.gameObject.GetComponent<Image>().enabled = true;
        }

        void HideSelectedPlantPieces()
        {
            GameObject p0 = GameObject.Find("sp0");
            p0.gameObject.GetComponent<Image>().enabled = false;
            if (levelNumber > 3)
            {
                GameObject p1 = GameObject.Find("sp1");
                p1.gameObject.GetComponent<Image>().enabled = false;
            }
        }

        void ShowSelectedPlantPieces()
        {
            GameObject p0 = GameObject.Find("sp0");
            p0.gameObject.GetComponent<Image>().enabled = true;
            if (levelNumber > 3)
            {
                GameObject p1 = GameObject.Find("sp1");
                p1.gameObject.GetComponent<Image>().enabled = true;
            }
        }


        // This is our cue to hide the old pieces on the panels and show the new ones
        void PanelIn()
        {

            // hide the rocket pieces of the old selected piece type
            if (lastPieceTypeSelected == Constants.WALL)
            {
                HideSelectedWallPieces();
                HidePieces(wallPanelHousePieces);
            }
            else if (lastPieceTypeSelected == Constants.ROOF)
            {
                HideSelectedRoofPieces();
                HidePieces(roofPanelHousePieces);
            }
            else if (lastPieceTypeSelected == Constants.PLANT)
            {
                HideSelectedPlantPieces();
                HidePieces(plantPanelHousePieces);
            }
            else if (lastPieceTypeSelected == Constants.DOOR)
            {
                HideSelectedDoorPieces();
                HidePieces(doorPanelHousePieces);
            }

            // show the rocket pieces of the new selected piece type
            if (currentPieceTypeSelected == Constants.WALL)
            {
                ShowSelectedWallPieces();
                ShowPieces(wallPanelHousePieces);
            }
            else if (currentPieceTypeSelected == Constants.ROOF)
            {
                ShowSelectedRoofPieces();
                ShowPieces(roofPanelHousePieces);
            }
            else if (currentPieceTypeSelected == Constants.PLANT)
            {
                ShowSelectedPlantPieces();
                ShowPieces(plantPanelHousePieces);
            }
            else if (currentPieceTypeSelected == Constants.DOOR)
            {
                ShowSelectedDoorPieces();
                ShowPieces(doorPanelHousePieces);
            }

            // indicate activity
            gameController.recentScreenInteraction = true;
        }

        void PieceAddedToPanel(GameObject pieceAdded)
        {
            if (pieceAdded.tag == "Wall")
            {
                wallPanelHousePieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Plant")
            {
                plantPanelHousePieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Roof")
            {
                roofPanelHousePieces.Add(pieceAdded);
            }
            else if (pieceAdded.tag == "Door")
            {
                doorPanelHousePieces.Add(pieceAdded);
            }

            // indicate activity
            gameController.recentScreenInteraction = true;
        }

        void PieceAddedToHouse(GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex)
        {

            // if we've moved a piece within the rocket, we must remove it from its previously stored location
            if (oldParentType == Constants.PARENT_HOUSE_OUTLINE)
            {
                if (pieceType == Constants.WALL)
                {
                    explainerHouse.wallPieces[oldParentOutlineIndex] = null;
                }
                else if (pieceType == Constants.ROOF)
                {
                    explainerHouse.roofPieces[oldParentOutlineIndex] = null;
                }
                else if (pieceType == Constants.PLANT)
                {
                    builderHouse.plantPieces[oldParentOutlineIndex] = null;
                }
                else if (pieceType == Constants.DOOR)
                {
                    explainerHouse.doorPieces[oldParentOutlineIndex] = null;
                }
            }
            // if the piece added to the rocket was taken from the panel, remove it from the panel pieces list
            // and remove the piece from the current lists of rocket pieces
            else
            {
                int removalIndex = -1;
                if (pieceType == Constants.WALL)
                {
                    for (int i = 0; i < wallPanelHousePieces.Count; i++)
                    {
                        if (pieceAdded.name == wallPanelHousePieces[i].name)
                        {
                            explainerHouse.n_wall_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    wallPanelHousePieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.ROOF)
                {
                    for (int i = 0; i < roofPanelHousePieces.Count; i++)
                    {
                        if (pieceAdded.name == roofPanelHousePieces[i].name)
                        {
                            explainerHouse.n_roof_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    roofPanelHousePieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.PLANT)
                {
                    for (int i = 0; i < plantPanelHousePieces.Count; i++)
                    {
                        if (pieceAdded.name == plantPanelHousePieces[i].name)
                        {
                            explainerHouse.n_plant_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    plantPanelHousePieces.RemoveAt(removalIndex);
                }
                else if (pieceType == Constants.DOOR)
                {
                    for (int i = 0; i < doorPanelHousePieces.Count; i++)
                    {
                        if (pieceAdded.name == doorPanelHousePieces[i].name)
                        {
                            explainerHouse.n_door_pieces++;
                            removalIndex = i;
                            break;
                        }
                    }
                    doorPanelHousePieces.RemoveAt(removalIndex);
                }
            }

            // add the piece added to the new location in the appropriate rocket pieces list
            if (pieceType == Constants.WALL)
            {

                //GameObject wallPrefabAdded = null;
                int i = 0;
                foreach (GameObject wallPrefab in gameController.wallHousePieceOptions)
                {
                    if (pieceAdded.name.Contains(wallPrefab.name))
                    {
                        explainerHouse.wall = i;
                        //wallPrefabAdded = wallPrefab;
                        break;
                    }
                    i++;
                }
                /*if (gameController.currentScene == Constants.EXPLAINER_SCENE)
                {
                    explainerHouse.wallPieces[newParentOutlineIndex] = wallPrefabAdded;
                }
                else if (gameController.currentScene == Constants.BUILDER_SCENE)
                {
                    builderHouse.wallPieces[newParentOutlineIndex] = wallPrefabAdded;
                }*/
            }
            else if (pieceType == Constants.ROOF)
            {
                GameObject roofPrefabAdded = null;
                int i = 0;
                foreach (GameObject roofPrefab in gameController.roofHousePieceOptions)
                {
                    if (pieceAdded.name.Contains(roofPrefab.name))
                    {
                        explainerHouse.roof = i;
                        roofPrefabAdded = roofPrefab;
                        break;
                    }
                    i++;
                }
                if (gameController.currentScene == Constants.EXPLAINER_SCENE)
                {
                    explainerHouse.roofPieces[newParentOutlineIndex] = roofPrefabAdded;
                }
                else if (gameController.currentScene == Constants.BUILDER_SCENE)
                {
                    builderHouse.roofPieces[newParentOutlineIndex] = roofPrefabAdded;
                }
            }
            else if (pieceType == Constants.DOOR)
            {
                GameObject doorPrefabAdded = null;
                int i = 0;
                foreach (GameObject doorPrefab in gameController.doorHousePieceOptions)
                {
                    if (pieceAdded.name.Contains(doorPrefab.name))
                    {
                        if (newParentOutlineIndex == 0)
                        {
                            explainerHouse.door1 = i;
                        }
                        else
                        {
                            explainerHouse.door2 = i;
                        }
                        doorPrefabAdded = doorPrefab;
                        break;
                    }
                    i++;
                }
                if (gameController.currentScene == Constants.EXPLAINER_SCENE)
                {
                    explainerHouse.doorPieces[newParentOutlineIndex] = doorPrefabAdded;
                }
                else if (gameController.currentScene == Constants.BUILDER_SCENE)
                {
                    builderHouse.doorPieces[newParentOutlineIndex] = doorPrefabAdded;
                }
            }
            else if (pieceType == Constants.PLANT)
            {
                int i = 0;
                GameObject plantPrefabAdded = null;
                foreach (GameObject plantPrefab in gameController.plantHousePieceOptions)
                {
                    if (pieceAdded.name.Contains(plantPrefab.name))
                    {
                        if (newParentOutlineIndex == 0)
                        {
                            explainerHouse.plant1 = i;
                        }
                        else
                        {
                            explainerHouse.plant2 = i;
                        }
                        //explainerHouse.plant1 = i;
                        plantPrefabAdded = plantPrefab;
                        break;
                    }
                    ++i;
                }
                if (gameController.currentScene == Constants.EXPLAINER_SCENE)
                {
                    explainerHouse.plantPieces[newParentOutlineIndex] = plantPrefabAdded;
                }
                else if (gameController.currentScene == Constants.BUILDER_SCENE)
                {
                    builderHouse.plantPieces[newParentOutlineIndex] = plantPrefabAdded;
                }
            }

            // indicate activity
            gameController.recentScreenInteraction = true;

        }

        void PieceRemoved(GameObject pieceToRemove, int pieceType, int oldParentType, int oldParentOutlineIndex)
        {

            if (pieceType == Constants.WALL)
            {
                explainerHouse.n_wall_pieces--;
                if (explainerHouse.n_wall_pieces == 0)
                {
                    wall_piece_type = Constants.TYPE_WALL_NONE;
                }
                //explainerHouse.wallPieces[oldParentOutlineIndex] = null;
            }
            if (pieceType == Constants.ROOF)
            {
                explainerHouse.n_roof_pieces--;
            }
            if (pieceType == Constants.DOOR)
            {
                explainerHouse.n_door_pieces--;
            }
            if (pieceType == Constants.PLANT)
            {
                explainerHouse.n_plant_pieces--;
            }
        }


        void PlaceOutlineAndHousePieces()
        {

            /*-------------------------------------- Rocket Pieces on the Panels --------------------------------------*/

            // wall pieces
            Transform wallPieceParentPanelLeft = HousePiecesLeftPanel.FindChild("LeftBodyPieces");
            Transform wallPieceParentPanelRight = HousePiecesRightPanel.FindChild("RightBodyPieces");
            int totalWallPieceOptions = Constants.LEVEL_INFO[levelNumber].wallPieceOptionNames.Count;
            int currentWallPieceOptions = 0;
            for (int i = 0; i < totalWallPieceOptions; i++)
            {
                for (int j = 0; j < gameController.wallHousePieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].wallPieceOptionNames[i] == gameController.wallHousePieceOptions[j].name)
                    {
                        GameObject panelWallPieceClone = Instantiate(gameController.wallHousePieceOptions[j]);
                        if (currentWallPieceOptions * 2 < totalWallPieceOptions)
                        {
                            panelWallPieceClone.transform.SetParent(wallPieceParentPanelLeft);
                        }
                        else
                        {
                            panelWallPieceClone.transform.SetParent(wallPieceParentPanelRight);
                        }
                        panelWallPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        wallPanelHousePieces.Add(panelWallPieceClone);
                        currentWallPieceOptions++;
                    }
                }
            }

            // roof pieces
            Transform roofPieceParentPanelLeft = HousePiecesLeftPanel.FindChild("LeftConePieces");
            Transform roofPieceParentPanelRight = HousePiecesRightPanel.FindChild("RightConePieces");
            int totalRoofPieceOptions = Constants.LEVEL_INFO[levelNumber].roofPieceOptionNames.Count;
            int currentRoofPieceOptions = 0;
            for (int i = 0; i < totalRoofPieceOptions; i++)
            {
                for (int j = 0; j < gameController.roofHousePieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].roofPieceOptionNames[i] == gameController.roofHousePieceOptions[j].name)
                    {
                        GameObject panelRoofPieceClone = Instantiate(gameController.roofHousePieceOptions[j]);
                        if (currentRoofPieceOptions * 2 < totalRoofPieceOptions)
                        {
                            panelRoofPieceClone.transform.SetParent(roofPieceParentPanelLeft);
                        }
                        else
                        {
                            panelRoofPieceClone.transform.SetParent(roofPieceParentPanelRight);
                        }
                        panelRoofPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        roofPanelHousePieces.Add(panelRoofPieceClone);
                        currentRoofPieceOptions++;
                    }
                }
            }

            // plant pieces
            Transform plantPieceParentPanelLeft = HousePiecesLeftPanel.FindChild("LeftFinPieces");
            Transform plantPieceParentPanelRight = HousePiecesRightPanel.FindChild("RightFinPieces");
            int totalPlantPieceOptions = Constants.LEVEL_INFO[levelNumber].plantsPieceOptionNames.Count;
            int currentPlantPieceOptions = 0;
            for (int i = 0; i < totalPlantPieceOptions; i++)
            {
                for (int j = 0; j < gameController.plantHousePieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].plantsPieceOptionNames[i] == gameController.plantHousePieceOptions[j].name)
                    {
                        GameObject panelPlantPieceClone = Instantiate(gameController.plantHousePieceOptions[j]);
                        if (currentPlantPieceOptions * 2 < totalPlantPieceOptions)
                        {
                            panelPlantPieceClone.transform.SetParent(plantPieceParentPanelLeft);
                        }
                        else
                        {
                            panelPlantPieceClone.transform.SetParent(plantPieceParentPanelRight);
                        }
                        panelPlantPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        plantPanelHousePieces.Add(panelPlantPieceClone);
                        currentPlantPieceOptions++;
                    }
                }
            }

            // door pieces
            Transform doorPieceParentPanelLeft = HousePiecesLeftPanel.FindChild("LeftDoorPieces");
            Transform doorPieceParentPanelRight = HousePiecesRightPanel.FindChild("RightDoorPieces");
            int totalDoorPieceOptions = Constants.LEVEL_INFO[levelNumber].doorPieceOptionNames.Count;
            int currentDoorPieceOptions = 0;
            for (int i = 0; i < totalDoorPieceOptions; i++)
            {
                for (int j = 0; j < gameController.doorHousePieceOptions.Count; j++)
                {
                    if (Constants.LEVEL_INFO[levelNumber].doorPieceOptionNames[i] == gameController.doorHousePieceOptions[j].name)
                    {
                        GameObject panelDoorPieceClone = Instantiate(gameController.doorHousePieceOptions[j]);
                        if (currentDoorPieceOptions * 2 < totalDoorPieceOptions)
                        {
                            panelDoorPieceClone.transform.SetParent(doorPieceParentPanelLeft);
                        }
                        else
                        {
                            panelDoorPieceClone.transform.SetParent(doorPieceParentPanelRight);
                        }
                        panelDoorPieceClone.GetComponent<RectTransform>().transform.localScale = new Vector3(1f, 1f, 1f);
                        doorPanelHousePieces.Add(panelDoorPieceClone);
                        currentDoorPieceOptions++;
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
            HousePiecesLeftPanel = GameObject.Find("RocketPiecesLeft").transform;
            HousePiecesRightPanel = GameObject.Find("RocketPiecesRight").transform;
            panelsAnimator = GameObject.Find("RocketField").GetComponent<Animator>();

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
            // only switch the panels if we're wanting to put on a different type of piece
            if (selectedOutlineType != currentPieceTypeSelected)
            {
                // play the animations to hide the sidebars
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
            if (lastPieceTypeSelected == Constants.WALL)
                ShowPieces(dashedWallOutlineSlots);
            else if (lastPieceTypeSelected == Constants.ROOF)
                ShowPieces(dashedRoofOutlineSlot);
            else if (lastPieceTypeSelected == Constants.PLANT)
                ShowPieces(dashedPlantOutlineSlots);
            else if (lastPieceTypeSelected == Constants.DOOR)
                ShowPieces(dashedDoorOutlineSlots);

            // hide the dashed outlines and show the selected outlines of the new selected piece type
            if (currentPieceTypeSelected == Constants.NONE_SELECTED)
            {

                // show all dashed pieces 
                ShowPieces(dashedWallOutlineSlots);
                ShowPieces(dashedRoofOutlineSlot);
                ShowPieces(dashedPlantOutlineSlots);
                ShowPieces(dashedDoorOutlineSlots);

                // hide all the pieces
                HidePieces(wallPanelHousePieces);
                HidePieces(roofPanelHousePieces);
                HidePieces(plantPanelHousePieces);
                HidePieces(doorPanelHousePieces);

            }
            else if (currentPieceTypeSelected == Constants.WALL)
                HidePieces(dashedWallOutlineSlots);
            else if (currentPieceTypeSelected == Constants.ROOF)
                HidePieces(dashedRoofOutlineSlot);
            else if (currentPieceTypeSelected == Constants.PLANT)
                HidePieces(dashedPlantOutlineSlots);
            else if (currentPieceTypeSelected == Constants.DOOR)
                HidePieces(dashedDoorOutlineSlots);

        }

        public void DisableDragAndDropGameplay()
        {

            // subscribe to the events that indicate clicks on outline pieces
            Slot.OnClickForPanelChangeOutlinePiece -= TriggerPanelChange;

            // subscribe to the event that alerts the game manager of pieces added to the rocket
            Slot.OnPieceAddedToHouse -= PieceAddedToHouse;

            // subscribe to the event that alerts the game manager of the panel going in
            PanelAnimationEventHandler.OnTriggerPanelIn -= PanelIn;
            DragHandler.OnClickForPanelChangeHousePiece -= TriggerPanelChange;

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
            Slot.OnPieceAddedToHouse += PieceAddedToHouse;

            // subscribe to the event that alerts the game manager of the panel going in
            PanelAnimationEventHandler.OnTriggerPanelIn += PanelIn;
            DragHandler.OnClickForPanelChangeHousePiece += TriggerPanelChange;

            // subscribe to the event that alerts the game manager of cloned pieces added to the panel
            DragHandler.OnPieceClonedToPanel += PieceAddedToPanel;

            // subscribe to the event that alerts the game manager of a deleted piece (via trash)
            DragHandler.OnPieceRemovedByTrash += PieceRemoved;

        }

    }
}