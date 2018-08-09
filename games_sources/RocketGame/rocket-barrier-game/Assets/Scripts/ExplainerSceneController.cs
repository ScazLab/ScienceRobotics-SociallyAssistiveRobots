using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Collections;
using System.Collections.Generic;

namespace RocketBarrierGame {
	
	public class ExplainerSceneController : MonoBehaviour {

		// buttons
		public GameObject explainerCompleteButton;
		public GameObject switchToBuilderButton;
		public GameObject tutorialButton;

		// player indicator text
		public Text playerIndicatorText;

		// game controller
		private MainGameController gameController;

		// tutorial state
		private int tutorialState = Constants.TUTORIAL_STATE_NONE;

		// tutorial visuals
		public List<GameObject> bodyOutlineTutorialHighlights;
		public List<GameObject> boosterOutlineTutorialHighlights;
		public List<GameObject> finOutlineTutorialHighlights;
		public List<GameObject> coneOutlineTutorialHighlights;
		public List<GameObject> ventBodyPieceTutorialHighlights;
		public List<GameObject> fuelBodyPieceTutorialHighlights;
		public List<GameObject> boosterPieceTutorialHighlights;
		public List<GameObject> finPieceTutorialHighlights;
		public List<GameObject> conePieceTutorialHighlights;
		public List<GameObject> trashCanTutorialHighlights;

		// timer variables
		private int secondsElapsed = 0;
		private float timeElapsed = 0f;

		void Awake () {
			gameController = GameObject.Find ("GameManager").GetComponent<MainGameController> ();
			gameController.currentScene = Constants.EXPLAINER_SCENE;
		}

		void Start () {

			// if it's our first time entering this scene, set up the screen for the explainer
			// to build their rocket
			if (gameController.explainerMode == Constants.EXPLAINER_BUILDING) {
				
				// set up the scene
				gameController.SetUpExplainerScene ();

				// enable drag and drop functionality
				gameController.EnableDragAndDropGameplay ();

				// turn off the buttons
				explainerCompleteButton.SetActive (false);
				switchToBuilderButton.SetActive (false);

				// hide the tutorial highlights
				HideAllTutorialHighlights ();

				// subscribe to appropriate events
				Slot.OnPieceAddedToRocket += ExplainerPieceAddedToRocket;
				DragHandler.OnPieceRemovedByTrash += ExplainerPieceRemoved;

				// set the tutorial state and say the first tutorial prompt
				if (gameController.tutorialMode == Constants.TUTORIAL_ON) {
					tutorialState = Constants.TUTORIAL_STATE_TOUCH_BODY_FRAME;
					gameController.SendRobotUtterance ("tutorial-02-touch-body-pieces", false);
					ShowOneTutorialHighlight (bodyOutlineTutorialHighlights);
				}

			} 
			// if the explainer is going back to this scene to explain their rocket to the builder
			else {

				// set up the scene
				gameController.SetUpExplainerScene ();
				gameController.PlaceExistingRocketPiecesOnRocketOutlines ();
				gameController.DisableTouchOfRocketPieces ();

				// show the swap button, not the completed building button
				explainerCompleteButton.SetActive (false);
				switchToBuilderButton.SetActive (true);

				// hide the tutorial highlights
				HideAllTutorialHighlights ();

				// set the tutorial state
				if (gameController.tutorialMode == Constants.TUTORIAL_ON) {
					tutorialState = Constants.TUTORIAL_STATE_END;
				}
			}

			// have the rocket label (child/caregiver) represent the true state of the game
			if (gameController.childExplainer) {
				playerIndicatorText.text = Constants.CHILD_ROCKET_STRING;
			} else {
				playerIndicatorText.text = Constants.CAREGIVER_ROCKET_STRING;
			}

			// display the appropriate text in the tutorial button
			if (gameController.tutorialMode == Constants.TUTORIAL_ON) {
				tutorialButton.SetActive (true);
			} else if (gameController.tutorialMode == Constants.TUTORIAL_OFF) {
				tutorialButton.SetActive (false);
			}
		
		}

		void Update () {
			// enter this if statement every 1 second
			if ((int)timeElapsed > secondsElapsed) {
				secondsElapsed = (int)timeElapsed;

				if (tutorialState == Constants.TUTORIAL_STATE_TOUCH_BODY_FRAME) {
					if (CheckBodyOutlinesSelected ()) {
						gameController.SendRobotUtterance ("tutorial-03-vent-body-piece", false);
						tutorialState = Constants.TUTORIAL_STATE_PUT_ON_VENT;
						HideOneTutorialHighlight (bodyOutlineTutorialHighlights);
						ShowOneTutorialHighlight (ventBodyPieceTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_PUT_ON_VENT) {
					if (CheckBodyOneVent ()) {
						gameController.SendRobotUtterance ("tutorial-04-trash", false);
						tutorialState = Constants.TUTORIAL_STATE_TRASH_VENT;
						HideOneTutorialHighlight (ventBodyPieceTutorialHighlights);
						ShowOneTutorialHighlight (trashCanTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_TRASH_VENT) {
					if (CheckBodyEmpty ()) {
						gameController.SendRobotUtterance ("tutorial-05-put-on-body-pieces", false);
						tutorialState = Constants.TUTORIAL_STATE_FUEL_BODY_PIECES;
						HideOneTutorialHighlight (trashCanTutorialHighlights);
						ShowOneTutorialHighlight (fuelBodyPieceTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_FUEL_BODY_PIECES) {
					if (CheckBodyFourFuelPieces ()) {
						gameController.SendRobotUtterance ("tutorial-06-touch-booster-pieces", false);
						tutorialState = Constants.TUTORIAL_STATE_TOUCH_BOOSTER_FRAME;
						HideOneTutorialHighlight (fuelBodyPieceTutorialHighlights);
						ShowOneTutorialHighlight (boosterOutlineTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_TOUCH_BOOSTER_FRAME) {
					if (CheckBoosterOutlinesSelected ()) {
						gameController.SendRobotUtterance ("tutorial-07-put-on-booster-pieces", false);
						tutorialState = Constants.TUTORIAL_STATE_BOOSTER_PIECES;
						HideOneTutorialHighlight (boosterOutlineTutorialHighlights);
						ShowOneTutorialHighlight (boosterPieceTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_BOOSTER_PIECES) {
					if (CheckBoosterTwoBlueEngineRedFlames ()) {
						gameController.SendRobotUtterance ("tutorial-08-touch-fin-pieces", false);
						tutorialState = Constants.TUTORIAL_STATE_TOUCH_FIN_FRAME;
						HideOneTutorialHighlight (boosterPieceTutorialHighlights);
						ShowOneTutorialHighlight (finOutlineTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_TOUCH_FIN_FRAME) {
					if (CheckFinsOutlinesSelected ()) {
						gameController.SendRobotUtterance ("tutorial-09-put-on-fin-pieces", false);
						tutorialState = Constants.TUTORIAL_STATE_FIN_PIECES;
						HideOneTutorialHighlight (finOutlineTutorialHighlights);
						ShowOneTutorialHighlight (finPieceTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_FIN_PIECES) {
					if (CheckFinsTwoPurpleFins ()) {
						gameController.SendRobotUtterance ("tutorial-10-touch-cone-piece", false);
						tutorialState = Constants.TUTORIAL_STATE_TOUCH_CONE_FRAME;
						HideOneTutorialHighlight (finPieceTutorialHighlights);
						ShowOneTutorialHighlight (coneOutlineTutorialHighlights);
					}
				} else if (tutorialState == Constants.TUTORIAL_STATE_TOUCH_CONE_FRAME) {
					if (CheckConeOutlineSelected ()) {
						gameController.SendRobotUtterance ("tutorial-11-put-on-cone-piece", false);
						tutorialState = Constants.TUTORIAL_STATE_CONE_PIECE;
						HideOneTutorialHighlight (coneOutlineTutorialHighlights);
						ShowOneTutorialHighlight (conePieceTutorialHighlights);
					}
					
				} else if (tutorialState == Constants.TUTORIAL_STATE_CONE_PIECE) {
					if (CheckConeYellowCone()) {
						gameController.SendRobotUtterance ("tutorial-12-finish-explainer-building", false);
						tutorialState = Constants.TUTORIAL_STATE_FINISH_BUILDING_EXPLAINER;
						HideOneTutorialHighlight (conePieceTutorialHighlights);
					}
				}
			}

			timeElapsed += Time.deltaTime;
		}

		public void ExplainerFinishedBuilding () {

			// switch the explainer mode
			gameController.explainerMode = Constants.EXPLAINER_EXPLAINING;

			// disable the drag and drop functionality
			gameController.DisableDragAndDropGameplay ();

			// clear scene variables
			gameController.ClearSceneVariables ();

			//unsbuscribe from appropriate events
			Slot.OnPieceAddedToRocket -= ExplainerPieceAddedToRocket;
			DragHandler.OnPieceRemovedByTrash -= ExplainerPieceRemoved;

			if (gameController.tutorialMode == Constants.TUTORIAL_OFF) {
				// have the robot alert the builder to start building
				gameController.SendRobotUtterance ("builder-start-building", false);
			} else {
				gameController.SendRobotUtterance ("tutorial-13-begin-builder", false);
			}

			// load the builder scene
			SceneManager.LoadScene ("Builder");
		}

		void ExplainerPieceAddedToRocket (GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex) {
			if (gameController.IsRocketComplete ()) {
				explainerCompleteButton.SetActive (true);
			} else {
				explainerCompleteButton.SetActive (false);
			}
		}

		void ExplainerPieceRemoved (GameObject pieceToRemove, int pieceType, int oldParentType, int oldParentOutlineIndex) {
			if (gameController.IsRocketComplete ()) {
				explainerCompleteButton.SetActive (true);
			} else {
				explainerCompleteButton.SetActive (false);
			}
		}

		public void OnTutorialButtonPress () {

			// clear scene variables
			gameController.ClearSceneVariables ();

			gameController.EndTutorial ();

			gameController.StartFirstGame ();

		}

		public void SwitchFromExplainerToBuilder () {

			// clear scene variables
			gameController.ClearSceneVariables ();

			// load the builder scene
			SceneManager.LoadScene ("Builder");
		}


		/*------------------------------------------------------*/
		/*		Functions having to do with the tutorial		*/
		/*------------------------------------------------------*/


		bool CheckBodyEmpty () {
			
			int bodyPieceCount = 0;

			for (int i = 0; i < gameController.explainerRocket.bodyPieces.Count; i++) {
				if (gameController.explainerRocket.bodyPieces [i] != null) {
					bodyPieceCount++;
				}
			}

			if (bodyPieceCount == 0) {
				return true;
			} else {
				return false;
			}
		}

		bool CheckBodyFourFuelPieces () {

			int fuelPieceCount = 0;

			for (int i = 0; i < gameController.explainerRocket.bodyPieces.Count; i++) {
				if (gameController.explainerRocket.bodyPieces [i] != null) {
					if (gameController.explainerRocket.bodyPieces [i].name.Contains ("body_MTank")) {
						fuelPieceCount++;
					}
				}
			}

			if (fuelPieceCount == 4) {
				return true;
			} else {
				return false;
			}

		}

		bool CheckBodyOneVent () {

			for (int i = 0; i < gameController.explainerRocket.bodyPieces.Count; i++) {
				if (gameController.explainerRocket.bodyPieces [i] != null) {
					if (gameController.explainerRocket.bodyPieces [i].name.Contains ("body_Vent")) {
						return true;
					}
				}
			}

			return false;

		}

		bool CheckBodyOutlinesSelected () {

			if (gameController.GetCurrentPieceTypeSelected () == Constants.BODY) {
				return true;
			} else {
				return false;
			}

		}

		bool CheckBoosterOutlinesSelected () {

			if (gameController.GetCurrentPieceTypeSelected () == Constants.BOOSTER) {
				return true;
			} else {
				return false;
			}

		}

		bool CheckBoosterTwoBlueEngineRedFlames () {

			int correctBoosterCount = 0;

			for (int i = 0; i < gameController.explainerRocket.boosterPieces.Count; i++) {
				if (gameController.explainerRocket.boosterPieces [i] != null) {
					if (gameController.explainerRocket.boosterPieces [i].name.Contains ("engine_Cylinder_Red")) {
						correctBoosterCount++;
					}
				}
			}

			if (correctBoosterCount == 2) {
				return true;
			} else {
				return false;
			}

		}

		bool CheckConeOutlineSelected () {

			if (gameController.GetCurrentPieceTypeSelected () == Constants.CONE) {
				return true;
			} else {
				return false;
			}

		}

		bool CheckConeYellowCone () {

			if (gameController.explainerRocket.conePieces [0] != null) {
				if (gameController.explainerRocket.conePieces [0].name.Contains ("cone_Isosceles")) {
					return true;
				}
			}

			return false;

		}

		bool CheckFinsOutlinesSelected () {

			if (gameController.GetCurrentPieceTypeSelected () == Constants.FIN) {
				return true;
			} else {
				return false;
			}

		}

		bool CheckFinsTwoPurpleFins () {

			int correctFinCount = 0;

			for (int i = 0; i < gameController.explainerRocket.finPieces.Count; i++) {
				if (gameController.explainerRocket.finPieces [i] != null) {
					if (gameController.explainerRocket.finPieces [i].name.Contains ("fin_Fin")) {
						correctFinCount++;
					}
				}
			}

			if (correctFinCount == 2) {
				return true;
			} else {
				return false;
			}

		}

		void HideAllTutorialHighlights () {

			HideOneTutorialHighlight (bodyOutlineTutorialHighlights);
			HideOneTutorialHighlight (boosterOutlineTutorialHighlights);
			HideOneTutorialHighlight (finOutlineTutorialHighlights);
			HideOneTutorialHighlight (coneOutlineTutorialHighlights);
			HideOneTutorialHighlight (ventBodyPieceTutorialHighlights);
			HideOneTutorialHighlight (fuelBodyPieceTutorialHighlights);
			HideOneTutorialHighlight (boosterPieceTutorialHighlights);
			HideOneTutorialHighlight (finPieceTutorialHighlights);
			HideOneTutorialHighlight (conePieceTutorialHighlights);
			HideOneTutorialHighlight (trashCanTutorialHighlights);

		}

		void HideOneTutorialHighlight (List<GameObject> tutorialHighlightList) {

			foreach (GameObject tutorialHighlight in tutorialHighlightList) {
				tutorialHighlight.SetActive (false);
			}

		}

		void ShowOneTutorialHighlight (List<GameObject> tutorialHighlightList) {

			foreach (GameObject tutorialHighlight in tutorialHighlightList) {
				tutorialHighlight.SetActive (true);
			}

		}

	}
}