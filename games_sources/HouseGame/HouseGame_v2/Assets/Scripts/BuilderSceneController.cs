/*using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Collections;


namespace RocketBarrierGame {
	
	public class BuilderSceneController : MonoBehaviour {

		// buttons
		public GameObject builderCompleteButton;
		public GameObject switchToExplainerButton;

		// game controller
		private MainGameController gameController;

		void Awake () {
			gameController = GameObject.Find ("GameManager").GetComponent<MainGameController> ();
			gameController.currentScene = Constants.BUILDER_SCENE;
		}

		void Start () {

			// set up the scene
			gameController.SetUpExplainerScene ();
			gameController.PlaceExistingRocketPiecesOnRocketOutlines ();

			// enable drag and drop functionality
			gameController.EnableDragAndDropGameplay ();

			// initialize the buttons
			if (gameController.IsRocketComplete ()) {
				builderCompleteButton.SetActive (true);
			} else {
				builderCompleteButton.SetActive (false);
			}
			switchToExplainerButton.SetActive (true);

			// subscribe to appropriate events
			Slot.OnPieceAddedToRocket += BuilderPieceAddedToRocket;
			DragHandler.OnPieceRemovedByTrash += BuilderPieceRemoved;
		}

		public void BuilderFinishedBuilding() {

			// clear scene variables
			gameController.ClearSceneVariables ();

			// disable the drag and drop functionality
			gameController.DisableDragAndDropGameplay ();

			//unsbuscribe from appropriate events
			Slot.OnPieceAddedToRocket -= BuilderPieceAddedToRocket;
			DragHandler.OnPieceRemovedByTrash -= BuilderPieceRemoved;

			// load the end game comparison scene
			SceneManager.LoadScene ("EndGameComparison");
		}

		void BuilderPieceAddedToRocket (GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex) {
			if (gameController.IsRocketComplete ()) {
				builderCompleteButton.SetActive (true);
			} else {
				builderCompleteButton.SetActive (false);
			}
		}

		void BuilderPieceRemoved (GameObject pieceToRemove, int pieceType, int oldParentType, int oldParentOutlineIndex) {			
			if (gameController.IsRocketComplete ()) {
				builderCompleteButton.SetActive (true);
			} else {
				builderCompleteButton.SetActive (false);
			}
		}

		public void SwitchFromBuilderToExplainer () {

			// clear scene variables
			gameController.ClearSceneVariables ();

			// disable the drag and drop functionality
			gameController.DisableDragAndDropGameplay ();

			//unsbuscribe from appropriate events
			Slot.OnPieceAddedToRocket -= BuilderPieceAddedToRocket;
			DragHandler.OnPieceRemovedByTrash -= BuilderPieceRemoved;

			// load the explainer scene
			SceneManager.LoadScene ("Explainer");
		}
	}
}*/