﻿using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Collections;
using System.Collections.Generic;


namespace RocketBarrierGame {
	
	public class EndGameComparisonController : MonoBehaviour {

		// game controller
		private MainGameController gameController;

		// buttons
		public GameObject restartButton;
		public GameObject tutorialButton;

		// button text
		public Text restartButtonText;

		private List<GameObject> mistakeMadePiecePair;
		private string feedbackUtteranceToMake = "";

		// timing variables
		private int secondsElapsed = 0;
		private float timeElapsed = 0f;
		private int timeToMakeFeedbackUtterance = 5; // seconds
		private int timeToShowRestartButton = 20; // seconds

		void Awake () {
			gameController = GameObject.Find ("GameManager").GetComponent<MainGameController> ();
			gameController.currentScene = Constants.END_GAME_COMPARISON_SCENE;
		}

		void Start () {

			// hide the restart button
			restartButton.SetActive(false);

			// show the rockets
			gameController.PlaceRocketsOnScreenSideBySide ();

			// update the performance metrics 
			gameController.UpdatePerformanceMetrics (true);

			// send robot utterance that the round is over and try to find one mistake to highlight
			gameController.SendRobotUtterance ("round-over", false);
			mistakeMadePiecePair = gameController.FindOneMistakeBetweenRockets ();
			feedbackUtteranceToMake = DetermineFeedbackToGive (mistakeMadePiecePair);

			// display the appropriate text in the tutorial button
			if (gameController.tutorialMode == Constants.TUTORIAL_ON) {
				tutorialButton.SetActive (true);
			} else if (gameController.tutorialMode == Constants.TUTORIAL_OFF) {
				tutorialButton.SetActive (false);
			}

			// initialize the timing variables
			secondsElapsed = 0;
			timeElapsed = 0f;
		}

		void Update () {

			if ((int)timeElapsed > secondsElapsed) {
				secondsElapsed = (int)timeElapsed;

				if (secondsElapsed == timeToMakeFeedbackUtterance) {
					gameController.SendRobotUtterance (feedbackUtteranceToMake, false);
				} else if (secondsElapsed == timeToShowRestartButton) {
					if (gameController.tutorialMode == Constants.TUTORIAL_ON) {
						restartButtonText.text = "Start New Game!";
						gameController.SendRobotUtterance ("tutorial-14-end-tutorial", false);
					} else {
						restartButtonText.text  = "Play Again!";
						gameController.SendRobotUtterance ("restart-prompt", false);
					}
					restartButton.SetActive (true);
				}
					
			}

			timeElapsed += Time.deltaTime;
		}

		private string DetermineFeedbackToGive (List<GameObject> mistakeMadePiecePair) {

			// if no mistakes were made
			if (mistakeMadePiecePair == null) {
				return "feedback-success";
			} else {
				bool hasSameColor = HasSameColorInList (mistakeMadePiecePair [0].GetComponent<RocketPieceInfo> ().colors, 
					                    mistakeMadePiecePair [1].GetComponent<RocketPieceInfo> ().colors);
				bool hasSameShape = (mistakeMadePiecePair [0].GetComponent<RocketPieceInfo> ().shape ==
				                    mistakeMadePiecePair [1].GetComponent<RocketPieceInfo> ().shape);
				bool hasSameFlameColor = (mistakeMadePiecePair [0].GetComponent<RocketPieceInfo> ().boosterFlameColor ==
				                         mistakeMadePiecePair [1].GetComponent<RocketPieceInfo> ().boosterFlameColor);
				if (mistakeMadePiecePair [0].tag == "Body") {
					if (hasSameColor) {
						return "feedback-body-wrong-piece-correct-color";
					} else {
						return "feedback-body-wrong-piece-wrong-color";
					}
				} else if (mistakeMadePiecePair [0].tag == "Engine") {
					if (!hasSameFlameColor && hasSameShape && mistakeMadePiecePair [0].GetComponent<RocketPieceInfo> ().boosterFlameColor != -1) {
						return "feedback-booster-correct-design-wrong-flamecolor";
					} else if (hasSameFlameColor && !hasSameShape && mistakeMadePiecePair [0].GetComponent<RocketPieceInfo> ().boosterFlameColor != -1) {
						return "feedback-booster-wrong-design-correct-flamecolor";
					} else if (hasSameColor) {
						return "feedback-booster-wrong-piece-correct-color";
					} else {
						return "feedback-booster-wrong-piece-wrong-color";
					}
				} else if (mistakeMadePiecePair [0].tag == "TopCone") {
					if (hasSameColor) {
						return "feedback-cone-wrong-shape-correct-color";
					} else if (hasSameShape) {
						return "feedback-cone-correct-shape-wrong-color";
					} else {
						return "feedback-cone-wrong-shape-wrong-color";
					}
				} else if (mistakeMadePiecePair [0].tag == "LeftFin" || mistakeMadePiecePair [0].tag == "RightFin") {
					if (hasSameColor) {
						return "feedback-fin-wrong-shape-correct-color";
					} else if (hasSameShape) {
						return "feedback-fin-correct-shape-wrong-color";
					} else {
						return "feedback-fin-wrong-shape-wrong-color";
					}
				} else {
					Logger.Log ("Mistake made with a piece with no tag - cannot send feedback");
					return "";
				}
			}
		}

		private bool HasSameColorInList (List<int> list1, List<int> list2) {

			foreach (int list1Item in list1) {
				foreach (int list2Item in list2) {
					if (list1Item == list2Item) {
						return true;
					}
				}
			}

			return false;

		}

		public void OnPlayAgainButtonPress () {

			gameController.StartNextGame ();

		}

		public void OnTutorialButtonPress () {

			gameController.EndTutorial ();

			gameController.StartFirstGame ();

		}

	}
}