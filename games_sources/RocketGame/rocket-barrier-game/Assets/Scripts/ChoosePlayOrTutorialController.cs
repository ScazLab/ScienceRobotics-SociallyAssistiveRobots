using UnityEngine;
using UnityEngine.SceneManagement;
using System.Collections;
using System.Collections.Generic;


namespace RocketBarrierGame {

	public class ChoosePlayOrTutorialController : MonoBehaviour {

		// game controller
		private MainGameController gameController;

		void Awake () {
			gameController = GameObject.Find ("GameManager").GetComponent<MainGameController> ();
			gameController.currentScene = Constants.CHOOSE_PLAY_OR_TUTORIAL_SCENE;
		}

		public void OnPlayButtonPress () {

			gameController.StartFirstGame ();

		}

		public void OnTutorialButtonPress () {

			gameController.StartTutorial ();

		}
	}
}