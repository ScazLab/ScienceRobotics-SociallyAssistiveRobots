using UnityEngine;
using System.Collections;
using UnityEngine.EventSystems;


namespace RocketBarrierGame {
	
	public class Slot : MonoBehaviour, IDropHandler, IPointerClickHandler {

		public delegate void ClickForPanelChangeOutlinePiece(int pieceType);
		public static event ClickForPanelChangeOutlinePiece OnClickForPanelChangeOutlinePiece; 

		public delegate void PieceAddedToRocket(GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex);
		public static event PieceAddedToRocket OnPieceAddedToRocket;

		public bool isDashedOutlinePiece;

		public GameObject item {
			get {
				if (transform.childCount > 0) {
					return transform.GetChild (0).gameObject;
				}
				return null;
			}
		}

		#region IDropHandler implementation
		public void OnDrop (PointerEventData eventData)
		{
			if (!item) {

				if (transform.tag == DragHandler.itemBeingDragged.tag || 
					(transform.tag.Contains("Fin") && DragHandler.itemBeingDragged.tag.Contains("Fin"))) {
					
					// determine the type of the itemBeingDragged
					int pieceIdentity = -1;
					if (DragHandler.itemBeingDragged.tag == "Body") {
						pieceIdentity = Constants.BODY;
					} else if (DragHandler.itemBeingDragged.tag == "Engine") {
						pieceIdentity = Constants.BOOSTER;
					} else if (DragHandler.itemBeingDragged.tag == "TopCone") {
						pieceIdentity = Constants.CONE;
					} else if (DragHandler.itemBeingDragged.tag == "LeftFin" || DragHandler.itemBeingDragged.tag == "RightFin") {
						pieceIdentity = Constants.FIN;
					}

					// determine the type of the old parent, if the parent was a rocket outline, determine which one
					int oldParentType = -1;
					int oldParentOutlineIndex = -1;
					if (DragHandler.itemBeingDragged.transform.parent.parent.gameObject.name.Contains ("Outline")) {
						oldParentType = Constants.PARENT_ROCKET_OUTLINE;
						if (pieceIdentity == Constants.FIN) {
							if (DragHandler.itemBeingDragged.transform.parent.gameObject.tag == "LeftFin") {
								oldParentOutlineIndex = 0;
							} else {
								oldParentOutlineIndex = 1;
							}
						} else if (pieceIdentity == Constants.BODY || pieceIdentity == Constants.BOOSTER || pieceIdentity == Constants.CONE) {
							oldParentOutlineIndex = DragHandler.itemBeingDragged.transform.parent.GetSiblingIndex ();
						}
					} else if (DragHandler.itemBeingDragged.transform.parent.parent.gameObject.name.Contains ("Piece")) {
						oldParentType = Constants.PARENT_PIECE_PANEL;
					}

					// set the parent in the heirarchy to be the selected outline pieces that contains the rocket piece
					DragHandler.itemBeingDragged.transform.SetParent (transform);
					DragHandler.itemBeingDragged.transform.localScale = new Vector3 (1f, 1f, 1f);
					DragHandler.itemBeingDragged.GetComponent<RectTransform> ().localPosition = new Vector3 (0, 0, 0);

					// get the index of the new parent
					int newParentOutlineIndex = -1;
					if (pieceIdentity == Constants.FIN) {
						if (DragHandler.itemBeingDragged.transform.parent.gameObject.tag == "LeftFin") {
							newParentOutlineIndex = 0;
						} else {
							newParentOutlineIndex = 1;
						}
					} else if (pieceIdentity == Constants.BODY || pieceIdentity == Constants.BOOSTER || pieceIdentity == Constants.CONE) {
						newParentOutlineIndex = DragHandler.itemBeingDragged.transform.parent.GetSiblingIndex ();
					}

					/* Note: pieces 'snap' to their slots as a result of having a horizontal layout group
				 	/*       on the selected outline piece that they're moving to	*/

					// If a piece has been dragged successfully, we'll let the game manager know
					OnPieceAddedToRocket (DragHandler.itemBeingDragged, pieceIdentity, oldParentType, oldParentOutlineIndex, newParentOutlineIndex);
				
				}
			} 
		}
		#endregion

		#region IPointerClickHandler implementation

		public void OnPointerClick (PointerEventData eventData)
		{
			// if there's a click on a piece that's a dashed outline piece then we want to trigger
			// the animation that brings in the panels with that kind of piece
			if (isDashedOutlinePiece) {
				if (OnClickForPanelChangeOutlinePiece != null) {
					int selectedOutlineType = Constants.NONE_SELECTED;
					if (gameObject.tag == "Body") {
						selectedOutlineType = Constants.BODY;
					} else if (gameObject.tag == "LeftFin" || gameObject.tag == "RightFin") {
						selectedOutlineType = Constants.FIN;
					} else if (gameObject.tag == "TopCone") {
						selectedOutlineType = Constants.CONE;
					} else if (gameObject.tag == "Engine") {
						selectedOutlineType = Constants.BOOSTER;
					}
					OnClickForPanelChangeOutlinePiece (selectedOutlineType);
				}
			}
		}

		#endregion
	}
}