using UnityEngine;
using System.Collections;
using UnityEngine.EventSystems;


namespace TrainGame {

	public class DragHandler : MonoBehaviour, IBeginDragHandler, IDragHandler, IEndDragHandler, IPointerClickHandler {

		public delegate void ClickForPanelChangeTrainPiece(int pieceType);
		public static event ClickForPanelChangeTrainPiece OnClickForPanelChangeTrainPiece; 

		public delegate void PieceClonedToPanel(GameObject pieceAdded);
		public static event PieceClonedToPanel OnPieceClonedToPanel;

		public delegate void PieceRemovedByTrash (GameObject pieceRemoved, int pieceType, int oldParentType, int oldParentOutlineIndex);
		public static event PieceRemovedByTrash OnPieceRemovedByTrash;

		public delegate void PieceDroppedOnQuestionMark (GameObject pieceDropped);
		//public static event PieceDroppedOnQuestionMark OnPieceDroppedOnQuestionMark;

		public static GameObject itemBeingDragged; 
		Vector3 startPosition;
		Transform startParent;
		Vector3 startScale;
		int startSiblingIndex;

		#region IBeginDragHandler implementation

		public void OnBeginDrag (PointerEventData eventData)
		{
			itemBeingDragged = gameObject;
			startPosition = transform.position;
			startParent = transform.parent;
			startScale = transform.localScale;
			startSiblingIndex = transform.GetSiblingIndex();
			// allows us to pass events from what's being dragged to the events behind it
			GetComponent<CanvasGroup> ().blocksRaycasts = false; 
	        
		}

		#endregion

		#region IDragHandler implementation

		public void OnDrag (PointerEventData eventData)
		{
	        var v3 = new Vector3(Input.mousePosition.x, Input.mousePosition.y, Camera.main.nearClipPlane);
			transform.position = Camera.main.ScreenToWorldPoint(v3); // Allows for camera space conversion
//			Debug.Log("Position of dragged object: (" + transform.position.x.ToString() + ", " + transform.position.y.ToString() + ", " + transform.position.z.ToString() + ")");
		
		}

		#endregion

		#region IEndDragHandler implementation

		public void OnEndDrag (PointerEventData eventData)
		{

			itemBeingDragged = null;
			GetComponent<CanvasGroup> ().blocksRaycasts = true;

			// determine the type of the itemBeingDragged
			int pieceIdentity = -1;
			if (gameObject.tag == "Front")
            {
				pieceIdentity = Constants.FRONT;
			}
            else if (gameObject.tag == "Body")
            {
				pieceIdentity = Constants.BODY;
			}
            else if (gameObject.tag == "Cabin")
            {
				pieceIdentity = Constants.CABIN;
			}
            else if (gameObject.tag == "Back")
            {
				pieceIdentity = Constants.BACK;
            }
            else if (gameObject.tag == "Smoke")
            {
                pieceIdentity = Constants.SMOKE;
            }
            else if (gameObject.tag == "Wheel")
            {
                pieceIdentity = Constants.WHEEL;
            }

			// if a piece was dragged from the rocket to the trash
			if (transform.parent.tag == "Trash" && startParent.tag != "PieceGroup")
            {
				int oldParentType = -1;
				int oldParentOutlineIndex = -1;
				if (startParent.parent.gameObject.name.Contains ("Outline"))
                {
					oldParentType = Constants.PARENT_TRAIN_OUTLINE;
                    if (pieceIdentity == Constants.FRONT || pieceIdentity == Constants.BODY || pieceIdentity == Constants.CABIN || pieceIdentity == Constants.BACK || pieceIdentity == Constants.SMOKE || pieceIdentity == Constants.WHEEL)
                    {
						oldParentOutlineIndex = startParent.GetSiblingIndex ();
                        //oldParentOutlineIndex = startParent.
					}
                    else
                    {

                    }
				} else if (startParent.parent.gameObject.name.Contains ("Piece")) {
					oldParentType = Constants.PARENT_PIECE_PANEL;
				}

				// remove the gameobject from any lists it's a part of in the game manager
				OnPieceRemovedByTrash (gameObject, pieceIdentity, oldParentType, oldParentOutlineIndex);
				// destroy it
				Destroy (gameObject);
			}
			// if the piece's parent is not of the same type or if it's parent is the panel
			// we send the piece back to where it came from
			else if (transform.parent == startParent || transform.tag != transform.parent.tag)
            {
				transform.position = startPosition;
				transform.SetParent (startParent);
			}
			// otherwise, we clone the piece and put a new one in the panel in place of the old one
			// as long as the parent is the panel
			else if (startParent.tag == "PieceGroup") {
				GameObject clone = Instantiate (gameObject);
				clone.transform.position = startPosition;
				clone.transform.SetParent (startParent);
				clone.transform.SetSiblingIndex (startSiblingIndex); // puts the cloned piece in the correct spot in the panel
				clone.transform.localScale = startScale;

				clone.transform.tag = gameObject.transform.tag;
				
				// let the game manager know that we've cloned a new piece
				OnPieceClonedToPanel (clone);
			}
		}

		#endregion

		#region IPointerClickHandler implementation

		// we want to enable players to select pieces to place on the rocket not only by touching
		// the empty slots, but also the pieces that are on those slots
		public void OnPointerClick (PointerEventData eventData)
		{
			// if the parent of the game object is an outline piece (has a slot script) 
			if (gameObject.transform.parent.gameObject.GetComponent<Slot> () != null) {
				if (OnClickForPanelChangeTrainPiece != null) {
					int selectedOutlineType = Constants.NONE_SELECTED;
					if (gameObject.tag == "Front") {
						selectedOutlineType = Constants.FRONT;
					} else if (gameObject.tag == "Body") {
						selectedOutlineType = Constants.BODY;
					} else if (gameObject.tag == "Cabin") {
						selectedOutlineType = Constants.CABIN;
					} else if (gameObject.tag == "Back") {
						selectedOutlineType = Constants.BACK;
                    }
                    else if (gameObject.tag == "Smoke")
                    {
                        selectedOutlineType = Constants.SMOKE;
                    }
                    else if (gameObject.tag == "Wheel")
                    {
                        selectedOutlineType = Constants.WHEEL;
                    }
					OnClickForPanelChangeTrainPiece (selectedOutlineType);
				}
			}
		}

		#endregion
	}

}