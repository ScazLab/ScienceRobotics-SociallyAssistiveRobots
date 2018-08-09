using UnityEngine;
using System.Collections;
using UnityEngine.EventSystems;


namespace TrainGame {
	
	public class Slot : MonoBehaviour, IDropHandler, IPointerClickHandler {

		public delegate void ClickForPanelChangeOutlinePiece(int pieceType);
		public static event ClickForPanelChangeOutlinePiece OnClickForPanelChangeOutlinePiece; 

		public delegate void PieceAddedToTrain(GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex);
		public static event PieceAddedToTrain OnPieceAddedToTrain;

		public bool isDashedOutlinePiece;

        public int index;

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
			/*if (DragHandler.itemBeingDragged.tag == "Wall") 
			{
				if (ChildBuilder.wall_piece_type == Constants.TYPE_WALL_NONE)
				{
                    ChildBuilder.wall_piece_type = DragHandler.itemBeingDragged.GetComponent<RocketPieceInfo> ().shape;
				}

				if (ChildBuilder.wall_piece_type != DragHandler.itemBeingDragged.GetComponent<RocketPieceInfo>().shape)
				{
					return;
				}
			}*/



			if (!item) {

				if (transform.tag == DragHandler.itemBeingDragged.tag ) {
					
					// determine the type of the itemBeingDragged
					int pieceIdentity = -1;
					if (DragHandler.itemBeingDragged.tag == "Front") {
						pieceIdentity = Constants.FRONT;
					} else if (DragHandler.itemBeingDragged.tag == "Body") {
						pieceIdentity = Constants.BODY;
					} else if (DragHandler.itemBeingDragged.tag == "Cabin") {
						pieceIdentity = Constants.CABIN;
					} else if (DragHandler.itemBeingDragged.tag == "Back" ) {
						pieceIdentity = Constants.BACK;
                    }
                    else if (DragHandler.itemBeingDragged.tag == "Smoke")
                    {
                        pieceIdentity = Constants.SMOKE;
                    }
                    else if (DragHandler.itemBeingDragged.tag == "Wheel")
                    {
                        pieceIdentity = Constants.WHEEL;
                    }

					// determine the type of the old parent, if the parent was a rocket outline, determine which one
					int oldParentType = -1;
					int oldParentOutlineIndex = -1;
					if (DragHandler.itemBeingDragged.transform.parent.parent.gameObject.name.Contains ("Outline")) {
						oldParentType = Constants.PARENT_TRAIN_OUTLINE;
                        if (pieceIdentity == Constants.FRONT || pieceIdentity == Constants.BODY || pieceIdentity == Constants.CABIN || pieceIdentity == Constants.BACK || pieceIdentity == Constants.SMOKE || pieceIdentity == Constants.WHEEL) {
							oldParentOutlineIndex = DragHandler.itemBeingDragged.transform.parent.GetSiblingIndex ();
						}
					}
                    else if (DragHandler.itemBeingDragged.transform.parent.parent.gameObject.name.Contains ("Piece"))
                    {
						oldParentType = Constants.PARENT_PIECE_PANEL;
					}

					// set the parent in the heirarchy to be the selected outline pieces that contains the rocket piece
					DragHandler.itemBeingDragged.transform.SetParent (transform);
					DragHandler.itemBeingDragged.transform.localScale = new Vector3 (1f, 1f, 1f);
					DragHandler.itemBeingDragged.GetComponent<RectTransform> ().localPosition = new Vector3 (0, 0, 0);

					// get the index of the new parent
					int newParentOutlineIndex = 0;
                    if (pieceIdentity == Constants.SMOKE)
                    {
                        if ((DragHandler.itemBeingDragged.transform.parent.name == "smoke1") || (DragHandler.itemBeingDragged.transform.parent.name == "s_smoke1"))
                        {
                            newParentOutlineIndex = 0;
                        }
                        else if ((DragHandler.itemBeingDragged.transform.parent.name == "smoke2") || (DragHandler.itemBeingDragged.transform.parent.name == "s_smoke2"))
                        {
                            newParentOutlineIndex = 1;
                        }
                        else
                        {
                            newParentOutlineIndex = 2;
                        }

                    }
                    if (pieceIdentity == Constants.WHEEL)
                    {
                        if ((DragHandler.itemBeingDragged.transform.parent.name == "wheel1") || (DragHandler.itemBeingDragged.transform.parent.name == "s_wheel1"))
                        {
                            newParentOutlineIndex = 0;
                        }
                        else if ((DragHandler.itemBeingDragged.transform.parent.name == "wheel2") || (DragHandler.itemBeingDragged.transform.parent.name == "s_wheel2"))
                        {
                            newParentOutlineIndex = 1;
                        }
                        else if ((DragHandler.itemBeingDragged.transform.parent.name == "wheel3") || (DragHandler.itemBeingDragged.transform.parent.name == "s_wheel3"))
                        {
                            newParentOutlineIndex = 2;
                        }
                        else
                        {
                            newParentOutlineIndex = 3;
                        }

                    }
                    /*if (pieceIdentity == Constants.WALL     )
                    { 
						newParentOutlineIndex = DragHandler.itemBeingDragged.transform.parent.GetSiblingIndex ();
					}
                    else
                    {
                        //newParentOutlineIndex = DragHandler.itemBeingDragged.transform.parent.GetSiblingIndex();
                        //newParentOutlineIndex = DragHandler.itemBeingDragged.transform.parent.gameObject.tag
                        if (pieceIdentity == Constants.DOOR)
                        {
                            if ((DragHandler.itemBeingDragged.transform.parent.name == "door0") || (DragHandler.itemBeingDragged.transform.parent.name == "sd0"))
                            {
                                newParentOutlineIndex = 0;
                            }
                            else
                            {
                                newParentOutlineIndex = 1;
                            }

                        }
                        if(pieceIdentity == Constants.PLANT)
                        {
                            if ((DragHandler.itemBeingDragged.transform.parent.gameObject.tag == "plant0") || (DragHandler.itemBeingDragged.transform.parent.name == "sp0"))
                            {
                                newParentOutlineIndex = 0;
                            }
                            else
                            {
                                newParentOutlineIndex = 1;
                            }
                        }
                        if (pieceIdentity == Constants.ROOF)
                        {
                            newParentOutlineIndex = 0;
                        }
                        
                    }*/

                    /* Note: pieces 'snap' to their slots as a result of having a horizontal layout group
				 	/*       on the selected outline piece that they're moving to	*/

                    // If a piece has been dragged successfully, we'll let the game manager know
                    OnPieceAddedToTrain (DragHandler.itemBeingDragged, pieceIdentity, oldParentType, oldParentOutlineIndex, newParentOutlineIndex);
				
				}
			} 
		}
        #endregion

        #region IPointerClickHandler implementation

        public void OnPointerClick(PointerEventData eventData)
        {
            // if there's a click on a piece that's a dashed outline piece then we want to trigger
            // the animation that brings in the panels with that kind of piece

            int selectedOutlineType = Constants.NONE_SELECTED;

            if (gameObject.tag == "Front")
            {
                selectedOutlineType = Constants.FRONT;
            }
            else if (gameObject.tag == "Body")
            {
				selectedOutlineType = Constants.BODY;
            }
            else if (gameObject.tag == "Cabin")
            {
                selectedOutlineType = Constants.CABIN;
            }
            else if (gameObject.tag == "Back")
            {
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

            OnClickForPanelChangeOutlinePiece(selectedOutlineType);
        




            /*if (isDashedOutlinePiece) {
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
			}*/
        }

		#endregion
	}
}