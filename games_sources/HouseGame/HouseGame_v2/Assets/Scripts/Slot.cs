using UnityEngine;
using System.Collections;
using UnityEngine.EventSystems;


namespace HouseGame {
	
	public class Slot : MonoBehaviour, IDropHandler, IPointerClickHandler {

		public delegate void ClickForPanelChangeOutlinePiece(int pieceType);
		public static event ClickForPanelChangeOutlinePiece OnClickForPanelChangeOutlinePiece; 

		public delegate void PieceAddedToHouse(GameObject pieceAdded, int pieceType, int oldParentType, int oldParentOutlineIndex, int newParentOutlineIndex);
		public static event PieceAddedToHouse OnPieceAddedToHouse;

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
			if (DragHandler.itemBeingDragged.tag == "Wall") 
			{
				if (ChildBuilder.wall_piece_type == Constants.TYPE_WALL_NONE)
				{
                    ChildBuilder.wall_piece_type = DragHandler.itemBeingDragged.GetComponent<RocketPieceInfo> ().shape;
				}

				if (ChildBuilder.wall_piece_type != DragHandler.itemBeingDragged.GetComponent<RocketPieceInfo>().shape)
				{
					return;
				}
			}



			if (!item) {

				if (transform.tag == DragHandler.itemBeingDragged.tag ) {
					
					// determine the type of the itemBeingDragged
					int pieceIdentity = -1;
					if (DragHandler.itemBeingDragged.tag == "Wall") {
						pieceIdentity = Constants.WALL;
					} else if (DragHandler.itemBeingDragged.tag == "Door") {
						pieceIdentity = Constants.DOOR;
					} else if (DragHandler.itemBeingDragged.tag == "Roof") {
						pieceIdentity = Constants.ROOF;
					} else if (DragHandler.itemBeingDragged.tag == "Plant" ) {
						pieceIdentity = Constants.PLANT;
					}

					// determine the type of the old parent, if the parent was a rocket outline, determine which one
					int oldParentType = -1;
					int oldParentOutlineIndex = -1;
					if (DragHandler.itemBeingDragged.transform.parent.parent.gameObject.name.Contains ("Outline")) {
						oldParentType = Constants.PARENT_HOUSE_OUTLINE;
                        if (pieceIdentity == Constants.WALL || pieceIdentity == Constants.DOOR || pieceIdentity == Constants.ROOF || pieceIdentity == Constants.PLANT) {
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
					int newParentOutlineIndex = -1;
					if (pieceIdentity == Constants.WALL     )
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
                        
                    }

                    /* Note: pieces 'snap' to their slots as a result of having a horizontal layout group
				 	/*       on the selected outline piece that they're moving to	*/

                    // If a piece has been dragged successfully, we'll let the game manager know
					OnPieceAddedToHouse (DragHandler.itemBeingDragged, pieceIdentity, oldParentType, oldParentOutlineIndex, newParentOutlineIndex);
				
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

            if (gameObject.tag == "Wall")
            {
                //Logger.Log("Wall");
                selectedOutlineType = Constants.WALL;
            }
            else if (gameObject.tag == "Plant")
            {
				selectedOutlineType = Constants.PLANT;
            }
            else if (gameObject.tag == "Roof")
            {
                selectedOutlineType = Constants.ROOF;
            }
            else if (gameObject.tag == "Door")
            {
                selectedOutlineType = Constants.DOOR;
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