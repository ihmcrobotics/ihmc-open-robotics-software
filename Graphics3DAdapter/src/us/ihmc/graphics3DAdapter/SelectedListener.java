package us.ihmc.graphics3DAdapter;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;

public interface SelectedListener
{
   /**
    * This function is called when a point on the screen is selected while holding the modifier key defined with getModifierKey()
    * 
    * @param graphics3dNode   The node that was selected, null if none selected
    * @param modifierKeys TODO
    * @param location location of the selected point
    * @param cameraLocation camera position
    * @param lookAtDirection camera view direction
    */
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyHolder modifierKeyHolder, Point3d location, Point3d cameraLocation, Vector3d lookAtDirection);
}
