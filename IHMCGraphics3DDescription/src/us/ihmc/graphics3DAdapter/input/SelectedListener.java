package us.ihmc.graphics3DAdapter.input;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public interface SelectedListener
{
   /**
    * This function is called when a point on the screen is selected while holding the modifier key defined with getModifierKey()
    * 
    * @param graphics3dNode   The node that was selected, null if none selected
    * @param modifierKeys TODO
    * @param location location of the selected point
    * @param cameraLocation camera position
    * @param cameraRotation camera view direction
    */
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3d location, Point3d cameraLocation, Quat4d cameraRotation);
}
