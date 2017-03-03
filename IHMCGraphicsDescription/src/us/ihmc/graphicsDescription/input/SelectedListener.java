package us.ihmc.graphicsDescription.input;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
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
   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3DReadOnly location, Point3DReadOnly cameraLocation, QuaternionReadOnly cameraRotation);
}
