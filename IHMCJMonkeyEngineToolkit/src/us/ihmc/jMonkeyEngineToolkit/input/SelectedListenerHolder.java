package us.ihmc.jMonkeyEngineToolkit.input;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class SelectedListenerHolder
{
   private final ArrayList<SelectedListener> selectedListeners = new ArrayList<SelectedListener>();
   
   public void addSelectedListener(SelectedListener listener)
   {
      selectedListeners.add(listener);
   }

   public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3DReadOnly location, Point3DReadOnly cameraLocation, QuaternionReadOnly cameraRotation)
   {
      for(SelectedListener selectedListener : selectedListeners)
      {
         selectedListener.selected(graphics3dNode, modifierKeyInterface, location, cameraLocation, cameraRotation);
      }
   }
}
