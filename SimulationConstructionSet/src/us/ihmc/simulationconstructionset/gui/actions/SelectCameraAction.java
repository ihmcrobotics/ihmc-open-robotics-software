package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.CameraSelector;

import java.awt.event.KeyEvent;

public class SelectCameraAction extends SCSAction
{
   private String name;
   private CameraSelector cameraSelector;

   public SelectCameraAction(CameraSelector selector, String name)
   {
      super(name,
              "",
              KeyEvent.VK_E,
              "Short Description",
              "Long Description"
      );

      this.cameraSelector = selector;
      this.name = name;
   }

   @Override
   public void doAction()
   {
      cameraSelector.selectCamera(name);
   }
}
