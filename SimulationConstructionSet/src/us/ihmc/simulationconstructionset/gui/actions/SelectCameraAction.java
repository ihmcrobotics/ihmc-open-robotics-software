package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.config.CameraSelector;

public class SelectCameraAction extends SCSAction
{
   private static final long serialVersionUID = -6442315727953394627L;
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

   public void doAction()
   {
      cameraSelector.selectCamera(name);
   }
}
