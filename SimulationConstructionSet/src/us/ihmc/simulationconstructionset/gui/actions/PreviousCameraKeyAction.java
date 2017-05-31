package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.PreviousCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class PreviousCameraKeyAction extends SCSAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private PreviousCameraKeyCommandExecutor executor;

   public PreviousCameraKeyAction(PreviousCameraKeyCommandExecutor executor)
   {
      super("Previous Camera Key",
              "",
              KeyEvent.VK_R,
              "Previous camera key",
              "Proceeds to the previous chornological CameraKey."
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.previousCameraKey();
   }
}
