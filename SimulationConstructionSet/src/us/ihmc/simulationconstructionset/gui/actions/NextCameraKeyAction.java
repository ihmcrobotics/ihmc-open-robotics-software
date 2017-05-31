package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.NextCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class NextCameraKeyAction extends SCSAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private NextCameraKeyCommandExecutor executor;

   public NextCameraKeyAction(NextCameraKeyCommandExecutor executor)
   {
      super("Next Camera Key",
              "",
              KeyEvent.VK_R,
              "Next camera key",
              "Proceeds to the next chronological CameraKey."
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.nextCameraKey();
   }
}
