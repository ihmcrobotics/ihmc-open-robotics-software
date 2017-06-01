package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.NextCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class NextCameraKeyAction extends SCSAction
{
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

   @Override
   public void doAction()
   {
      executor.nextCameraKey();
   }
}
