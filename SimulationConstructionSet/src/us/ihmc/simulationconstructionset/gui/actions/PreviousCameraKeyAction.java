package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.PreviousCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class PreviousCameraKeyAction extends SCSAction
{
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

   @Override
   public void doAction()
   {
      executor.previousCameraKey();
   }
}
