package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.RemoveCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class RemoveCameraKeyAction extends SCSAction
{
   private RemoveCameraKeyCommandExecutor executor;

   public RemoveCameraKeyAction(RemoveCameraKeyCommandExecutor executor)
   {
      super("Remove Camera Key",
              "",
              KeyEvent.VK_R,
              "Removes current camera key",
              "Removes the camera key that is stored at the time indicated on the graph."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.removeCameraKey();
   }
}
