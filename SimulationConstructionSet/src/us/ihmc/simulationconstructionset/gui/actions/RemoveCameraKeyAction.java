package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.RemoveCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class RemoveCameraKeyAction extends SCSAction
{
   private static final long serialVersionUID = -5162293334622550111L;
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

   public void doAction()
   {
      executor.removeCameraKey();
   }
}
