package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class StopAction extends SCSAction
{
   private static final long serialVersionUID = 4635663288191284465L;

   private StopCommandExecutor executor;

   public StopAction(StopCommandExecutor executor)
   {
      super("Stop",
              "icons/Stop24.gif",
              KeyEvent.VK_T,
              "Stop",
              "Stop playing simulation."
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.stop();
   }
}
