package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class StopAction extends SCSAction
{
   private StopCommandExecutor executor;

   public StopAction(StopCommandExecutor executor)
   {
      super("Stop",
              "icons/Stop.png",
              KeyEvent.VK_T,
              "Stop",
              "Stop playing simulation."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.stop();
   }
}
