package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.StepForwardCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class StepForwardAction extends SCSAction
{
   private StepForwardCommandExecutor executor;

   public StepForwardAction(StepForwardCommandExecutor executor)
   {
      super("Step Forward",
              "icons/StepForward.png",
              KeyEvent.VK_F,
      "Step Forward",
      "Step forward one frame in the data buffer."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.stepForward();
   }
}
