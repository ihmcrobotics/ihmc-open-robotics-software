package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.StepBackwardCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class StepBackwardAction extends SCSAction
{
   private StepBackwardCommandExecutor executor;

   public StepBackwardAction(StepBackwardCommandExecutor executor)
   {
      super("Step Backward",
              "icons/StepBackward.png",
              KeyEvent.VK_B,
              "Step Backward",
              "Step backward one frame in the data buffer."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.stepBackward();
   }
}
