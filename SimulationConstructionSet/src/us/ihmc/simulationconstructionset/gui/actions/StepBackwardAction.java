package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.StepBackwardCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class StepBackwardAction extends SCSAction
{
   private static final long serialVersionUID = 8975600674258754815L;

   private StepBackwardCommandExecutor executor;

   public StepBackwardAction(StepBackwardCommandExecutor executor)
   {
      super("Step Backward",
              "icons/StepBack24.gif",
              KeyEvent.VK_B,
              "Step Backward",
              "Step Backward"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.stepBackward();
   }
}
