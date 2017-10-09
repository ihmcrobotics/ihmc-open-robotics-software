package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.StepBackwardCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class StepBackwardAction extends AbstractAction
{
   private static final long serialVersionUID = 8975600674258754815L;

   private StepBackwardCommandExecutor executor;

   public StepBackwardAction(StepBackwardCommandExecutor executor)
   {
      super("Step Backward");
      this.executor = executor;

      String iconFilename = "icons/StepBack24.gif";
      int shortKey = KeyEvent.VK_B;
      String longDescription = "Step Backward";
      String shortDescription = "Step Backward";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.stepBackward();
   }
}
