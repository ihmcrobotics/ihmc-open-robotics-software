package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.StepForwardCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class StepForwardAction extends AbstractAction
{
   private static final long serialVersionUID = -4007043816767478116L;

   private StepForwardCommandExecutor executor;

   public StepForwardAction(StepForwardCommandExecutor executor)
   {
      super("Step Forward");
      this.executor = executor;

      String iconFilename = "icons/StepForward24.gif";
      int shortKey = KeyEvent.VK_F;
      String longDescription = "Step Forward";
      String shortDescription = "Step Forward";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.stepForward();
   }
}
