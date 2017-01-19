package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.StopCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class StopAction extends AbstractAction
{
   private static final long serialVersionUID = 4635663288191284465L;

   private StopCommandExecutor executor;

   public StopAction(StopCommandExecutor executor)
   {
      super("Stop");
      this.executor = executor;

      String iconFilename = "icons/Stop24.gif";
      int shortKey = KeyEvent.VK_T;
      String longDescription = "Stop playing simulation.";
      String shortDescription = "Stop";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      executor.stop();
   }
}
