package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SimulateCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class SimulateAction extends AbstractAction
{
   private static final long serialVersionUID = 2609814986889034283L;

   private final SimulateCommandExecutor executor;

   public SimulateAction(SimulateCommandExecutor listener)
   {
      super("Simulate");
      this.executor = listener;

      String iconFilename = "icons/YoSimulate32.gif";
      int shortKey = KeyEvent.VK_S;
      String longDescription = "Start simulating.";
      String shortDescription = "Simulate";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.simulate();
   }
}
