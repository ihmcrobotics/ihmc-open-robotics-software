package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.commands.SimulateCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class SimulateAction extends SCSAction
{
   private static final long serialVersionUID = 2609814986889034283L;

   private final SimulateCommandExecutor executor;

   public SimulateAction(SimulateCommandExecutor listener)
   {
      super("Simulate",
              "icons/YoSimulate32.gif",
              KeyEvent.VK_S,
              "Simulate",
              "Start simulating"
      );

      this.executor = listener;
   }

   public void doAction()
   {
      executor.simulate();
   }
}
