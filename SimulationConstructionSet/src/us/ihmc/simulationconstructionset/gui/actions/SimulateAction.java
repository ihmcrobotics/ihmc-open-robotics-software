package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.KeyEvent;
import us.ihmc.simulationconstructionset.commands.SimulateCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

@SuppressWarnings("serial")
public class SimulateAction extends SCSAction
{
   private final SimulateCommandExecutor executor;

   public SimulateAction(SimulateCommandExecutor listener)
   {
      super("Simulate",
              "icons/Simulate6.png",
              KeyEvent.VK_S,
              "Simulate",
              "Start simulating."
      );

      this.executor = listener;
   }

   @Override
   public void doAction()
   {
      executor.simulate();
   }
}
