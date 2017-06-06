package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.AddKeyPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class AddKeyPointAction extends SCSAction
{
   private AddKeyPointCommandExecutor executor;

   public AddKeyPointAction(AddKeyPointCommandExecutor executor)
   {
      super("Add Key Point",
              "icons/NewKey.png",
              KeyEvent.VK_F,
              "Add Key Point to graphs",
              "Add Key Point to graphs to mark a time of interest in the simulation."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.addKeyPoint();
   }
}
