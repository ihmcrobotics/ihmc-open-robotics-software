package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.AddKeyPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

import java.awt.event.KeyEvent;

public class AddKeyPointAction extends SCSAction
{
   private AddKeyPointCommandExecutor executor;

   public AddKeyPointAction(AddKeyPointCommandExecutor executor)
   {
      super("Add Key Point",
              "icons/setKey.gif",
              KeyEvent.VK_F,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.addKeyPoint();
   }
}
