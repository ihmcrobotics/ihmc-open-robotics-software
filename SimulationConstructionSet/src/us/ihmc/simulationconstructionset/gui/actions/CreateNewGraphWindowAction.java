package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.CreateNewGraphWindowCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

import java.awt.event.KeyEvent;

public class CreateNewGraphWindowAction extends SCSAction
{
   private CreateNewGraphWindowCommandExecutor executor;

   public CreateNewGraphWindowAction(CreateNewGraphWindowCommandExecutor executor)
   {
      super("New Graph Window",
              "",
              KeyEvent.VK_G,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.createNewGraphWindow();
   }
}
