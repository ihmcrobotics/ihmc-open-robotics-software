package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.CreateNewGraphWindowCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class CreateNewGraphWindowAction extends SCSAction
{
   private CreateNewGraphWindowCommandExecutor executor;

   public CreateNewGraphWindowAction(CreateNewGraphWindowCommandExecutor executor)
   {
      super("New Graph Window",
              "",
              KeyEvent.VK_G,
              "Create New Graph Window",
              "Create a new graph window to hold YoGraphs."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.createNewGraphWindow();
   }
}
