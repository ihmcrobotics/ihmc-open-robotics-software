package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CreateNewGraphWindowCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class CreateNewGraphWindowAction extends SCSAction
{
   private static final long serialVersionUID = 5063019587274426802L;
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

   public void doAction()
   {
      executor.createNewGraphWindow();
   }
}
