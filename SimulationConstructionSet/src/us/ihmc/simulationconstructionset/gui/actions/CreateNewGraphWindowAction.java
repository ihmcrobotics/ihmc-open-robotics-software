package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CreateNewGraphWindowCommandExecutor;

public class CreateNewGraphWindowAction extends AbstractAction
{
   private static final long serialVersionUID = 5063019587274426802L;
   private CreateNewGraphWindowCommandExecutor executor;

   public CreateNewGraphWindowAction(CreateNewGraphWindowCommandExecutor executor)
   {
      super("New Graph Window");
      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_G));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.createNewGraphWindow();
   }
}
