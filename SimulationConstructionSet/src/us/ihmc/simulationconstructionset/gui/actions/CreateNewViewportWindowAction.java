package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CreateNewViewportWindowCommandExecutor;

public class CreateNewViewportWindowAction extends AbstractAction
{
   private static final long serialVersionUID = 1890952078365483337L;
   private CreateNewViewportWindowCommandExecutor executor;

   public CreateNewViewportWindowAction(CreateNewViewportWindowCommandExecutor executor)
   {
      super("New Viewport Window");
      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_V));
      this.putValue(Action.LONG_DESCRIPTION, "Creates a new Viewport Window for showing 3D Graphics in SCS.");
      this.putValue(Action.SHORT_DESCRIPTION, "Creates a new Viewport Window.");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.createNewViewportWindow();
   }
}
