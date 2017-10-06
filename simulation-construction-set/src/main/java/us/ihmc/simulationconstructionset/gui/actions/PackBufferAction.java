package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.PackBufferCommandExecutor;

public class PackBufferAction extends AbstractAction
{
   private static final long serialVersionUID = 3740016405361627977L;
   private PackBufferCommandExecutor executor;

   public PackBufferAction(PackBufferCommandExecutor executor)
   {
      super("Pack Buffer to In/Out");
      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_C));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      executor.packBuffer();
   }
}
