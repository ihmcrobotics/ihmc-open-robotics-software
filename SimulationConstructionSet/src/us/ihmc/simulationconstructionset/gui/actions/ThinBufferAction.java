package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ThinBufferCommandExecutor;

public class ThinBufferAction extends AbstractAction
{
   private static final long serialVersionUID = 5502411154340525798L;
   private ThinBufferCommandExecutor executor;

   public ThinBufferAction(ThinBufferCommandExecutor executor)
   {
      super("Thin Buffer, Removing Every Other Tick");
      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_T));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.thinBuffer(2);
   }
}
