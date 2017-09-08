package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CutBufferCommandExecutor;

public class CutBufferAction extends AbstractAction
{
   private static final long serialVersionUID = 5039888169851613916L;
   private CutBufferCommandExecutor executor;

   public CutBufferAction(CutBufferCommandExecutor executor)
   {
      super("Cut Buffer Between In/Out");
      this.executor = executor;

      this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_C));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.cutBuffer();
   }
}
