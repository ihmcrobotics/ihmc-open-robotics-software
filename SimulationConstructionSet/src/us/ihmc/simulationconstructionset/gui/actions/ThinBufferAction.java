package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ThinBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class ThinBufferAction extends SCSAction
{
   private static final long serialVersionUID = 5502411154340525798L;
   private ThinBufferCommandExecutor executor;

   public ThinBufferAction(ThinBufferCommandExecutor executor)
   {
      super("Thin Buffer, Removing Every Other Tick",
              "",
              KeyEvent.VK_T,
              "Short Description",
              "Long Description"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.thinBuffer(2);
   }
}
