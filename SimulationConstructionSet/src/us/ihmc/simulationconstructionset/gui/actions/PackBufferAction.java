package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.PackBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class PackBufferAction extends SCSAction
{
   private static final long serialVersionUID = 3740016405361627977L;
   private PackBufferCommandExecutor executor;

   public PackBufferAction(PackBufferCommandExecutor executor)
   {
      super("Pack Buffer to In/Out",
              "",
              KeyEvent.VK_C,
              "Short Description",
              "Long Description"
      );

      this.executor = executor;
   }

   public void doAction()
   {
      executor.packBuffer();
   }
}
