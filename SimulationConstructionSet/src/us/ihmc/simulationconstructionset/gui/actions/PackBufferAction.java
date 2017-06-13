package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.PackBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class PackBufferAction extends SCSAction
{
   private PackBufferCommandExecutor executor;

   public PackBufferAction(PackBufferCommandExecutor executor)
   {
      super("Pack Buffer to In/Out",
              "",
              KeyEvent.VK_C,
              "?",
              "?"
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.packBuffer();
   }
}
