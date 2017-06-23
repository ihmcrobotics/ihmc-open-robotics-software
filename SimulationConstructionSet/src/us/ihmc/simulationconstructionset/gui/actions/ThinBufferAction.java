package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.ThinBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class ThinBufferAction extends SCSAction
{
   private ThinBufferCommandExecutor executor;

   public ThinBufferAction(ThinBufferCommandExecutor executor)
   {
      super("Thin Buffer",
              "",
              KeyEvent.VK_T,
              "Thin Buffer, Removing Every Other Tick",
              "Remove every other tick of data in the buffer to thin it out."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.thinBuffer(2);
   }
}
