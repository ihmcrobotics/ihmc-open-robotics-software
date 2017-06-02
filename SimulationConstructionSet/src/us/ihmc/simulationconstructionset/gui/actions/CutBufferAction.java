package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.CutBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class CutBufferAction extends SCSAction
{
   private CutBufferCommandExecutor executor;

   public CutBufferAction(CutBufferCommandExecutor executor)
   {
      super("Cut Buffer Between In/Out",
              "",
              KeyEvent.VK_C,
              "Short Description", // TODO
              "Long Description" // TODO
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.cutBuffer();
   }
}
