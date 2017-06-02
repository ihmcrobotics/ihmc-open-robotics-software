package us.ihmc.simulationconstructionset.gui.actions;

import us.ihmc.simulationconstructionset.commands.CropBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;
import java.awt.event.KeyEvent;

@SuppressWarnings("serial")
public class CropBufferAction extends SCSAction
{
   private CropBufferCommandExecutor executor;

   public CropBufferAction(CropBufferCommandExecutor executor)
   {
      super("Crop Buffer to In/Out",
              "",
              KeyEvent.VK_C,
              "Short Description",
              "Long Description"
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.cropBuffer();
   }
}
