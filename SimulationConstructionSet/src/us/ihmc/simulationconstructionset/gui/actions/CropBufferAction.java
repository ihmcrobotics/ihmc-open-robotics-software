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
              "icons/CropBuffer.png",
              KeyEvent.VK_C,
              "Crop Buffer to In/Out",
              "Crop the data buffer to between the current In and Out points."
      );

      this.executor = executor;
   }

   @Override
   public void doAction()
   {
      executor.cropBuffer();
   }
}
