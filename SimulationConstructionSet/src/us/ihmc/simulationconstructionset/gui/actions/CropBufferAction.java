package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CropBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.SCSAction;

public class CropBufferAction extends SCSAction
{
   private static final long serialVersionUID = 5039888169851613916L;
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

   public void doAction()
   {
      executor.cropBuffer();
   }
}
