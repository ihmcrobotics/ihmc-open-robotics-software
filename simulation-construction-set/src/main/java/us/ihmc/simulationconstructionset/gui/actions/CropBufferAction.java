package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.*;

import us.ihmc.simulationconstructionset.commands.CropBufferCommandExecutor;
import us.ihmc.simulationconstructionset.gui.actions.dialogActions.AbstractActionTools;

public class CropBufferAction extends AbstractAction
{
   private static final long serialVersionUID = 5039888169851613916L;
   
   private CropBufferCommandExecutor executor;

   public CropBufferAction(CropBufferCommandExecutor executor)
   {
      super("Crop Buffer to In/Out");
      this.executor = executor;
      
      String iconFilename = "icons/CropBuffer.png";
      int shortKey = KeyEvent.VK_C;
      String longDescription = "Long Description";
      String shortDescription = "Short Description";

      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.cropBuffer();
   }
}
