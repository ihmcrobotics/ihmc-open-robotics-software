package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportDataDialogConstructor;

public class ExportDataAction extends AbstractAction
{
   private static final long serialVersionUID = -5481556236530603500L;

   private ExportDataDialogConstructor constructor;

   public ExportDataAction(ExportDataDialogConstructor constructor)
   {
      super("Export Data...");
      this.constructor = constructor;

      String iconFilename = "icons/Export24.gif";
      int shortKey = KeyEvent.VK_E;
      String longDescription = "Export simulation data to a file.";
      String shortDescription = "Export Data";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      constructor.constructDialog();
   }

   public void closeAndDispose()
   {
      if (constructor != null)
      {
         constructor.closeAndDispose();
         constructor = null;
      }
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

}
