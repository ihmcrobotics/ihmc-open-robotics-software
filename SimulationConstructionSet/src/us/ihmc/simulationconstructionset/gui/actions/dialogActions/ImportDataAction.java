package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.ImportDataDialogConstructor;

public class ImportDataAction extends AbstractAction
{
   private static final long serialVersionUID = 7296204170538611841L;

   private ImportDataDialogConstructor constructor;

   public ImportDataAction(ImportDataDialogConstructor constructor)
   {
      super("Import Data...");
      this.constructor = constructor;

      String iconFilename = "icons/Import24.gif";
      int shortKey = KeyEvent.VK_I;
      String longDescription = "Import simulation data to a file.";
      String shortDescription = "Import Data";
      
      AbstractActionTools.setupIconButton(this, iconFilename, shortKey, longDescription, shortDescription);
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
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
}
