package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ImportDataDialogConstructor;

public class ImportDataAction extends SCSAction
{
   private static final long serialVersionUID = 7296204170538611841L;

   private ImportDataDialogConstructor constructor;

   public ImportDataAction(ImportDataDialogConstructor constructor)
   {
      super("Import Data...",
              "icons/Import24.gif",
              KeyEvent.VK_I,
              "Import Data",
              "Import simulation data to a file."
      );

      this.constructor = constructor;
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void doAction()
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
