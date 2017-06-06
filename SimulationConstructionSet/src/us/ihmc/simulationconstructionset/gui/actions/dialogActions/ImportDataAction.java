package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ImportDataDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class ImportDataAction extends SCSAction
{
   private ImportDataDialogConstructor constructor;

   public ImportDataAction(ImportDataDialogConstructor constructor)
   {
      super("Import Data...",
              "icons/ImportData.png",
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

   @Override
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
