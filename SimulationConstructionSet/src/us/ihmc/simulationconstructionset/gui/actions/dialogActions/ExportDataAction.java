package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportDataDialogConstructor;
import java.awt.event.KeyEvent;
import java.io.File;

@SuppressWarnings("serial")
public class ExportDataAction extends SCSAction
{
   private ExportDataDialogConstructor constructor;

   public ExportDataAction(ExportDataDialogConstructor constructor)
   {
      super("Export Data...",
              "icons/ExportData.png",
              KeyEvent.VK_E,
              "Export Data",
              "Export simulation data to a file."
      );

      this.constructor = constructor;
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

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

}
