package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.SCSAction;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSnapshotDialogConstructor;

public class ExportSnapshotAction extends SCSAction
{
   private static final long serialVersionUID = 958525206323850018L;

   private final ExportSnapshotDialogConstructor constructor;
   
   public ExportSnapshotAction(ExportSnapshotDialogConstructor constructor)
   {
      super("Export Snapshot...",
              "icons/YoExportSnapshot.gif",
              KeyEvent.VK_S,
              "Export Snapshot",
              "Export snapshot to a file."
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
}
