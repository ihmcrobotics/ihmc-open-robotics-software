package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.File;

import javax.swing.AbstractAction;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportSnapshotDialogConstructor;

public class ExportSnapshotAction extends AbstractAction
{
   private static final long serialVersionUID = 958525206323850018L;

   private final ExportSnapshotDialogConstructor constructor;
   
   public ExportSnapshotAction(ExportSnapshotDialogConstructor constructor)
   {
      super("Export Snapshot...");
      this.constructor = constructor;

      String iconFilename = "icons/YoExportSnapshot.gif";
      int shortKey = KeyEvent.VK_S;
      String longDescription = "Export snapshot to a file.";
      String shortDescription = "Export Snapshot";
      
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
   public void actionPerformed(ActionEvent actionEvent)
   {
     constructor.constructDialog();
   }
}
