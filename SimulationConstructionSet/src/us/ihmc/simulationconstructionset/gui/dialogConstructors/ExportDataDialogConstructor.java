package us.ihmc.simulationconstructionset.gui.dialogConstructors;

public interface ExportDataDialogConstructor
{
   public abstract void constructExportDataDialog();
   public abstract void setCurrentDirectory(String directory);      
   public abstract void closeAndDispose();
}
