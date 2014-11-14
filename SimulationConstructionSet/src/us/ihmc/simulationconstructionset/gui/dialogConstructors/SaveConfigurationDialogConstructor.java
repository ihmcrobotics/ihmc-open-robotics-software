package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface SaveConfigurationDialogConstructor
{
   public abstract void constructSaveConfigurationDialog();
   public abstract void setCurrentDirectory(String directory);
   public abstract void setCurrentDirectory(File directory);
}
