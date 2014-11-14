package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface SaveGraphConfigurationDialogConstructor
{
   public abstract void constructSaveGraphConfigurationDialog();
   public abstract void setCurrentDirectory(String directory);
   public abstract void setCurrentDirectory(File directory);
}
