package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface LoadRobotConfigurationDialogConstructor
{
   public abstract void constructLoadRobotConfigurationDialog();
   public abstract void setCurrentDirectory(File directory);
   public abstract void setCurrentDirectory(String directory);
}
