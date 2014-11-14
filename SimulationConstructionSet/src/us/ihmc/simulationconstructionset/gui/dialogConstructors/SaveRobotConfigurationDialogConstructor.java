package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;

public interface SaveRobotConfigurationDialogConstructor
{
   public abstract void constructRobotConfigurationDialog();
   public abstract void setCurrentDirectory(String directory);
   public abstract void setCurrentDirectory(File directory);
   
}
