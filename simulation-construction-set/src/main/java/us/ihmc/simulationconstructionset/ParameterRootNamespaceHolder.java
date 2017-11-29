package us.ihmc.simulationconstructionset;

import java.io.File;

import us.ihmc.yoVariables.registry.NameSpace;

public interface ParameterRootNamespaceHolder
{
   public NameSpace getParameterRootPath();
   public File getDefaultParameterFile();
}
