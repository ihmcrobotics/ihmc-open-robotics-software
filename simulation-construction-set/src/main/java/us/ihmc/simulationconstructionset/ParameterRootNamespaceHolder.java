package us.ihmc.simulationconstructionset;

import java.net.URL;

import us.ihmc.yoVariables.registry.NameSpace;

public interface ParameterRootNamespaceHolder
{
   public NameSpace getParameterRootPath();
   public URL getDefaultParameterFile();
}
