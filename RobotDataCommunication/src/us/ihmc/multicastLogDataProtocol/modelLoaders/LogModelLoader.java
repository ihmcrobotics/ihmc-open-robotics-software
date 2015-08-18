package us.ihmc.multicastLogDataProtocol.modelLoaders;

import us.ihmc.SdfLoader.SDFBaseRobot;

public interface LogModelLoader
{
   public void load(String modelName, byte[] model, String[] resourceDirectories, byte[] resourceZip);
   public SDFBaseRobot createRobot();
   
   public String getModelName();
   public byte[] getModel();
   public String[] getResourceDirectories();
   public byte[] getResourceZip();
}
