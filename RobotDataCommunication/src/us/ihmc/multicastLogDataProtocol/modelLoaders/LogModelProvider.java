package us.ihmc.multicastLogDataProtocol.modelLoaders;

public interface LogModelProvider
{
   public Class<? extends LogModelLoader> getLoader();
   
   public String getModelName();
   public String[] getResourceDirectories();
   public byte[] getResourceZip();
   public byte[] getModel();
   
}
