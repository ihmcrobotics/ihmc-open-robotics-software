package us.ihmc.multicastLogDataProtocol.modelLoaders;

public interface LogModelProvider
{
   public Class<? extends LogModelLoader> getLoader();
   public byte[] getModelPackage();
   
}
