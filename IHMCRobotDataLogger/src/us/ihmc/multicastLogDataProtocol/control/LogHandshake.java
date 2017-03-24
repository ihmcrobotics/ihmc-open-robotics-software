package us.ihmc.multicastLogDataProtocol.control;

import us.ihmc.robotDataLogger.Handshake;

public class LogHandshake
{

   private Handshake handshake;

   private String modelLoaderClass = null;
   private String modelName;
   private byte[] model;
   private String[] resourceDirectories;
   private byte[] resourceZip;

   public Handshake getHandshake()
   {
      return handshake;
   }

   public void setHandshake(Handshake handshake)
   {
      this.handshake = handshake;
   }

   public String getModelLoaderClass()
   {
      return modelLoaderClass;
   }

   public void setModelLoaderClass(String modelLoaderClass)
   {
      this.modelLoaderClass = modelLoaderClass;
   }

   public String getModelName()
   {
      return modelName;
   }

   public void setModelName(String modelName)
   {
      this.modelName = modelName;
   }

   public byte[] getModel()
   {
      return model;
   }

   public void setModel(byte[] model)
   {
      this.model = model;
   }

   public String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   public void setResourceDirectories(String[] resourceDirectories)
   {
      this.resourceDirectories = resourceDirectories;
   }

   public byte[] getResourceZip()
   {
      return resourceZip;
   }

   public void setResourceZip(byte[] resourceZip)
   {
      this.resourceZip = resourceZip;
   }

}
