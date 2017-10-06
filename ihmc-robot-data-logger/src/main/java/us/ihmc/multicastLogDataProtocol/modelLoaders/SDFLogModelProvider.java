package us.ihmc.multicastLogDataProtocol.modelLoaders;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.regex.Pattern;

import org.apache.commons.io.IOUtils;

import us.ihmc.modelFileLoaders.SdfLoader.SDFParameters;
import us.ihmc.tools.ClassLoaderTools;

public class SDFLogModelProvider implements LogModelProvider
{
   private final String sdfModelName;
   private final byte[] model;
   private final String[] resourceDirectories;
   
   public SDFLogModelProvider(SDFParameters sdfParameters)
   {
      this(sdfParameters.getSdfModelName(), sdfParameters.getSdfAsInputStream(), sdfParameters.getResourceDirectories());
   }

   public SDFLogModelProvider(String modelName, InputStream sdfFileAsStream, String[] resourceDirectories)
   {
      this.sdfModelName = modelName;
      
      try
      {
         this.model = IOUtils.toByteArray(sdfFileAsStream);
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      }
      
      this.resourceDirectories = new String[resourceDirectories.length];
      System.arraycopy(resourceDirectories, 0, this.resourceDirectories, 0, resourceDirectories.length);
   }

   @Override
   public Class<? extends LogModelLoader> getLoader()
   {
      return SDFModelLoader.class;
   }

   @Override
   public String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   @Override
   public byte[] getResourceZip()
   {
      ByteArrayOutputStream os = new ByteArrayOutputStream();
      try
      {
         Pattern zipExclude = null; //Pattern.compile(".*\\.(?i)(zip)$");
         ClassLoaderTools.createZipBundle(os, zipExclude, resourceDirectories);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      return os.toByteArray();
   }

   @Override
   public byte[] getModel()
   {
      return model;
   }

   @Override
   public String getModelName()
   {
      return sdfModelName;
   }
}
