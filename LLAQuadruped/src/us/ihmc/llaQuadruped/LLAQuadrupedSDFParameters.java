package us.ihmc.llaQuadruped;

import java.io.InputStream;

import us.ihmc.SdfLoader.SDFParameters;

public class LLAQuadrupedSDFParameters implements SDFParameters
{
   private static final String SDF_MODEL_NAME = "llaQuadruped";
   private static final String RESOURCE_DIRECTORY = "models/";
   private static final String[] RESOURCE_DIRECTORIES = new String[] {RESOURCE_DIRECTORY};
   private static final String SDF_FILE = RESOURCE_DIRECTORY + "lla_quadruped_description/model/sdf/" + SDF_MODEL_NAME + ".sdf";
   
   @Override
   public String getSdfFilePath()
   {
      return SDF_FILE;
   }

   @Override
   public String getSdfModelName()
   {
      return SDF_MODEL_NAME;
   }

   @Override
   public String[] getResourceDirectories()
   {
      return RESOURCE_DIRECTORIES;
   }

   @Override
   public String getResourceDirectory()
   {
      return RESOURCE_DIRECTORY;
   }

   @Override
   public InputStream getSdfAsInputStream()
   {
      return getClass().getClassLoader().getResourceAsStream(getSdfFilePath());
   }
}
