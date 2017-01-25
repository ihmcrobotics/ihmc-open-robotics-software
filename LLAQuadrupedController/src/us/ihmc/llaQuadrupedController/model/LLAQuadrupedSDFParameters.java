package us.ihmc.llaQuadrupedController.model;

import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;

import us.ihmc.modelFileLoaders.SdfLoader.SDFParameters;
import us.ihmc.tools.io.resources.ResourceTools;

public class LLAQuadrupedSDFParameters implements SDFParameters
{
   private static final String SDF_MODEL_NAME = "llaQuadruped";
   private static final String RESOURCE_DIRECTORY = "models/";
   private static final String[] RESOURCE_DIRECTORIES = new String[] {RESOURCE_DIRECTORY};
   private static final Path SDF_MODEL_PATH = Paths.get("models", "lla_quadruped_description", "model", "sdf", SDF_MODEL_NAME + ".sdf");
   
   @Override
   public Path getSdfModelPath()
   {
      return SDF_MODEL_PATH;
   }

   @Override
   public String getSdfFilePath()
   {
      return SDF_MODEL_PATH.toString();
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
      return ResourceTools.openStreamSystem(SDF_MODEL_PATH);
   }
}
