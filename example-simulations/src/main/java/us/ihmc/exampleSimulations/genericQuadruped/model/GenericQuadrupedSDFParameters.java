package us.ihmc.exampleSimulations.genericQuadruped.model;

import us.ihmc.modelFileLoaders.SdfLoader.SDFParameters;
import us.ihmc.tools.io.resources.ResourceTools;

import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;

public class GenericQuadrupedSDFParameters implements SDFParameters
{
   private static final String SDF_MODEL_NAME = "genericQuadruped";
   private static final String RESOURCE_DIRECTORY = "models/";
   private static final String[] RESOURCE_DIRECTORIES = new String[] {RESOURCE_DIRECTORY};
   private static final Path SDF_MODEL_PATH = Paths.get("models", "genericQuadruped", "sdf", SDF_MODEL_NAME + ".sdf");
   
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
