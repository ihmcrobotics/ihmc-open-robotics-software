package us.ihmc.exampleSimulations.beetle.parameters;

import java.io.InputStream;
import java.nio.file.Path;
import java.nio.file.Paths;

import us.ihmc.SdfLoader.SDFParameters;

public class RhinoBeetleSDFParameters implements SDFParameters
{
   public static final String SDF_MODEL_NAME = "rhinoBeetle";

   private static final String RESOURCE_DIRECTORY = "models/";
   private static final String[] RESOURCE_DIRECTORIES = new String[] {RESOURCE_DIRECTORY};

   // Model paths are represented as Strings because resource paths should always be separated with "/" -- this is not platform dependent
   private static final String SDF_MODEL_PATH = "models/rhino_beetle_description/model/sdf/model.sdf";

   @Override
   public String getSdfFilePath()
   {
      return SDF_MODEL_PATH;
   }

   @Override
   public Path getSdfModelPath()
   {
      return Paths.get(SDF_MODEL_PATH);
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
      InputStream is = getClass().getClassLoader().getResourceAsStream(SDF_MODEL_PATH);

      if (is == null)
      {
         throw new RuntimeException("Unable to open robot model file: " + SDF_MODEL_PATH);
      }

      return is;
   }
}
