package us.ihmc.aware.params;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class ParameterPersister
{
   private static final Path DEFAULT_PARAMETER_PATH = Paths.get(System.getProperty("user.home"), ".ihmc", "controller_parameters.yaml");

   public static void loadFromFile(ParameterMapRepository repository)
   {
      loadFromFile(repository, DEFAULT_PARAMETER_PATH.toFile());
   }

   public static void loadFromFile(ParameterMapRepository repository, File file)
   {
      if (!file.exists())
      {
         System.err.println("Not loading parameters because " + file.getAbsolutePath() + " does not exist.");
         return;
      }

      try
      {
         repository.load(new FileReader(file));
      }
      catch (FileNotFoundException e)
      {
         // Shouldn't happen because we checked that the file exists.
         e.printStackTrace();
      }
   }

   public static void dumpToFile(ParameterMapRepository repository) throws IOException
   {
      dumpToFile(repository, DEFAULT_PARAMETER_PATH.toFile());
   }

   public static void dumpToFile(ParameterMapRepository repository, File file) throws IOException
   {
      repository.dump(new FileWriter(file));
   }
}
