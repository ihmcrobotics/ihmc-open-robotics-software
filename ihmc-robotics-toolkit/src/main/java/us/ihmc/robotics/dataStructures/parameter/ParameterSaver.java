package us.ihmc.robotics.dataStructures.parameter;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

public class ParameterSaver
{
   private final Path path;

   public ParameterSaver(Path parametersFile)
   {
      this(null, parametersFile);
   }

   public ParameterSaver(String filename, Path parametersDirectory)
   {
      // If an absolute path is specified, use it as-is. If not, try to locate the robot-specific resources directory.
      if (!parametersDirectory.isAbsolute())
      {
         Path pathFromCwd = Paths.get("").toAbsolutePath().getParent().resolve(parametersDirectory);

         // If $IHMC_WORKSPACE is defined, use it.
         if (System.getenv("IHMC_WORKSPACE") != null)
         {
            Path workspacePath = Paths.get(System.getenv("IHMC_WORKSPACE"));
            parametersDirectory = workspacePath.resolve(parametersDirectory).resolve(filename);
         }
         // If ../<defaultParametersPath> exists, use that.
         else if (Files.exists(pathFromCwd))
         {
            parametersDirectory = pathFromCwd.resolve(filename);
         }
         else
         {
            System.err.println("Could not find parameters file.");
            System.err.println("If a relative path is supplied then either $IHMC_WORKSPACE must be defined or cwd must be your _IHMCWorkspace");
            System.exit(1);
         }
      }

      this.path = parametersDirectory;

      System.out.println("Set path variable to " + this.path.toString());
   }

   public void writeParameters(List<Parameter> parameters)
   {
      System.out.println("Writing parameters file");

      File file = path.toFile();
      try (PrintWriter writer = new PrintWriter(new FileOutputStream(file)))
      {
         for (Parameter parameter : parameters)
         {
            writer.print(parameter.dump() + "\n");
         }
      }
      catch (IOException e)
      {
         System.err.println("Failed to write parameter list: " + e.getMessage());
         e.printStackTrace();
      }
   }
}
