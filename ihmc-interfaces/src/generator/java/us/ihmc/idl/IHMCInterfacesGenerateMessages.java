package us.ihmc.idl;

import org.apache.commons.io.FileUtils;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.ros2.rosidl.RosInterfaceGenerator;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;

public class IHMCInterfacesGenerateMessages
{
   /**
    * Must be run from ihmc-interfaces dir!!!
    *
    * This is actually a replacement for the generateMessages Gradle task for now.
    */
   public static void main(String[] args) throws IOException
   {
      FileTools.deleteQuietly(Paths.get("src/main/generated-idl"));
      FileTools.deleteQuietly(Paths.get("src/main/generated-java"));
      FileTools.deleteQuietly(Paths.get("src/main/messages/ros1/controller_msgs/msg"));
//      FileTools.deleteQuietly(Paths.get("build/tmp/generateMessages")); // this needs to be found and copied via Gradle

      RosInterfaceGenerator generator = new RosInterfaceGenerator();
      generator.addPackageRootToIDLGenerator(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/rcl_interfaces"));
      generator.addPackageRootToIDLGenerator(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/common_interfaces"));
      generator.addPackageRootToIDLGenerator(Paths.get("src/main/messages/ihmc_interfaces"));
      generator.addPackageRootToROS1Generator(Paths.get("src/main/messages/ihmc_interfaces"));

      generator.addCustomIDLFiles(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/"));

      generator.generate(Paths.get("build/tmp/generateMessages/generated-idl"),
                         Paths.get("build/tmp/generateMessages/generated-ros1"),
                         Paths.get("build/tmp/generateMessages/generated-java"));

      FileUtils.copyDirectory(Paths.get("build/tmp/generateMessages/generated-idl/controller_msgs").toFile(),
                              Paths.get("src/main/generated-idl/controller_msgs").toFile());
      FileUtils.copyDirectory(Paths.get("build/tmp/generateMessages/generated-java/controller_msgs").toFile(),
                              Paths.get("src/main/generated-java/controller_msgs").toFile());
      FileUtils.copyDirectory(Paths.get("build/tmp/generateMessages/generated-ros1/controller_msgs").toFile(),
                              Paths.get("src/main/messages/ros1/controller_msgs").toFile());

      RosInterfaceGenerator.convertDirectoryToUnixEOL(Paths.get("src/main/generated-idl"));
      RosInterfaceGenerator.convertDirectoryToUnixEOL(Paths.get("src/main/generated-java"));
      RosInterfaceGenerator.convertDirectoryToUnixEOL(Paths.get("src/main/messages/ros1"));
   }

   private static void deleteDuplicateFiles(String outputDirectory)
   {
      PathTools.walkFlat(Paths.get(outputDirectory), new BasicPathVisitor()
      {
         @Override
         public FileVisitResult visitPath(Path path, PathType pathType)
         {
            if (!path.getFileName().toString().equals("controller_msgs"))
            {
               FileTools.deleteQuietly(path);
            }

            return FileVisitResult.CONTINUE;
         }
      });
   }
}
