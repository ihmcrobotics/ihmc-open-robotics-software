package us.ihmc.idl;

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

      RosInterfaceGenerator generator = new RosInterfaceGenerator();

//      generator.addPackageRootToIDLGenerator(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/rcl_interfaces"));
//      generator.addPackageRootToIDLGenerator(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/common_interfaces"));
//      generator.addPackageRootToIDLGenerator(Paths.get("src/main/rosidl/ihmc_interfaces"));
//
//      generator.addCustomIDLFiles(Paths.get("build/tmp/generateMessages/ros2-common-interfaces"));
//
//      generator.generate(Paths.get("build/tmp/idl"), Paths.get("build/tmp/generateMessages/java"));

      // Temp stuff for Sylvain
      generator.addPackageRootToIDLGenerator(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/rcl_interfaces"));
      generator.addPackageRootToIDLGenerator(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/common_interfaces"));
      generator.addPackageRootToIDLGenerator(Paths.get("src/main/messages/ihmc_interfaces"));
      generator.addPackageRootToROS1Generator(Paths.get("src/main/messages/ihmc_interfaces"));

      generator.addCustomIDLFiles(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/"));

      generator.generate(Paths.get("src/main/generated-idl"), Paths.get("src/main/messages/ros1"), Paths.get("src/main/generated-java"));

      deleteDuplicateFiles("src/main/generated-idl");
      deleteDuplicateFiles("src/main/generated-java");

      RosInterfaceGenerator.convertDirectoryToUnixEOL(Paths.get("src/main/generated-idl"));
      RosInterfaceGenerator.convertDirectoryToUnixEOL(Paths.get("src/main/generated-java"));
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
