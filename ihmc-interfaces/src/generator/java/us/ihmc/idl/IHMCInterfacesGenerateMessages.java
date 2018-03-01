package us.ihmc.idl;

import us.ihmc.ros2.rosidl.RosInterfaceGenerator;

import java.io.IOException;
import java.io.InputStream;
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
      InputStream ihmcPubSubTemplate = Thread.currentThread().getContextClassLoader().getResourceAsStream("us/ihmc/idl/msg.idl.em");
      RosInterfaceGenerator generator = new RosInterfaceGenerator(ihmcPubSubTemplate);

      generator.addPackageRoot(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/rcl_interfaces"));
      generator.addPackageRoot(Paths.get("build/tmp/generateMessages/ros2-common-interfaces/common_interfaces"));
      generator.addPackageRoot(Paths.get("src/main/rosidl/ihmc_interfaces"));

      generator.addCustomIDLFiles(Paths.get("build/tmp/generateMessages/ros2-common-interfaces"));

      generator.generate(Paths.get("build/tmp/idl"), Paths.get("build/tmp/generateMessages/java"));
   }
}
