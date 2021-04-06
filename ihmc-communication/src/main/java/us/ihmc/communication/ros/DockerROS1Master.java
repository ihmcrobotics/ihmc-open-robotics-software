package us.ihmc.communication.ros;

import us.ihmc.tools.DockerCommand;

public class DockerROS1Master extends DockerCommand
{
   public DockerROS1Master()
   {
      super("ihmc-open-robotics-software", "ihmc-communication/src/main/resources/us/ihmc/communication/ros/runRosCoreDocker.sh");
   }

   public static void main(String[] args)
   {
      DockerROS1Master command = new DockerROS1Master();
      command.start();
      command.waitFor();
   }
}
