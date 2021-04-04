package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.tools.DockerCommand;

public class DockerMapSense extends DockerCommand
{
   public DockerMapSense()
   {
      super("ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources/us/ihmc/runMapSenseDocker.sh");
   }

   public static void main(String[] args)
   {
      DockerMapSense command = new DockerMapSense();
      command.start();
      command.waitFor();
   }
}
