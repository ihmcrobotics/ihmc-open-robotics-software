package us.ihmc.avatar.ros.networkTest;

public class ROS2NetworkTestMachine
{
   private final String hostname;
   private final String username;
   private final String deployDirectory;

   public ROS2NetworkTestMachine(String hostname, String username, String deployDirectory)
   {
      this.hostname = hostname;
      this.username = username;
      this.deployDirectory = deployDirectory;
   }

   public String getHostname()
   {
      return hostname;
   }

   public String getUsername()
   {
      return username;
   }

   public String getDeployDirectory()
   {
      return deployDirectory;
   }
}
