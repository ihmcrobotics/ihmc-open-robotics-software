package us.ihmc.avatar.ros.networkTest;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.net.InetAddress;

public enum ROS2NetworkTestMachine
{
   OCU("atlas-ocu", "atlas-ocu", "172.16.66.62", "robotlab", "unused"),
   CPU0("cpu0", "CPU0", "172.16.66.100", "shadylady", "/usr/local/bin/mission_control/bin"),
   CPU1("cpu1", "cpu1", "172.16.66.101", "shadylady", "/usr/local/bin/mission_control/bin"),
   CPU4("cpu4", "cpu4", "172.16.66.104", "shadylady", "/usr/local/bin/mission_control/bin"),
   ;

   public static final String LOCAL_HOSTNAME
         = ExceptionTools.handle(() -> InetAddress.getLocalHost().getHostName(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

   public static ROS2NetworkTestMachine getLocalMachine()
   {
      for (ROS2NetworkTestMachine machine : values())
      {
         if (machine.getLocalHostname().equals(LOCAL_HOSTNAME))
         {
            return machine;
         }
      }

      throw new RuntimeException("Must be run from registered machine. Detected hostname: " + LOCAL_HOSTNAME);
   }

   private final String sshHostname;
   private final String localHostname;
   private final String ipAddress;
   private final String username;
   private final String deployDirectory;

   ROS2NetworkTestMachine(String sshHostname, String localHostname, String ipAddress, String username, String deployDirectory)
   {
      this.sshHostname = sshHostname;
      this.localHostname = localHostname;
      this.ipAddress = ipAddress;
      this.username = username;
      this.deployDirectory = deployDirectory;
   }

   public String getSSHHostname()
   {
      return sshHostname;
   }

   public String getUsername()
   {
      return username;
   }

   public String getDeployDirectory()
   {
      return deployDirectory;
   }

   public String getLocalHostname()
   {
      return localHostname;
   }

   public String getIPAddress()
   {
      return ipAddress;
   }

   public String getMachineName()
   {
      return name().toLowerCase();
   }
}
