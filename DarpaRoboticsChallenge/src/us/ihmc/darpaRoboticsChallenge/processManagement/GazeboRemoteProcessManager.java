package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Properties;
import java.util.concurrent.ConcurrentHashMap;

import javax.swing.JOptionPane;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig.LocalCloudMachines;

import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.ChannelShell;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

public class GazeboRemoteProcessManager
{
   public static final char KILL_CHAR = 0x03;
   private String username = System.getProperty("user.name");
   private String ip = "localhost";
   private String hostname = "localhost";
   private String password;

   private JSch jsch;
   private Properties config;

   private static volatile ConcurrentHashMap<LocalCloudMachines, String> runningSims = new ConcurrentHashMap<LocalCloudMachines, String>();

   private static volatile ConcurrentHashMap<LocalCloudMachines, Session> sessions = new ConcurrentHashMap<LocalCloudMachines, Session>();
   private static volatile ConcurrentHashMap<Session, ChannelShell> shellChannels = new ConcurrentHashMap<Session, ChannelShell>();
   private static volatile ConcurrentHashMap<ChannelShell, PrintStream> shellPrintStreams = new ConcurrentHashMap<ChannelShell, PrintStream>();

   public GazeboRemoteProcessManager()
   {
      jsch = new JSch();

      config = new Properties();
      config.put("StrictHostKeyChecking", "no");

      scanForSims();
   }
   
   public void sendCommandThroughShellChannel(LocalCloudMachines machine, String command)
   {
      shellCommandStreamForMachine(machine).println(command);
   }
   
   public void sendCommandThroughExecChannel(LocalCloudMachines machine, String command)
   {
      try
      {
         ChannelExec channel = (ChannelExec) sessionForMachine(machine).openChannel("exec");
         channel.setCommand(command);
         channel.setInputStream(null);
         channel.setOutputStream(System.out);
      }
      catch (JSchException e)
      {
         e.printStackTrace();
      }
   }
   
   public void startRosSim()
   {
      
   }

   public boolean isMachineRunningSim(LocalCloudMachines machine)
   {
      if (getRosSimPID(machine) < 0)
         return false;
      else
         return true;
   }

   public int getRosSimPID(LocalCloudMachines machine)
   {
      int pid = -1;

      try
      {
         ChannelExec channel = (ChannelExec) sessions.get(machine).openChannel("exec");
         //         channel.setCommand("/home/unknownid/workspace/GazeboStateCommunicator/util/getRoslaunchPID.sh");
         channel.setCommand(GazeboProcessManagementCommandStrings.GET_ROSLAUNCH_PID);
         channel.setInputStream(null);
         channel.setOutputStream(System.out);

         BufferedReader reader = new BufferedReader(new InputStreamReader(channel.getInputStream()));

         channel.connect();

         String tmp = reader.readLine();

         if (tmp != null)
            pid = Integer.parseInt(tmp);

         //         System.out.println(pid);

         channel.disconnect();
      }
      catch (JSchException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return pid;
   }

   public String getRosSimTaskname(LocalCloudMachines machine)
   {
      String taskName = "";

      try
      {
         ChannelExec channel = (ChannelExec) sessions.get(machine).openChannel("exec");
         //         channel.setCommand("/home/unknownid/workspace/GazeboStateCommunicator/util/getRoslaunchTask.sh");
         channel.setCommand(GazeboProcessManagementCommandStrings.GET_ROSLAUNCH_TASK);
         channel.setInputStream(null);
         channel.setOutputStream(System.out);

         BufferedReader reader = new BufferedReader(new InputStreamReader(channel.getInputStream()));

         channel.connect();

         taskName = reader.readLine();

         //         System.out.println(taskName);

         channel.disconnect();
      }
      catch (JSchException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return taskName;
   }

   private Session sessionForMachine(LocalCloudMachines machine)
   {
      return sessions.get(machine);
   }

   private ChannelShell shellChannelForMachine(LocalCloudMachines machine)
   {
      return shellChannels.get(sessionForMachine(machine));
   }

   private PrintStream shellCommandStreamForMachine(LocalCloudMachines machine)
   {
      return shellPrintStreams.get(shellChannelForMachine(machine));
   }

   private void initCredentials(LocalCloudMachines machine)
   {
      ip = DRCLocalCloudConfig.getIPAddress(machine);
      hostname = DRCLocalCloudConfig.getHostName(machine);

      if (hostname == "localhost")
      {
         username = System.getProperty("user.name");
         password = JOptionPane.showInputDialog("Enter your local password");
      }
      else
      {
         username = "unknownid";
         password = "unknownpw";
      }
   }

   private void setupSession(LocalCloudMachines machine) throws JSchException
   {
      Session session;
      session = jsch.getSession(username, ip, 22);

      session.setConfig(config);

      session.setPassword(password);

      session.connect(30000);

      sessions.put(machine, session);

      try
      {
         ChannelShell channel = (ChannelShell) sessions.get(machine).openChannel("shell");
         shellPrintStreams.put(channel, new PrintStream(channel.getOutputStream(), true));

         channel.setOutputStream(System.out);
         channel.setInputStream(null);

         channel.connect(30000);

         shellChannels.put(session, channel);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void scanForSims()
   {
      try
      {
         for (LocalCloudMachines machine : LocalCloudMachines.values())
         {
            if (machine != LocalCloudMachines.LOCALHOST)
            {
               //               System.out.println("Checking " + machine);
               initCredentials(machine);
               if (InetAddress.getByName(ip).isReachable(500))
               {
                  setupSession(machine);
                  if (isMachineRunningSim(machine))
                  {
                     //                     System.out.println(machine + " is running " + task);
                     runningSims.put(machine, getRosSimTaskname(machine));
                  }
                  else
                  {
                     shutdownSession(machine);
                  }
               }
            }
         }
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      catch (JSchException e)
      {
         e.printStackTrace();
      }
   }

   private void shutdownSession(LocalCloudMachines machine)
   {
      Session session = sessions.get(machine);
      ChannelShell channel = shellChannels.get(session);

      sessions.remove(machine);
      shellChannels.remove(session);
      shellPrintStreams.remove(channel);

      session.disconnect();
      channel.disconnect();

   }
}
