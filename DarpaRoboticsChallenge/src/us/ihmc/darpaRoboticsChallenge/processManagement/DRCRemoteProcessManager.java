package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.net.InetAddress;
import java.util.Properties;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ConcurrentHashMap;

import javax.swing.JOptionPane;

import org.apache.commons.lang.mutable.MutableBoolean;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig.LocalCloudMachines;

import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.ChannelShell;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

public class DRCRemoteProcessManager extends Thread
{
   private static final boolean DEBUG = false;

   public static final char KILL_CHAR = 0x03;
   private String username = System.getProperty("user.name");
   private String ip = "localhost";
   private String hostname = "localhost";
   private String password;

   private JSch jsch;
   private Properties config;

   private static volatile ConcurrentHashMap<LocalCloudMachines, String> runningRosTaskNames = new ConcurrentHashMap<LocalCloudMachines, String>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, Integer> runningRosPIDs = new ConcurrentHashMap<LocalCloudMachines, Integer>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, Integer> runningControllerPIDs = new ConcurrentHashMap<LocalCloudMachines, Integer>();

   private static volatile ConcurrentHashMap<LocalCloudMachines, Session> sessions = new ConcurrentHashMap<LocalCloudMachines, Session>();
   private static volatile ConcurrentHashMap<Session, ChannelShell> shellChannels = new ConcurrentHashMap<Session, ChannelShell>();
   private static volatile ConcurrentHashMap<ChannelShell, PrintStream> shellPrintStreams = new ConcurrentHashMap<ChannelShell, PrintStream>();

   private static volatile ConcurrentHashMap<LocalCloudMachines, MutableBoolean> reachability = new ConcurrentHashMap<LocalCloudMachines, MutableBoolean>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, MutableBoolean> isRunningRos = new ConcurrentHashMap<DRCLocalCloudConfig.LocalCloudMachines,
                                                                                                   MutableBoolean>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, MutableBoolean> isRunningController =
      new ConcurrentHashMap<DRCLocalCloudConfig.LocalCloudMachines, MutableBoolean>();

   private static final int REACHABLE_TIMEOUT = 5000;
   private static final int CONNECTION_TIMEOUT = 5000;

   private boolean waitingOnGazebo = false;

   public DRCRemoteProcessManager()
   {

      jsch = new JSch();

      config = new Properties();
      config.put("StrictHostKeyChecking", "no");

      for(LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if(machine != LocalCloudMachines.LOCALHOST)
         {
            reachability.put(machine, new MutableBoolean(false));
            isRunningRos.put(machine, new MutableBoolean(false));
            isRunningController.put(machine, new MutableBoolean(false));
         }
      }
   }

   public void launchSim(String task, LocalCloudMachines gazeboMachine, String launchCommandType)
   {
      String drcTask;

      initCredentials(gazeboMachine);

      if (isMachineReachable(gazeboMachine))
      {
         checkSessionStatus(gazeboMachine);

         if (isMachineRunningSim(gazeboMachine))
         {
            System.err.println(gazeboMachine + " is already running a sim.");
         }
         else
         {
            if (launchCommandType.contains("plugin"))
               drcTask = DRCDashboardTypes.getPluginCommand(DRCDashboardTypes.DRCPluginTasks.valueOf(task));
            else
               drcTask = DRCDashboardTypes.getDefaultCommand(DRCDashboardTypes.DRCROSTasks.valueOf(task));

            sendCommandThroughShellChannel(gazeboMachine, drcTask);

            updateRosSimPID(gazeboMachine);
            updateRosSimTaskname(gazeboMachine);
         }
      }
   }

   public void killSim(LocalCloudMachines gazeboMachine)
   {
      if(isMachineReachable(gazeboMachine))
      {
         if(isMachineRunningSim(gazeboMachine))
         {
            sendCommandThroughExecChannel(gazeboMachine, "kill -2 " + getRosSimPID(gazeboMachine));
            updateRosSimPID(gazeboMachine);
            updateRosSimTaskname(gazeboMachine);
            isRunningRos.get(gazeboMachine).setValue(false);
         }
         else
         {
            System.err.println("No sim running on " + gazeboMachine);
         }
      }
   }

   public boolean isWaitingOnGazebo()
   {
      return waitingOnGazebo;
   }

   public void run()
   {
      initializeNetworkStatus();

      setupNetworkPollingTimer();

      while(true);
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

   public boolean isMachineReachable(LocalCloudMachines machine)
   {
      return reachability.get(machine).booleanValue();
   }

   public boolean isMachineRunningSim(LocalCloudMachines machine)
   {
      return isRunningRos.get(machine).booleanValue();
   }

   public boolean isMachineRunningController(LocalCloudMachines machine)
   {
      return isRunningController.get(machine).booleanValue();
   }

   public String getRunningRosSimTaskName(LocalCloudMachines machine)
   {
      return runningRosTaskNames.get(machine);
   }

   public int getRosSimPID(LocalCloudMachines machine)
   {
      return runningRosPIDs.get(machine);
   }

   public int getControllerPID(LocalCloudMachines machine)
   {
      return runningControllerPIDs.get(machine);
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

         channel.setOutputStream(null);
         channel.setInputStream(null);

         channel.connect(CONNECTION_TIMEOUT);

         shellChannels.put(session, channel);

         channel.setOutputStream(System.out);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void initializeNetworkStatus()
   {
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         try
         {
            if (machine != LocalCloudMachines.LOCALHOST)
            {
               initCredentials(machine);

               if (InetAddress.getByName(ip).isReachable(REACHABLE_TIMEOUT))
               {
                  reachability.put(machine, new MutableBoolean(true));
                  checkSessionStatus(machine);
                  updateRosSimPID(machine);

                  if (getRosSimPID(machine) > 0)
                  {
                     updateRosSimTaskname(machine);
                     runningRosTaskNames.put(machine, runningRosTaskNames.get(machine));
                     isRunningRos.put(machine, new MutableBoolean(true));
                  }
                  else
                  {
                     isRunningRos.put(machine, new MutableBoolean(false));
                  }

                  updateControllerPID(machine);

                  if (runningControllerPIDs.get(machine) > 0)
                  {
                     isRunningController.put(machine, new MutableBoolean(false));
                     runningControllerPIDs.put(machine, runningControllerPIDs.get(machine));
                  }
               }
            }
         }
         catch (Exception e)
         {
            reachability.put(machine, new MutableBoolean(false));
            isRunningController.put(machine, new MutableBoolean(false));
            isRunningRos.put(machine, new MutableBoolean(false));
         }
      }
   }

   private void setupNetworkPollingTimer()
   {
      Timer networkPollingTimer = new Timer();

      networkPollingTimer.schedule(new TimerTask()
      {
         @Override
         public void run()
         {
            updateNetworkStatus();

         }
      }, 0l, 3000l);
   }

   private void updateNetworkStatus()
   {
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            try
            {
               if (InetAddress.getByName(ip).isReachable(REACHABLE_TIMEOUT))
               {
                  reachability.get(machine).setValue(true);
                  checkSessionStatus(machine);
                  updateControllerPID(machine);
                  if (getControllerPID(machine) > 0)
                     isRunningController.get(machine).setValue(true);
                  else
                     isRunningController.get(machine).setValue(false);

                  updateRosSimPID(machine);
                  if (getRosSimPID(machine) > 0)
                     isRunningRos.get(machine).setValue(true);
                  else
                     isRunningRos.get(machine).setValue(false);

                  updateRosSimTaskname(machine);
               }
               else
               {
                  reachability.get(machine).setValue(false);
                  isRunningRos.get(machine).setValue(false);
                  isRunningController.get(machine).setValue(false);

                  runningControllerPIDs.remove(machine);
                  runningControllerPIDs.put(machine, -1);

                  runningRosPIDs.remove(machine);
                  runningRosPIDs.put(machine, -1);

                  runningRosTaskNames.remove(machine);
                  runningRosTaskNames.put(machine, null);
               }
            }
            catch (Exception e)
            {
               shutdownSession(machine);

               reachability.get(machine).setValue(false);
               isRunningRos.get(machine).setValue(false);
               isRunningController.get(machine).setValue(false);

               runningControllerPIDs.remove(machine);
//               runningControllerPIDs.put(machine, -1);

               runningRosPIDs.remove(machine);
//               runningRosPIDs.put(machine, -1);

               runningRosTaskNames.remove(machine);
//               runningRosTaskNames.put(machine, "");
            }
         }
      }
   }

   private void updateRosSimPID(LocalCloudMachines machine)
   {
      int pid = -1;
      try
      {
         ChannelExec channel = (ChannelExec) sessions.get(machine).openChannel("exec");
         channel.setCommand(GazeboProcessManagementCommandStrings.GET_ROSLAUNCH_PID);
         channel.setInputStream(null);
         channel.setOutputStream(System.out);

         BufferedReader reader = new BufferedReader(new InputStreamReader(channel.getInputStream()));

         channel.connect(CONNECTION_TIMEOUT);

         String tmp = reader.readLine();

         if (tmp != null)
            pid = Integer.parseInt(tmp);

         if (DEBUG)
            System.out.println(pid);

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

      runningRosPIDs.remove(machine);
      runningRosPIDs.put(machine, pid);
   }

   private void updateControllerPID(LocalCloudMachines machine)
   {
      int pid = -1;

      try
      {
         ChannelExec channel = (ChannelExec) sessions.get(machine).openChannel("exec");
         channel.setCommand(GazeboProcessManagementCommandStrings.GET_SCS_PID);
         channel.setInputStream(null);
         channel.setOutputStream(System.out);

         BufferedReader reader = new BufferedReader(new InputStreamReader(channel.getInputStream()));

         channel.connect(CONNECTION_TIMEOUT);

         String tmp = reader.readLine();

         if (tmp != null)
            pid = Integer.parseInt(tmp);

         if (DEBUG)
            System.out.println(pid);

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

      runningControllerPIDs.remove(machine);
      runningControllerPIDs.put(machine, pid);
   }

   private void updateRosSimTaskname(LocalCloudMachines machine)
   {
      String taskName = null;

      try
      {
         ChannelExec channel = (ChannelExec) sessions.get(machine).openChannel("exec");
         channel.setCommand(GazeboProcessManagementCommandStrings.GET_ROSLAUNCH_TASK);
         channel.setInputStream(null);
         channel.setOutputStream(System.out);

         BufferedReader reader = new BufferedReader(new InputStreamReader(channel.getInputStream()));

         channel.connect(CONNECTION_TIMEOUT);

         taskName = reader.readLine();

         if (DEBUG)
            System.out.println(taskName);

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

      runningRosTaskNames.remove(machine);
      runningRosTaskNames.put(machine, taskName);
   }

   private void checkSessionStatus(LocalCloudMachines machine)
   {
      if (sessions.containsKey(machine) && sessions.get(machine).isConnected())
      {
         return;
      }
      else
      {
         if(sessions.containsKey(machine))
            shutdownSession(machine);

         try
         {
            setupSession(machine);
         }
         catch (JSchException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void shutdownSession(LocalCloudMachines machine)
   {
      shellCommandStreamForMachine(machine).close();
      shellChannelForMachine(machine).disconnect();
      sessionForMachine(machine).disconnect();
      shellPrintStreams.remove(shellChannels.get(sessions.get(machine)));
      shellChannels.remove(sessions.get(machine));
      sessions.remove(machine);
   }
}
