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

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jcraft.jsch.ChannelExec;
import com.jcraft.jsch.ChannelShell;
import com.jcraft.jsch.JSch;
import com.jcraft.jsch.JSchException;
import com.jcraft.jsch.Session;

import us.ihmc.darpaRoboticsChallenge.configuration.LocalCloudMachines;

public class GazeboRemoteSimulationAdapter extends Thread
{
   private static final boolean DEBUG = true;

   public static final char KILL_CHAR = 0x03;

   private JSch jsch;
   private Properties config;

   private static volatile ConcurrentHashMap<LocalCloudMachines, String> runningRosTaskNames = new ConcurrentHashMap<LocalCloudMachines, String>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, Integer> runningRosPIDs = new ConcurrentHashMap<LocalCloudMachines, Integer>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, Integer> runningControllerPIDs = new ConcurrentHashMap<LocalCloudMachines, Integer>();

   private static volatile ConcurrentHashMap<LocalCloudMachines, Session> sessions = new ConcurrentHashMap<LocalCloudMachines, Session>();
   private static volatile ConcurrentHashMap<Session, ChannelShell> shellChannels = new ConcurrentHashMap<Session, ChannelShell>();
   private static volatile ConcurrentHashMap<ChannelShell, PrintStream> shellPrintStreams = new ConcurrentHashMap<ChannelShell, PrintStream>();

   private static volatile ConcurrentHashMap<LocalCloudMachines, MutableBoolean> reachability = new ConcurrentHashMap<LocalCloudMachines, MutableBoolean>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, MutableBoolean> isRunningRos = new ConcurrentHashMap<LocalCloudMachines,
                                                                                                   MutableBoolean>();
   private static volatile ConcurrentHashMap<LocalCloudMachines, MutableBoolean> isRunningController =
      new ConcurrentHashMap<LocalCloudMachines, MutableBoolean>();

   private static final int REACHABLE_TIMEOUT = 5000;
   private static final int CONNECTION_TIMEOUT = 5000;

   public GazeboRemoteSimulationAdapter()
   {
      jsch = new JSch();

      config = new Properties();
      config.put("StrictHostKeyChecking", "no");

      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (machine != LocalCloudMachines.LOCALHOST)
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
      if (isMachineReachable(gazeboMachine))
      {
         if (isMachineRunningSim(gazeboMachine))
         {
            try
            {
               int PID = getRosSimPID(gazeboMachine);
               ChannelExec channel = (ChannelExec) sessions.get(gazeboMachine).openChannel("exec");
               channel.setCommand("kill -2 " + PID);
               channel.setInputStream(null);
               channel.setOutputStream(System.out);

               channel.connect(CONNECTION_TIMEOUT);

               channel.disconnect();

               updateRosSimPID(gazeboMachine);
               updateRosSimTaskname(gazeboMachine);

            } catch (JSchException e)
            {
               e.printStackTrace();
            }
         }
         else
         {
            System.err.println("No sim running on " + gazeboMachine);
         }
      }
   }

   public void run()
   {
      initializeNetworkStatus();

      setupNetworkPollingTimer();

//      while (true);
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

   private ImmutablePair<String, String> initCredentials(LocalCloudMachines machine)
   {
      String hostname = machine.getHost();

      String username, password;

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

      return new ImmutablePair<String, String>(username, password);
   }

   private void setupSession(LocalCloudMachines machine, ImmutablePair<String, String> authPair) throws JSchException
   {
      Session session;
      session = jsch.getSession(authPair.getLeft(), machine.getIp(), 22);

      session.setConfig(config);

      session.setPassword(authPair.getRight());

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
               if (InetAddress.getByName(machine.getIp()).isReachable(REACHABLE_TIMEOUT))
               {
                  reachability.get(machine).setValue(true);
                  checkSessionStatus(machine);
                  updateRosSimPID(machine);

                  if (getRosSimPID(machine) > 0)
                  {
                     updateRosSimTaskname(machine);
                     runningRosTaskNames.put(machine, runningRosTaskNames.get(machine));
                     isRunningRos.get(machine).setValue(true);
                  }
                  else
                  {
                     isRunningRos.get(machine).setValue(false);
                  }

                  updateControllerPID(machine);

                  if (runningControllerPIDs.get(machine) > 0)
                  {
                     isRunningController.get(machine).setValue(false);
                     runningControllerPIDs.put(machine, runningControllerPIDs.get(machine));
                  }
               }
            }
         }
         catch (Exception e)
         {
            if (DEBUG)
               e.printStackTrace();
            reachability.get(machine).setValue(false);
            isRunningController.get(machine).setValue(false);
            isRunningRos.get(machine).setValue(false);
         }
      }
   }

   private void setupNetworkPollingTimer()
   {
      Timer networkPollingTimer = new Timer("Gazebo Network Polling Timer");

      networkPollingTimer.schedule(new TimerTask()
      {
         @Override
         public void run()
         {
            updateNetworkStatus();

         }
      }, 0l, REACHABLE_TIMEOUT + 1000l);
   }

   private void updateNetworkStatus()
   {
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            try
            {
               if (InetAddress.getByName(machine.getIp()).isReachable(REACHABLE_TIMEOUT))
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
                  runningRosTaskNames.put(machine, "");
               }
            }
            catch (Exception e)
            {
               if (DEBUG)
                  e.printStackTrace();
               shutdownSession(machine);

               reachability.get(machine).setValue(false);
               isRunningRos.get(machine).setValue(false);
               isRunningController.get(machine).setValue(false);

               runningControllerPIDs.remove(machine);
               runningControllerPIDs.put(machine, -1);

               runningRosPIDs.remove(machine);
               runningRosPIDs.put(machine, -1);

               runningRosTaskNames.remove(machine);
               runningRosTaskNames.put(machine, "");
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

//       if (DEBUG)
//          System.out.println(pid);

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

//       if (DEBUG)
//          System.out.println(pid);

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
      String taskName = "";

      try
      {
         ChannelExec channel = (ChannelExec) sessions.get(machine).openChannel("exec");
         channel.setCommand(GazeboProcessManagementCommandStrings.GET_ROSLAUNCH_TASK);
         channel.setInputStream(null);
         channel.setOutputStream(System.out);

         BufferedReader reader = new BufferedReader(new InputStreamReader(channel.getInputStream()));

         channel.connect(CONNECTION_TIMEOUT);

         String tmp = reader.readLine();
         if (tmp != null)
            taskName = tmp;

         if(taskName.contains("vehicle"))
         {
            taskName = taskName.substring(0, taskName.indexOf("_Purgatory"));
         }

//       if (DEBUG)
//          System.out.println(taskName);

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
         if (sessions.containsKey(machine))
            shutdownSession(machine);

         try
         {
            setupSession(machine, initCredentials(machine));
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
