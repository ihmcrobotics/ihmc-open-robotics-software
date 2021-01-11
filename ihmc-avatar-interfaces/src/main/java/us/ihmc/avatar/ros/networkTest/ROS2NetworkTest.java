package us.ihmc.avatar.ros.networkTest;

import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.avatar.ros.networkTest.profiles.IntegersAt100HzNetworkTestProfile;
import us.ihmc.avatar.ros.networkTest.profiles.IntegersAt1HzNetworkTestProfile;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataVisualizer.BasicYoVariablesUpdatedListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import javax.swing.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedDeque;

public class ROS2NetworkTest
{
   private static final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ISO_LOCAL_DATE_TIME;
   public static final double UPDATE_PERIOD = UnitConversions.hertzToSeconds(100.0);
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private PausablePeriodicThread yoServerUpdateThread;
   private ConcurrentLinkedDeque<YoVariableClient> yoVariableClients = new ConcurrentLinkedDeque<>();
   private YoVariableServer yoVariableServer;
   private SimulationConstructionSet scs;
   private boolean paused;

   private YoDouble experimentTime;

   private final HashMap<String, ROS2NetworkTestProfile> profiles = new HashMap<>();
   {
      addProfile(IntegersAt1HzNetworkTestProfile.class);
      addProfile(IntegersAt100HzNetworkTestProfile.class);
   }

   public ROS2NetworkTest(List<String> args)
   {
      String profileName = args.get(args.indexOf("--profile") + 1);
      ROS2NetworkTestProfile profile = profiles.get(profileName);
      LogTools.info("Running profile: {}", profileName);

      experimentTime = new YoDouble(ROS2NetworkTestMachine.getLocalMachine().getMachineName() + "Time", profile.getYoRegistry());

      LocalDateTime now = LocalDateTime.now();
      LocalDateTime startTime;

      if (!args.contains("--startTime"))
      {
         LogTools.info("Running as host.");

         // decide time to start
         startTime = now.plusSeconds(15);
         String formattedStartTime = startTime.format(dateTimeFormatter);
         LogTools.info("Starting nodes at {}", formattedStartTime);

         ArrayList<YoVariableClient> yoVariableClients = new ArrayList<>();
         ConcurrentLinkedDeque<BasicYoVariablesUpdatedListener> clientUpdateListeners = new ConcurrentLinkedDeque<>();
         Object variableSynchronizer = new Object();
         profile.getRemoteMachines().parallelStream().forEach(remoteMachine ->
         //for (ROS2NetworkTestMachine remoteMachine : profile.getRemoteMachines())
         {
            try
            {
               ThreadTools.startAThread(() ->
               {
                  // use SSHJ to remote execute this class with arguments
                  // connect to server at hostname
                  RemoteSSHTools.session(remoteMachine.getSSHHostname(), remoteMachine.getUsername(), connection ->
                  {
                     // find deploy directory and start the script
                     connection.exec(remoteMachine.getDeployDirectory() + "/ROS2NetworkTest" +
                                     " --profile " + profileName +
                                     " --startTime " + formattedStartTime
                     );
                  });
               }, "RemoteTestExecutor");

               ThreadTools.sleepSeconds(10.0);

               // start YoVariableClient
               BasicYoVariablesUpdatedListener clientUpdateListener = new BasicYoVariablesUpdatedListener(yoRegistry);
               YoVariableClient yoVariableClient = new YoVariableClient(clientUpdateListener);
               LogTools.info("Connecting to {}:{}", remoteMachine.getIPAddress(), DataServerSettings.DEFAULT_PORT);
               yoVariableClient.start(remoteMachine.getIPAddress(), DataServerSettings.DEFAULT_PORT);
               yoVariableClient.setVariableSynchronizer(variableSynchronizer);
               clientUpdateListeners.add(clientUpdateListener);
               yoVariableClients.add(yoVariableClient);
            }
            catch (Exception e)
            {
               e.printStackTrace();
               throw e;
            }
         });
         //}

         // wait until they are all started
         for (BasicYoVariablesUpdatedListener clientUpdateListener : clientUpdateListeners)
         {
            if (!clientUpdateListener.isHandshakeComplete())
               LogTools.info("Waiting for handshake...");
            while (!clientUpdateListener.isHandshakeComplete())
            {
               ThreadTools.sleep(100);
            }
            LogTools.info("Handshake complete.");
         }

         // spin up SCS thread to tick it at some frequency

         // start SCS
         LogTools.info("Starting SCS");
         scs = new SimulationConstructionSet(new Robot("robot"),
                                             new NullGraphics3DAdapter(),
                                             new SimulationConstructionSetParameters());
         yoRegistry.addChild(profile.getYoRegistry()); // add this node's profile variables
         scs.addYoRegistry(yoRegistry);
         YoDouble yoTime = (YoDouble) scs.getRootRegistry().findVariable("t");
         scs.setDT(UPDATE_PERIOD, 1);
         scs.setupGraph(yoTime.getName());
         scs.skipLoadingDefaultConfiguration();
         scs.hideViewport();
         scs.changeBufferSize(4096);
         scs.getGUI().getFrame().setSize(1600, 900);
         scs.setScrollGraphsEnabled(false);
         JToggleButton pauseButton = new JToggleButton("Pause/End"); // TODO Fix up
         pauseButton.addActionListener(e ->
         {
            paused = pauseButton.isSelected();
            scs.setScrollGraphsEnabled(paused);
         });
         scs.addButton(pauseButton);

         List<ROS2NetworkTestMachine> remoteMachines = profile.getRemoteMachines();
         String[] nanoTimeGraphs = new String[remoteMachines.size() + 1];
         nanoTimeGraphs[0] = experimentTime.getName();
         for (int i = 0; i < remoteMachines.size(); i++)
         {
            nanoTimeGraphs[i + 1] = remoteMachines.get(i).getMachineName() + "Time";
         }
         scs.setupGraph(nanoTimeGraphs);
         for (String[] graph : profile.getGraphsToSetup())
         {
            LogTools.info("Setting up graph: {}", Arrays.toString(graph));
            scs.setupGraph(graph);
         }

         scs.startOnAThread();
         while (!scs.hasSimulationThreadStarted())
            ThreadTools.sleep(200);

         new PausablePeriodicThread("SCSUpdater", UPDATE_PERIOD, () ->
         {
            if (!paused)
            {
               yoTime.add(UPDATE_PERIOD);
               synchronized (variableSynchronizer)
               {
                  experimentTime.set(Conversions.nanosecondsToSeconds(-LocalDateTime.now().until(startTime, ChronoUnit.NANOS)));
                  profile.updateDerivativeVariables(yoRegistry);

                  // dynamically resize buffer
                  int bufferSize = scs.getDataBuffer().getBufferSize();
                  if (scs.getDataBuffer().getCurrentIndex() == bufferSize - 2)
                  {
                     scs.changeBufferSize(bufferSize + bufferSize / 2);
                  }

                  scs.tickAndUpdate();
               }
            }
         }).start();
      }
      else
      {
         if (!args.contains("--startTime"))
         {
            throw new RuntimeException("Client mode requires a time to start specified with --startTime");
         }

         LogTools.info("Running as client.");

         String startTimeString = args.get(args.indexOf("--startTime") + 1);
         LogTools.info("Start time: {}", startTimeString);
         startTime = LocalDateTime.parse(startTimeString, dateTimeFormatter);

         // start YoVariableServer
         yoVariableServer = new YoVariableServer(profile.getMachineName() + profileName, null, new DataServerSettings(false), UPDATE_PERIOD);
         yoVariableServer.setMainRegistry(profile.getYoRegistry(), null, null);
         LogTools.info("Starting YoVariableServer...");
         yoVariableServer.start();

         LogTools.info("Starting server update thread...");
         MutableLong timestamp = new MutableLong();
         yoServerUpdateThread = new PausablePeriodicThread("YoServerUpdate", UPDATE_PERIOD, () ->
         {
            experimentTime.set(Conversions.nanosecondsToSeconds(-LocalDateTime.now().until(startTime, ChronoUnit.NANOS)));
            yoVariableServer.update(timestamp.getAndAdd(Conversions.secondsToNanoseconds(UPDATE_PERIOD)));
         });
         yoServerUpdateThread.start();
      }

      if (!args.contains("--profile"))
      {
         throw new RuntimeException("Must select a profile with --profile");
      }

      // wait until decided time
      LogTools.info("Waiting to start test...");
      ThreadTools.sleep(LocalDateTime.now().until(startTime, ChronoUnit.MILLIS));
      LogTools.info("Starting test");

      profile.runExperiment();
      LogTools.info("Experiment finished.");
      ThreadTools.sleepSeconds(7.0); // allow those last messages to come in

      LogTools.info("Stopping experiment...");
      paused = true;

      if (scs != null)
         scs.setScrollGraphsEnabled(true);
      if (yoServerUpdateThread != null)
         yoServerUpdateThread.destroy();
      if (yoVariableServer != null)
         yoVariableServer.close();
      for (YoVariableClient yoVariableClient : yoVariableClients)
         yoVariableClient.stop();
      profile.destroy();
   }

   private void addProfile(Class<? extends ROS2NetworkTestProfile> clazz)
   {
      profiles.put(clazz.getSimpleName(), ExceptionTools.handle(clazz::newInstance, DefaultExceptionHandler.RUNTIME_EXCEPTION));
   }

   public static void main(String[] args)
   {
      new ROS2NetworkTest(Arrays.asList(args));
   }
}
