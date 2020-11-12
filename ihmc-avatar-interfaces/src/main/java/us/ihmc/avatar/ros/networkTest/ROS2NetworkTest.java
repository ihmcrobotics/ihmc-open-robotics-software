package us.ihmc.avatar.ros.networkTest;

import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jMonkeyEngineToolkit.NullGraphics3DAdapter;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;

import javax.swing.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class ROS2NetworkTest
{
   private static final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ISO_LOCAL_DATE_TIME;
   public static final double UPDATE_PERIOD = UnitConversions.hertzToSeconds(100.0);
   private final YoRegistry yoRegistry = new YoRegistry(getClass().getSimpleName());
   private boolean paused;

   private final HashMap<String, ROS2NetworkTestProfile> profiles = new HashMap<>();
   {
      addProfile(IntegersAt1HzNetworkTestProfile.class);
   }

   public ROS2NetworkTest(List<String> args)
   {
      String profileName = args.get(args.indexOf("--profile") + 1);
      ROS2NetworkTestProfile profile = profiles.get(profileName);
      LogTools.info("Running profile: {}", profileName);

      LocalDateTime now = LocalDateTime.now();
      LocalDateTime startTime;

      if (!args.contains("--startTime"))
      {
         LogTools.info("Running as host. Starting SCS.");

         // start SCS
         SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("robot"),
                                                                         new NullGraphics3DAdapter(),
                                                                         new SimulationConstructionSetParameters());
         scs.addYoRegistry(yoRegistry);
         scs.setDT(UPDATE_PERIOD, 1);

         // decide time to start
         startTime = now.plusSeconds(15);
         String formattedStartTime = startTime.format(dateTimeFormatter);
         LogTools.info("Starting nodes at {}", formattedStartTime);

         ArrayList<YoVariableClient> yoVariableClients = new ArrayList<>();
         Object variableSynchronizer = new Object();
         for (ROS2NetworkTestMachine remoteMachine : profile.getRemoteMachines())
         {
            // use SSHJ to remote execute this class with arguments
            // connect to server at hostname
            RemoteSSHTools.session(remoteMachine.getHostname(), remoteMachine.getUsername(), connection ->
            {
               // find deploy directory and start the script
               connection.exec(remoteMachine.getDeployDirectory() + "/ROS2NetworkTest"
                               + " --profile " + profileName
                               + " --startTime " + formattedStartTime
               );
            });

            // start YoVariableClient
            YoVariableClient yoVariableClient = new YoVariableClient(new ROS2NetworkTestYoVariablesUpdatedListener(yoRegistry));
            yoVariableClient.start(remoteMachine.getHostname(), DataServerSettings.DEFAULT_PORT);
            yoVariableClient.setVariableSynchronizer(variableSynchronizer);
            yoVariableClients.add(yoVariableClient);
         }

         // wait until they are all started

         // spin up SCS thread to tick it at some frequency
         scs.setupGraph("t");
         scs.skipLoadingDefaultConfiguration();
         scs.hideViewport();
         scs.changeBufferSize(8096);
         scs.setScrollGraphsEnabled(false);
         JToggleButton pauseButton = new JToggleButton("Pause/End"); // TODO Fix up
         pauseButton.addActionListener(e ->
         {
            paused = pauseButton.isSelected();
            scs.setScrollGraphsEnabled(paused);
         });
         scs.addButton(pauseButton);
         scs.startOnAThread();
         while (!scs.hasSimulationThreadStarted())
            ThreadTools.sleep(200);

         new PausablePeriodicThread("SCSUpdater", UPDATE_PERIOD, () ->
         {
            synchronized (variableSynchronizer)
            {
               scs.tickAndUpdate();
            }
         }).start();
      }
      else
      {
         if (!args.contains("--startTime"))
         {
            throw new RuntimeException("Client mode requires a time to start specified with --startTime");
         }

         String startTimeString = args.get(args.indexOf("--startTime") + 1);
         startTime = LocalDateTime.parse(startTimeString, dateTimeFormatter);

         // start YoVariableServer
         YoVariableServer yoVariableServer = new YoVariableServer(profileName, null, new DataServerSettings(false), UPDATE_PERIOD);
         yoVariableServer.setMainRegistry(profile.getYoRegistry(), null, null);
         yoVariableServer.start();

         MutableLong timestamp = new MutableLong();
         new PausablePeriodicThread("YoServerUpdate", UPDATE_PERIOD, () ->
         {
            yoVariableServer.update(timestamp.getAndAdd(Conversions.secondsToNanoseconds(UPDATE_PERIOD)));
         }).start();
      }

      if (!args.contains("--profile"))
      {
         throw new RuntimeException("Must select a profile with --profile");
      }

      // wait until decided time
      ThreadTools.sleep(LocalDateTime.now().until(startTime, ChronoUnit.MILLIS));


      profile.runExperiment();
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
