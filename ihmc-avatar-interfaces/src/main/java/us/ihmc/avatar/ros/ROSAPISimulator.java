package us.ihmc.avatar.ros;

import java.io.IOException;
import java.net.URI;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.ros.internal.message.Message;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.avatar.rosAPI.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

abstract public class ROSAPISimulator
{
   private static final String DEFAULT_TF_PREFIX = null;
   private static final String DEFAULT_PREFIX = "/ihmc_ros";
   private static final boolean START_UI = false;
   private static final boolean REDIRECT_UI_PACKETS_TO_ROS = false;

   protected final DRCRobotModel robotModel;

   protected static final String DEFAULT_STRING = "DEFAULT";
   protected abstract CommonAvatarEnvironmentInterface createEnvironment();
   protected abstract List<Map.Entry<String, RosTopicSubscriberInterface<? extends Message>>> createCustomSubscribers(String namespace, PacketCommunicator communicator);
   protected abstract List<Map.Entry<String, RosTopicPublisher<? extends Message>>> createCustomPublishers(String namespace, PacketCommunicator communicator);

   public ROSAPISimulator(DRCRobotModel robotModel, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix, boolean runAutomaticDiagnosticRoutine, boolean disableViz) throws IOException
   {
         this(robotModel, startingLocation, nameSpace, tfPrefix, runAutomaticDiagnosticRoutine, disableViz, Collections.<Class>emptySet());
   }

   public ROSAPISimulator(DRCRobotModel robotModel, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix, boolean runAutomaticDiagnosticRoutine, boolean disableViz, Collection<Class> additionalPacketTypes) throws IOException
   {
      this.robotModel = robotModel;

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, createEnvironment());
      simulationStarter.setRunMultiThreaded(true);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      
      URI rosUri = NetworkParameters.getROSURI();
      networkProcessorParameters.setRosUri(rosUri);

      PacketCommunicator rosAPI_communicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.ROS_API_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());

      networkProcessorParameters.enableROSAPICommunicator(true);
      networkProcessorParameters.enableBehaviorModule(true);
      networkProcessorParameters.enableBehaviorVisualizer(true);

      if (runAutomaticDiagnosticRoutine)
      {
         networkProcessorParameters.enableAutomaticDiagnostic(true, 5);
      }

      if (START_UI)
      {
         networkProcessorParameters.enableUiModule(true);
         System.err.println("Cannot start UI automatically from open source projects. Start UI Manually");
      }

      if(disableViz)
      {
         DRCGuiInitialSetup guiSetup = new DRCGuiInitialSetup(false, false, false);
         simulationStarter.setGuiInitialSetup(guiSetup);
      }

      simulationStarter.setStartingLocation(startingLocation);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.startSimulation(networkProcessorParameters, true);

      if (REDIRECT_UI_PACKETS_TO_ROS)
      {
//         PacketRouter<PacketDestination> packetRouter = simulationStarter.getPacketRouter();
//         new UiPacketToRosMsgRedirector(robotModel, rosUri, rosAPI_communicator, packetRouter, DEFAULT_PREFIX + "/" + robotModel.getSimpleRobotName().toLowerCase());
      }
      
      LocalObjectCommunicator sensorCommunicator = simulationStarter.getSimulatedSensorsPacketCommunicator();
      RobotROSClockCalculator rosClockCalculator = new RobotROSClockCalculatorFromPPSOffset(new SimulationRosClockPPSTimestampOffsetProvider());
      new ThePeoplesGloriousNetworkProcessor(rosUri, rosAPI_communicator, sensorCommunicator, rosClockCalculator, robotModel, nameSpace, tfPrefix, additionalPacketTypes,
            createCustomSubscribers(nameSpace, rosAPI_communicator), createCustomPublishers(nameSpace, rosAPI_communicator));
   }

   protected static Options parseArguments(String[] args) throws JSAPException, IOException
   {
      JSAP jsap = new JSAP();

      FlaggedOption rosNameSpace = new FlaggedOption("namespace").setLongFlag("namespace").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      rosNameSpace.setDefault(DEFAULT_PREFIX);
      
      FlaggedOption tfPrefix = new FlaggedOption("tfPrefix").setLongFlag("tfPrefix").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      tfPrefix.setDefault(DEFAULT_TF_PREFIX);

      FlaggedOption model = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      model.setDefault(DEFAULT_STRING);
      
      FlaggedOption location = new FlaggedOption("startingLocation").setLongFlag("location").setShortFlag('s').setRequired(false).setStringParser(
            JSAP.STRING_PARSER);
      location.setDefault(DEFAULT_STRING);

      Switch visualizeSCSSwitch = new Switch("disable-visualize").setShortFlag('d').setLongFlag("disable-visualize");
      visualizeSCSSwitch.setHelp("Disable rendering/visualization of Simulation Construction Set");

      Switch requestAutomaticDiagnostic = new Switch("requestAutomaticDiagnostic").setLongFlag("requestAutomaticDiagnostic").setShortFlag(JSAP.NO_SHORTFLAG);
      requestAutomaticDiagnostic.setHelp("enable automatic diagnostic routine");

      jsap.registerParameter(model);
      jsap.registerParameter(location);
      jsap.registerParameter(rosNameSpace);
      jsap.registerParameter(tfPrefix);
      jsap.registerParameter(requestAutomaticDiagnostic);
      jsap.registerParameter(visualizeSCSSwitch);
      JSAPResult config = jsap.parse(args);
      
      Options options = new Options();
      options.robotModel = config.getString(model.getID());
      options.disableViz = config.getBoolean(visualizeSCSSwitch.getID());
      options.startingLocation = config.getString(location.getID());
      options.tfPrefix = config.getString(tfPrefix.getID());
      options.nameSpace = config.getString(rosNameSpace.getID());
      options.runAutomaticDiagnosticRoutine = config.getBoolean(requestAutomaticDiagnostic.getID());
      return options;
   }
   
   protected static class Options
   {
      public String robotModel;
      public String startingLocation;
      public String nameSpace;
      public String tfPrefix;
      public boolean runAutomaticDiagnosticRoutine;
      public boolean disableViz;
   }
}
