package us.ihmc.darpaRoboticsChallenge;

import java.io.IOException;
import java.net.URI;
import java.util.List;
import java.util.Map;

import org.ros.internal.message.Message;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JointPositionControllerFactory;
import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.LocalObjectCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

abstract public class ROSAPISimulator
{
   private static final String DEFAULT_TF_PREFIX = null;
   private static final String DEFAULT_PREFIX = "/ihmc_ros";
   private static final boolean START_UI = false;
   private static final boolean REDIRECT_UI_PACKETS_TO_ROS = false;
   
   protected static final String DEFAULT_STRING = "DEFAULT";
   protected abstract CommonAvatarEnvironmentInterface createEnvironment();
   protected abstract List<Map.Entry<String, RosTopicSubscriberInterface<? extends Message>>> createCustomSubscribers(PacketCommunicator communicator);
   protected abstract List<Map.Entry<String, RosTopicPublisher<? extends Message>>> createCustomPublishers(PacketCommunicator communicator);

   public ROSAPISimulator(DRCRobotModel robotModel, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix, boolean runAutomaticDiagnosticRoutine, boolean disableViz) throws IOException
   {
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, createEnvironment());
      simulationStarter.setRunMultiThreaded(true);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      
      URI rosUri = NetworkParameters.getROSURI();
      networkProcessorParameters.setRosUri(rosUri);

      PacketCommunicator gfe_communicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());

      networkProcessorParameters.enableGFECommunicator(true);
      if (runAutomaticDiagnosticRoutine)
      {
         networkProcessorParameters.enableBehaviorModule(true);
         networkProcessorParameters.enableBehaviorVisualizer(true);
         networkProcessorParameters.enableAutomaticDiagnostic(true, 5);
      }

      if (START_UI)
      {
         networkProcessorParameters.enableUiModule(true);
         simulationStarter.startOpertorInterfaceUsingProcessSpawner();
      }

      if(disableViz)
      {
         DRCGuiInitialSetup guiSetup = new DRCGuiInitialSetup(false, false, false);
         simulationStarter.setGuiInitialSetup(guiSetup);
      }

      simulationStarter.registerHighLevelController(new JointPositionControllerFactory(false));
      simulationStarter.setStartingLocation(startingLocation);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.startSimulation(networkProcessorParameters, true);

      if (REDIRECT_UI_PACKETS_TO_ROS)
      {
         PacketRouter<PacketDestination> packetRouter = simulationStarter.getPacketRouter();
         new UiPacketToRosMsgRedirector(robotModel, rosUri, gfe_communicator, packetRouter, DEFAULT_PREFIX + "/" + robotModel.getSimpleRobotName().toLowerCase());
      }
      
      LocalObjectCommunicator sensorCommunicator = simulationStarter.getSimulatedSensorsPacketCommunicator();
      SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
      new ThePeoplesGloriousNetworkProcessor(rosUri, gfe_communicator, sensorCommunicator, ppsOffsetProvider, robotModel, nameSpace, tfPrefix,
            createCustomSubscribers(gfe_communicator), createCustomPublishers(gfe_communicator));
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
