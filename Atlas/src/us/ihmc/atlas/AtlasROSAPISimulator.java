package us.ihmc.atlas;

import java.io.IOException;
import java.net.URI;

import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationTools;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class AtlasROSAPISimulator
{
   private static String defaultPrefix = "/ihmc_msgs/atlas";
   private static String defaultRobotModel = "ATLAS_UNPLUGGED_V5_NO_HANDS";
   private final boolean startUI = true;
   private boolean redirectUiPacketsToRos = true;
   private KryoLocalPacketCommunicator gfe_communicator;

   public AtlasROSAPISimulator(DRCRobotModel robotModel, String nameSpace, boolean runAutomaticDiagnosticRoutine) throws IOException
   {
      DRCSimulationStarter simulationStarter = DRCSimulationTools.createObstacleCourseSimulationStarter(robotModel);
      simulationStarter.setRunMultiThreaded(true);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      
      URI rosUri = NetworkParameters.getROSURI();
      networkProcessorParameters.setRosUri(rosUri);

      gfe_communicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.GFE.ordinal(), "GFE_Communicator");
      networkProcessorParameters.setGFEPacketCommunicator(gfe_communicator);

      if (runAutomaticDiagnosticRoutine)
      {
         networkProcessorParameters.setUseBehaviorModule(true);
         networkProcessorParameters.setUseBehaviorVisualizer(true);
         networkProcessorParameters.setRunAutomaticDiagnostic(true, 5);
      }

      if (startUI)
      {
         networkProcessorParameters.setUseUiModule(true);
         simulationStarter.startOpertorInterfaceUsingProcessSpawner();
      }

      simulationStarter.startSimulation(networkProcessorParameters, true);

      if (redirectUiPacketsToRos)
      {
         PacketRouter packetRouter = simulationStarter.getPacketRouter();
         new UiPacketToRosMsgRedirector(robotModel, rosUri, gfe_communicator, packetRouter);
      }
      
      PacketCommunicator sensorCommunicator = simulationStarter.getSimulatedSensorsPacketCommunicator();
      SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();
      new ThePeoplesGloriousNetworkProcessor(rosUri, gfe_communicator, sensorCommunicator, ppsOffsetProvider, robotModel, nameSpace);

   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      JSAP jsap = new JSAP();

      FlaggedOption rosNameSpace = new FlaggedOption("namespace").setLongFlag("namespace").setShortFlag(JSAP.NO_SHORTFLAG).setRequired(false)
            .setStringParser(JSAP.STRING_PARSER);
      rosNameSpace.setDefault(defaultPrefix);

      FlaggedOption model = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(false).setStringParser(JSAP.STRING_PARSER);
      model.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
      model.setDefault(defaultRobotModel);

      Switch requestAutomaticDiagnostic = new Switch("requestAutomaticDiagnostic").setLongFlag("requestAutomaticDiagnostic").setShortFlag(JSAP.NO_SHORTFLAG);
      requestAutomaticDiagnostic.setHelp("enable automatic diagnostic routine");

      jsap.registerParameter(model);
      jsap.registerParameter(rosNameSpace);
      jsap.registerParameter(requestAutomaticDiagnostic);
      JSAPResult config = jsap.parse(args);

      DRCRobotModel robotModel;

      try
      {
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), AtlasRobotModel.AtlasTarget.SIM, false);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + config.getString("robotModel"));
         System.out.println(jsap.getHelp());
         return;
      }

      new AtlasROSAPISimulator(robotModel, config.getString("namespace"), config.getBoolean(requestAutomaticDiagnostic.getID()));
   }
}
