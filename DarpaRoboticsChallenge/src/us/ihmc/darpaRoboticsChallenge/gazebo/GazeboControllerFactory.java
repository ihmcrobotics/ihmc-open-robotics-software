package us.ihmc.darpaRoboticsChallenge.gazebo;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ComponentBasedVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DataProducerVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.DRCEstimatorThread;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.gfe.ThePeoplesGloriousNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.modules.uiConnector.UiPacketToRosMsgRedirector;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.SimulationRosClockPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.robotController.GazeboThreadedRobotController;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.GlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;

public class GazeboControllerFactory
{
   private static final boolean USE_GUI = true;

   private final DRCRobotSensorInformation sensorInformation;

   private static final double gravity = -9.81;

   public GazeboControllerFactory(DRCRobotModel robotModel, String nameSpace, String robotName, String tfPrefix) throws URISyntaxException, IOException
   {
      /*
       * Create registries
       */
      sensorInformation = robotModel.getSensorInformation();


      /*
       * Create network servers/clients
       */

      PacketCommunicator controllerCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
            new IHMCCommunicationKryoNetClassList());

      //      KryoLocalPacketCommunicator packetCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.CONTROLLER.ordinal(), "GazeboPluginController");
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler("GazeboYoVariableServer"), robotModel.getLogModelProvider(), robotModel.getLogSettings(),
            robotModel.getEstimatorDT());

      GlobalDataProducer dataProducer = new GlobalDataProducer(controllerCommunicator);

      /*
       * Create controllers
       */
      MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, dataProducer);
      /*
       * Create sensors
       */
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      GazeboSensorReaderFactory sensorReaderFactory = new GazeboSensorReaderFactory(sensorInformation, stateEstimatorParameters);

      /*
       * Create output writer
       */

      GazeboOutputWriter outputWriter = new GazeboOutputWriter(robotModel);

      PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(dataProducer);
      dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

      /*
       * Build controller
       */
      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSensorInformation(), robotModel.getContactPointParameters(),
            robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer, new PeriodicNonRealtimeThreadScheduler("DRCPoseCommunicator"), dataProducer, yoVariableServer, gravity);
      estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer,
            outputWriter, dataProducer, yoVariableServer, gravity, robotModel.getEstimatorDT());

      /*
       * Setup threads
       */
      GazeboThreadedRobotController robotController = new GazeboThreadedRobotController();
      int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
      int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

      robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
      robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);

      try
      {
         controllerCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
      URI rosURI = NetworkParameters.getROSURI();
      networkModuleParameters.setRosUri(rosURI);
      networkModuleParameters.enableGFECommunicator(true);
      networkModuleParameters.enableLocalControllerCommunicator(true);
      networkModuleParameters.enableUiModule(true);
      networkModuleParameters.enableGFECommunicator(true);

      DRCNetworkProcessor networkProcessor = new DRCNetworkProcessor(robotModel, networkModuleParameters);

      PacketCommunicator gfe_communicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.GFE_COMMUNICATOR, new IHMCCommunicationKryoNetClassList());
      SimulationRosClockPPSTimestampOffsetProvider ppsOffsetProvider = new SimulationRosClockPPSTimestampOffsetProvider();

      new UiPacketToRosMsgRedirector(robotModel, rosURI, gfe_communicator, networkProcessor.getPacketRouter(), nameSpace + "/" + robotName);
      new ThePeoplesGloriousNetworkProcessor(rosURI, gfe_communicator, null, ppsOffsetProvider, robotModel, nameSpace + "/" + robotName, tfPrefix);

      yoVariableServer.start();

      outputWriter.connect();

      sensorReaderFactory.getSensorReader().connect();

      Thread simulationThread = new Thread(robotController);
      simulationThread.start();

//      if (USE_GUI)
//      {

//         new DRCNetworkProcessor(robotModel, networkModuleParameters);
//      }
      try
      {
         simulationThread.join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }

   }

   private MomentumBasedControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, GlobalDataProducer dataProducer)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      ArmControllerParameters armControllerParameters = robotModel.getArmControllerParameters();
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      final HighLevelState initialBehavior;
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      initialBehavior = HighLevelState.WALKING; // HERE!!

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForSlowWalkingOnRobot(walkingControllerParameters);
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters, initialBehavior);

//      controllerFactory.addHighLevelBehaviorFactory(new JointPositionControllerFactory(true));

      if (USE_GUI)
      {
         VariousWalkingProviderFactory variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters, new PeriodicNonRealtimeThreadScheduler("CapturabilityBasedStatusProducer"));
         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);

      }
      else
      {
         VariousWalkingProviderFactory variousWalkingProviderFactory = new ComponentBasedVariousWalkingProviderFactory(true, null, robotModel.getControllerDT());
         controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      }

      return controllerFactory;
   }
}
