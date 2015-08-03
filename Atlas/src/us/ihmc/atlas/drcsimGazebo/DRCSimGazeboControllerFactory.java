package us.ihmc.atlas.drcsimGazebo;

import java.io.IOException;
import java.net.URI;
import java.util.List;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ComponentBasedVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DataProducerVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.JointPositionControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.StampedPosePacket;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.DRCEstimatorThread;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkProcessor;
import us.ihmc.darpaRoboticsChallenge.robotController.DRCSimGazeboThreadedRobotController;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotiq.simulatedHand.SimulatedRobotiqHandsController;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;

public class DRCSimGazeboControllerFactory
{
   private static final boolean USE_GUI = true;

   private final AtlasSensorInformation sensorInformation;

   private static final double gravity = -9.81;
   private static final boolean useRobotiqHands = false;

   public DRCSimGazeboControllerFactory()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.GAZEBO_ATLAS_UNPLUGGED_V5_NO_HANDS, AtlasRobotModel.AtlasTarget.GAZEBO, false);
      /*
       * Create registries
       */
      sensorInformation = (AtlasSensorInformation) robotModel.getSensorInformation();

      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      contactPointParameters.createHandKnobContactPoints();

      /*
       * Create network servers/clients
       */
      PacketCommunicator drcNetworkProcessorServer = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.CONTROLLER_PORT,
            new IHMCCommunicationKryoNetClassList());
      

      //      KryoLocalPacketCommunicator packetCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(), PacketDestination.CONTROLLER.ordinal(), "GazeboPluginController");
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler("DRCSimGazeboYoVariableServer"), robotModel.getLogModelProvider(), robotModel.getLogSettings(),
            robotModel.getEstimatorDT());

      GlobalDataProducer dataProducer = new GlobalDataProducer(drcNetworkProcessorServer);

      /*
       * Create controllers
       */
      MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, dataProducer);
      /*
       * Create sensors
       */
      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      DRCSimGazeboSensorReaderFactory sensorReaderFactory = new DRCSimGazeboSensorReaderFactory(sensorInformation, stateEstimatorParameters);

      /*
       * Create output writer
       */

      AtlasDRCSimGazeboOutputWriter outputWriter = new AtlasDRCSimGazeboOutputWriter(robotModel);

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
      DRCSimGazeboThreadedRobotController robotController = new DRCSimGazeboThreadedRobotController();
      int estimatorTicksPerSimulationTick = (int) Math.round(robotModel.getEstimatorDT() / robotModel.getEstimatorDT());
      int controllerTicksPerSimulationTick = (int) Math.round(robotModel.getControllerDT() / robotModel.getEstimatorDT());

      robotController.addController(estimatorThread, estimatorTicksPerSimulationTick, false);
      robotController.addController(controllerThread, controllerTicksPerSimulationTick, true);

      if (useRobotiqHands)
      {
         boolean createCollisionMeshes = false;
         SDFRobot sdfRobot = robotModel.createSdfRobot(createCollisionMeshes);
         SimulatedRobotiqHandsController simulatedHandsController = (SimulatedRobotiqHandsController) robotModel.createSimulatedHandController(sdfRobot,
               threadDataSynchronizer, dataProducer);
         robotController.addController(simulatedHandsController, controllerTicksPerSimulationTick, true);

         SideDependentList<List<OneDegreeOfFreedomJoint>> allFingerJoints = simulatedHandsController.getAllFingerJoints();
         outputWriter.setFingerJointsProvider(allFingerJoints);
      }

      try
      {
         drcNetworkProcessorServer.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      yoVariableServer.start();

      outputWriter.connect();

      sensorReaderFactory.getSensorReader().connect();

      Thread simulationThread = new Thread(robotController);
      simulationThread.start();

      if (USE_GUI)
      {
         DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
         URI rosURI = NetworkParameters.getROSURI();
         networkModuleParameters.setRosUri(rosURI);
         networkModuleParameters.enableUiModule(true);
         networkModuleParameters.enableRosModule(true);
         networkModuleParameters.enableLocalControllerCommunicator(true);
         new DRCNetworkProcessor(robotModel, networkModuleParameters);
      }
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

      controllerFactory.addHighLevelBehaviorFactory(new JointPositionControllerFactory(true));

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

   public static void main(String[] args)
   {
      new DRCSimGazeboControllerFactory();
   }
}
