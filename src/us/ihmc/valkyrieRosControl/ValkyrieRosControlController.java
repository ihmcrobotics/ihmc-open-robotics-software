package us.ihmc.valkyrieRosControl;

import java.io.IOException;
import java.util.HashMap;

import us.ihmc.affinity.Affinity;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ComponentBasedVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DataProducerVariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.YoVariableVariousWalkingProviderFactory;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.darpaRoboticsChallenge.DRCEstimatorThread;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.rosControl.JointHandle;
import us.ihmc.rosControl.valkyrie.ForceTorqueSensorHandle;
import us.ihmc.rosControl.valkyrie.IHMCValkyrieControlJavaBridge;
import us.ihmc.rosControl.valkyrie.IMUHandle;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.SettableTimestampProvider;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieSensorInformation;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputWriter;
import us.ihmc.wholeBodyController.DRCOutputWriterWithAccelerationIntegration;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingControllerFactory;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;

public class ValkyrieRosControlController extends IHMCValkyrieControlJavaBridge
{   
//   private static final String[] controlledJoints = { "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll",
//         "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw", "torsoPitch", "torsoRoll",
//         "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", "lowerNeckPitch",
//         "neckYaw", "upperNeckPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll",
//         "rightWristPitch" };
	private static final String[] controlledJoints = { "leftHipYaw",
			"leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch",
			"leftAnkleRoll", "rightHipYaw", "rightHipRoll", "rightHipPitch",
			"rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "torsoYaw",
			"torsoPitch", "torsoRoll", "leftShoulderPitch", "leftShoulderRoll",
			"leftShoulderYaw", "leftElbowPitch", "lowerNeckPitch", "neckYaw",
			"upperNeckPitch", "rightShoulderPitch", "rightShoulderRoll",
			"rightShoulderYaw", "rightElbowPitch" };
   
   private static final String[] readIMUs = { ValkyrieSensorInformation.getMiddlePelvisIMUSensor().replace("pelvis_", "") };
   
   private static final String[] readForceTorqueSensors = { "leftFootSixAxis", "rightFootSixAxis" };
   private static final String[] forceTorqueSensorModelNames = { "leftAnkleRoll", "rightAnkleRoll" };
   
   private static final double gravity = -9.792; //Tuned on Valkyrie 9.785; // Measured with IMUs on real robot
   
   private static final WalkingProvider walkingProvider = WalkingProvider.DATA_PRODUCER;

   private static final boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = false;
   
   private MultiThreadedRealTimeRobotController robotController;
   
   private final SettableTimestampProvider timestampProvider = new SettableTimestampProvider();
   
   private boolean firstTick = true;
   
   private final ValkyrieAffinity valkyrieAffinity = new ValkyrieAffinity();
   

//   private static final boolean AUTO_CALIBRATE_TORQUE_OFFSETS = false;

   public ValkyrieRosControlController()
   {
      
   }

   private DiagnosticsWhenHangingControllerFactory diagnosticControllerFactory = null;

   private MomentumBasedControllerFactory createDRCControllerFactory(ValkyrieRobotModel robotModel,
         HumanoidGlobalDataProducer dataProducer, DRCRobotSensorInformation sensorInformation)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      final HighLevelState initialBehavior = HighLevelState.DO_NOTHING_BEHAVIOR; // HERE!!
      WalkingControllerParameters walkingControllerParamaters = robotModel.getWalkingControllerParameters();
      ArmControllerParameters armControllerParamaters = robotModel.getArmControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForSlowWalkingOnRobot(walkingControllerParamaters);

      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParamaters, armControllerParamaters, capturePointPlannerParameters, initialBehavior);

      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList();
      humanoidJointPoseList.createPoseSetters();
      humanoidJointPoseList.createPoseSettersJustArms();
      humanoidJointPoseList.createPoseSettersTuneWaist();

      ValkyrieTorqueOffsetPrinter valkyrieTorqueOffsetPrinter = new ValkyrieTorqueOffsetPrinter();
      valkyrieTorqueOffsetPrinter.setRobotName(robotModel.getFullRobotName());
      diagnosticControllerFactory = new DiagnosticsWhenHangingControllerFactory(humanoidJointPoseList, true, true, valkyrieTorqueOffsetPrinter);
      diagnosticControllerFactory.setTransitionRequested(true);
      controllerFactory.addHighLevelBehaviorFactory(diagnosticControllerFactory);

      VariousWalkingProviderFactory variousWalkingProviderFactory;
      switch(walkingProvider)
      {
      case YOVARIABLE:
         variousWalkingProviderFactory = new YoVariableVariousWalkingProviderFactory();
         break;
      case DATA_PRODUCER:
         variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters, new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY));
         break;
      case VELOCITY_HEADING_COMPONENT:
         variousWalkingProviderFactory = new ComponentBasedVariousWalkingProviderFactory(false, null, robotModel.getControllerDT());
         break;
      default:
         throw new RuntimeException("Invalid walking provider.");
      }

      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      return controllerFactory;
   }

   @Override
   protected void init()
   {
      
      long maxMemory = Runtime.getRuntime().maxMemory();
      
      System.out.println("Partying hard with max memory of: " + maxMemory);
      /*
       * Create joints
       */
      
      HashMap<String, JointHandle> jointHandles = new HashMap<>();
      for(String joint : controlledJoints)
      {
         jointHandles.put(joint, createJointHandle(joint));
      }
      
      HashMap<String, IMUHandle> imuHandles = new HashMap<>();
      for(String imu : readIMUs)
      {
         imuHandles.put(imu, createIMUHandle(imu));
      }
      
      HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles = new HashMap<>();
      for(int i = 0; i < readForceTorqueSensors.length; i++)
      {
    	  
    	 String forceTorqueSensor = readForceTorqueSensors[i];
    	 String modelName = forceTorqueSensorModelNames[i];
         forceTorqueSensorHandles.put(modelName, createForceTorqueSensorHandle(forceTorqueSensor));
      }
      
      /*
       * Create registries
       */

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.REAL_ROBOT, true);
      DRCRobotSensorInformation sensorInformation = (ValkyrieSensorInformation) robotModel.getSensorInformation();
      
      /*
       * Create network servers/clients
       */
      PacketCommunicator drcNetworkProcessorServer = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.LOGGER_PRIORITY), robotModel.getLogModelProvider(), robotModel.getLogSettings(), robotModel.getEstimatorDT());
      HumanoidGlobalDataProducer dataProducer = new HumanoidGlobalDataProducer(drcNetworkProcessorServer);
      
      /*
       * Create sensors
       */

      StateEstimatorParameters stateEstimatorParameters = robotModel.getStateEstimatorParameters();

      ValkyrieRosControlSensorReaderFactory sensorReaderFactory = new ValkyrieRosControlSensorReaderFactory(timestampProvider, stateEstimatorParameters, jointHandles, imuHandles, forceTorqueSensorHandles, robotModel.getSensorInformation());
      
      /*
       * Create controllers
       */
      MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, dataProducer, sensorInformation);
      
      /*
       * Create output writer
       */
      ValkyrieRosControlOutputWriter valkyrieOutputWriter = new ValkyrieRosControlOutputWriter(robotModel);
      DRCOutputWriter drcOutputWriter = valkyrieOutputWriter;
      
      if (INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
      {         
         DRCOutputWriterWithAccelerationIntegration valkyrieOutputWriterWithAccelerationIntegration = new DRCOutputWriterWithAccelerationIntegration(drcOutputWriter, robotModel.getControllerDT(), true);

         valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredVelocity(0.85, 0.85);
         valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredPosition(0.0, 0.0);
         valkyrieOutputWriterWithAccelerationIntegration.setVelocityGains(9.0, 0.0);
         valkyrieOutputWriterWithAccelerationIntegration.setPositionGains(0.0, 0.0);
         
         drcOutputWriter = valkyrieOutputWriterWithAccelerationIntegration;
      }

      PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = null;
      externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(null);
      dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

      /*
       * Build controller
       */
      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSensorInformation(), robotModel.getContactPointParameters(), robotModel.getStateEstimatorParameters(),
           sensorReaderFactory, threadDataSynchronizer, new PeriodicRealtimeThreadScheduler(ValkyriePriorityParameters.POSECOMMUNICATOR_PRIORITY), dataProducer, yoVariableServer, gravity);
      estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer, drcOutputWriter, dataProducer,
            yoVariableServer, gravity, robotModel.getEstimatorDT());

      if (diagnosticControllerFactory != null)
         diagnosticControllerFactory.attachJointTorqueOffsetProcessor(sensorReaderFactory.getSensorReader());
      
      /*
       * Connect all servers
       */
      try
      {
         drcNetworkProcessorServer.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
      
      yoVariableServer.start();
      
      robotController = new MultiThreadedRealTimeRobotController(estimatorThread);
      if(valkyrieAffinity.setAffinity())
      {
         robotController.addController(controllerThread, ValkyriePriorityParameters.CONTROLLER_PRIORITY, valkyrieAffinity.getControlThreadProcessor());
      }
      else
      {
         robotController.addController(controllerThread, ValkyriePriorityParameters.CONTROLLER_PRIORITY, null);
      }

      robotController.start();
   }

   @Override
   protected void doControl(long time, long duration)
   {
      if(firstTick)
      {
         if(valkyrieAffinity.setAffinity())
         {
            System.out.println("Setting estimator thread affinity to processor " + valkyrieAffinity.getEstimatorThreadProcessor().getId());
            Affinity.setAffinity(valkyrieAffinity.getEstimatorThreadProcessor());
         }
         firstTick = false;
      }
      
      
      timestampProvider.setTimestamp(time);
      robotController.read();
   }
}
