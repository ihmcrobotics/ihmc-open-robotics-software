package us.ihmc.steppr.hardware.controllers;

import java.io.IOException;
import java.util.logging.Level;

import javax.xml.bind.JAXBException;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.acsell.hardware.AcsellAffinity;
import us.ihmc.acsell.hardware.AcsellSetup;
import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.steppr.hardware.output.StepprOutputWriter;
import us.ihmc.steppr.hardware.sensorReader.StepprSensorReaderFactory;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.tools.io.logging.LogTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputWriter;
import us.ihmc.wholeBodyController.DRCOutputWriterWithAccelerationIntegration;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingControllerFactory;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;

public class StepprControllerFactory
{
   private static final double gravity = -9.80; // From xsens

   //Use DATA_PRODUCER for ui control. Use VELOCITY_HEADING_COMPONENT for joystick control. Use YOVARIABLE for editing step variables by hand.
   private static final WalkingProvider walkingProvider = WalkingProvider.VELOCITY_HEADING_COMPONENT;

   private final PriorityParameters estimatorPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
   private final PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   private final PriorityParameters loggerPriority = new PriorityParameters(40);
   private final PriorityParameters poseCommunicatorPriority = new PriorityParameters(45);
   
   public StepprControllerFactory() throws IOException, JAXBException
   {

      /*
       * Create registries
       */
      AcsellAffinity stepprAffinity = new AcsellAffinity();
      BonoRobotModel robotModel = new BonoRobotModel(true, true);
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create network servers/clients
       */
      PacketCommunicator controllerPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicRealtimeThreadScheduler(loggerPriority), robotModel.getLogModelProvider(), robotModel.getLogSettings(),
            robotModel.getEstimatorDT());
      HumanoidGlobalDataProducer dataProducer = new HumanoidGlobalDataProducer(controllerPacketCommunicator);

      /*
       * Create controllers
       */
      MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, controllerPacketCommunicator, sensorInformation);

      /*
       * Create sensors
       */

      StepprSensorReaderFactory sensorReaderFactory = new StepprSensorReaderFactory(robotModel);

      /*
       * Create output writer
       */

      StepprOutputWriter stepprOutputWriter = new StepprOutputWriter(robotModel);
      controllerFactory.attachControllerStateChangedListener(stepprOutputWriter);
      controllerFactory.attachControllerFailureListener(stepprOutputWriter);
      DRCOutputWriter drcOutputWriter = stepprOutputWriter;
      
      boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = true;
      if (INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
      {
         DRCOutputWriterWithAccelerationIntegration stepprOutputWriterWithAccelerationIntegration = new DRCOutputWriterWithAccelerationIntegration(
               drcOutputWriter, new LegJointName[] { LegJointName.KNEE_PITCH, LegJointName.ANKLE_PITCH }, null, null, robotModel.getControllerDT(), true);
         stepprOutputWriterWithAccelerationIntegration.setAlphaDesiredVelocity(0.0, 0.0);
         stepprOutputWriterWithAccelerationIntegration.setAlphaDesiredPosition(0.0, 0.0);
         stepprOutputWriterWithAccelerationIntegration.setVelocityGains(0.0, 0.0);
         stepprOutputWriterWithAccelerationIntegration.setPositionGains(0.0, 0.0);
         
         drcOutputWriter = stepprOutputWriterWithAccelerationIntegration;
      }
      /*
       * Pelvis Pose Correction
       */
      PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber = new PelvisPoseCorrectionCommunicator(dataProducer);
      dataProducer.attachListener(StampedPosePacket.class, externalPelvisPoseSubscriber);

      /*
       * Build controller
       */
      ThreadDataSynchronizer threadDataSynchronizer = new ThreadDataSynchronizer(robotModel);
      DRCEstimatorThread estimatorThread = new DRCEstimatorThread(robotModel.getSensorInformation(), robotModel.getContactPointParameters(),
            robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer, new PeriodicRealtimeThreadScheduler(poseCommunicatorPriority), dataProducer, yoVariableServer, gravity);
      estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer,
            drcOutputWriter, dataProducer, yoVariableServer, gravity, robotModel.getEstimatorDT());

      MultiThreadedRealTimeRobotController robotController = new MultiThreadedRealTimeRobotController(estimatorThread);
      if (stepprAffinity.setAffinity())
      {
         robotController.addController(controllerThread, controllerPriority, stepprAffinity.getControlThreadProcessor());
      }
      else
      {
         robotController.addController(controllerThread, controllerPriority, null);
      }

      /*
       * Connect all servers
       */
      try
      {
         controllerPacketCommunicator.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      AcsellSetup stepprSetup = new AcsellSetup(yoVariableServer);

      //yoVariableServer.start();

      AcsellSetup.startStreamingData();
      stepprSetup.start();

      robotController.start();
      StepprRunner runner = new StepprRunner(estimatorPriority, sensorReaderFactory, robotController);
      runner.start();
      
      ThreadTools.sleep(2000);
      yoVariableServer.start();
      
      runner.join();

      System.exit(0);
   }

   private MomentumBasedControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, PacketCommunicator packetCommunicator,
         DRCRobotSensorInformation sensorInformation)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      final HighLevelState initialBehavior = HighLevelState.DO_NOTHING_BEHAVIOR; // HERE!!
      WalkingControllerParameters walkingControllerParamaters = robotModel.getWalkingControllerParameters();
      ArmControllerParameters armControllerParamaters = robotModel.getArmControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();
      ICPOptimizationParameters icpOptimizationParameters = robotModel.getICPOptimizationParameters();

      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, feetForceSensorNames,
            feetContactSensorNames, wristForceSensorNames , walkingControllerParamaters, armControllerParamaters, capturePointPlannerParameters, initialBehavior);
      controllerFactory.setICPOptimizationControllerParameters(icpOptimizationParameters);

      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList();
      humanoidJointPoseList.createPoseSettersJustLegs();
      
      boolean useArms = false;
      boolean robotIsHanging = true;
      
      DiagnosticsWhenHangingControllerFactory diagnosticsWhenHangingHighLevelBehaviorFactory = new DiagnosticsWhenHangingControllerFactory(humanoidJointPoseList, useArms, robotIsHanging, null);
      // Configure the MomentumBasedControllerFactory so we start with the diagnostic controller
      diagnosticsWhenHangingHighLevelBehaviorFactory.setTransitionRequested(true);
      controllerFactory.addHighLevelBehaviorFactory(diagnosticsWhenHangingHighLevelBehaviorFactory);
      controllerFactory.createControllerNetworkSubscriber(new PeriodicRealtimeThreadScheduler(poseCommunicatorPriority), packetCommunicator);
      
      if (walkingProvider == WalkingProvider.VELOCITY_HEADING_COMPONENT)
         controllerFactory.createComponentBasedFootstepDataMessageGenerator();

      return controllerFactory;
   }

   public static void main(String args[]) throws JSAPException, IOException, JAXBException
   {
      LogTools.setGlobalLogLevel(Level.CONFIG);
      new StepprControllerFactory();
   }
}
