package us.ihmc.steppr.hardware.controllers;

import java.io.IOException;
import java.util.logging.Level;

import javax.xml.bind.JAXBException;

import us.ihmc.acsell.parameters.BonoRobotModel;
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
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.steppr.hardware.StepprAffinity;
import us.ihmc.steppr.hardware.StepprSetup;
import us.ihmc.steppr.hardware.output.StepprOutputWriter;
import us.ihmc.steppr.hardware.sensorReader.StepprSensorReaderFactory;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.io.logging.LogTools;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputWriter;
import us.ihmc.wholeBodyController.DRCOutputWriterWithAccelerationIntegration;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingControllerFactory;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;

import com.martiansoftware.jsap.JSAPException;

public class StepprControllerFactory
{
   private static final double gravity = -9.80; // From xsens

   //Use DATA_PRODUCER for ui control. Use VELOCITY_HEADING_COMPONENT for joystick control. Use YOVARIABLE for editing step variables by hand.
   private static final WalkingProvider walkingProvider = WalkingProvider.VELOCITY_HEADING_COMPONENT;

   public StepprControllerFactory() throws IOException, JAXBException
   {

      /*
       * Create registries
       */
      StepprAffinity stepprAffinity = new StepprAffinity();
      PriorityParameters estimatorPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
      PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);

      BonoRobotModel robotModel = new BonoRobotModel(true, true);
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create network servers/clients
       */
      PacketCommunicator drcNetworkProcessorServer = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), robotModel.getLogModelProvider(), robotModel.getLogSettings(),
            robotModel.getEstimatorDT());
      GlobalDataProducer dataProducer = new GlobalDataProducer(drcNetworkProcessorServer);

      /*
       * Create controllers
       */
      MomentumBasedControllerFactory controllerFactory = createDRCControllerFactory(robotModel, dataProducer, sensorInformation);

      /*
       * Create sensors
       */

      StepprSensorReaderFactory sensorReaderFactory = new StepprSensorReaderFactory(robotModel);

      /*
       * Create output writer
       */

      StepprOutputWriter stepprOutputWriter = new StepprOutputWriter(robotModel);
      controllerFactory.attachControllerStateChangedListener(stepprOutputWriter);
      DRCOutputWriter drcOutputWriter = stepprOutputWriter;
      
      boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = true;
      if (INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
      {
         DRCOutputWriterWithAccelerationIntegration stepprOutputWriterWithAccelerationIntegration = new DRCOutputWriterWithAccelerationIntegration(
               drcOutputWriter, new LegJointName[] { LegJointName.KNEE, LegJointName.ANKLE_PITCH }, null, null, robotModel.getControllerDT(), true);
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
            robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer, dataProducer, yoVariableServer, gravity);
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
         drcNetworkProcessorServer.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      StepprSetup stepprSetup = new StepprSetup(yoVariableServer);

      yoVariableServer.start();

      StepprSetup.startStreamingData();
      stepprSetup.start();

      robotController.start();
      StepprRunner runner = new StepprRunner(estimatorPriority, sensorReaderFactory, robotController);
      runner.start();
      runner.join();

      System.exit(0);
   }

   private MomentumBasedControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, GlobalDataProducer dataProducer,
         DRCRobotSensorInformation sensorInformation)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      final HighLevelState initialBehavior = HighLevelState.DO_NOTHING_BEHAVIOR; // HERE!!
      WalkingControllerParameters walkingControllerParamaters = robotModel.getWalkingControllerParameters();
      ArmControllerParameters armControllerParamaters = robotModel.getArmControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      FootstepTimingParameters footstepTimingParameters = FootstepTimingParameters.createForSlowWalkingOnRobot(walkingControllerParamaters);

      MomentumBasedControllerFactory controllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory,
            sensorInformation.getFeetForceSensorNames(), sensorInformation.getFeetContactSensorNames(),
            walkingControllerParamaters, armControllerParamaters, capturePointPlannerParameters, initialBehavior);

      double kneeAngleMultiplicationFactor = -1.0;
      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList(kneeAngleMultiplicationFactor);
      humanoidJointPoseList.createPoseSettersJustLegs();
      
      boolean useArms = false;
      boolean robotIsHanging = true;
      
      DiagnosticsWhenHangingControllerFactory diagnosticsWhenHangingHighLevelBehaviorFactory = new DiagnosticsWhenHangingControllerFactory(humanoidJointPoseList, useArms, robotIsHanging);
      // Configure the MomentumBasedControllerFactory so we start with the diagnostic controller
      diagnosticsWhenHangingHighLevelBehaviorFactory.setTransitionRequested(true);
      controllerFactory.addHighLevelBehaviorFactory(diagnosticsWhenHangingHighLevelBehaviorFactory);
      
      VariousWalkingProviderFactory variousWalkingProviderFactory;
      switch(walkingProvider)
      {
         case YOVARIABLE:
            variousWalkingProviderFactory = new YoVariableVariousWalkingProviderFactory();
            break;
         case DATA_PRODUCER:
            variousWalkingProviderFactory = new DataProducerVariousWalkingProviderFactory(dataProducer, footstepTimingParameters);
            break;
         case VELOCITY_HEADING_COMPONENT:
            variousWalkingProviderFactory = new ComponentBasedVariousWalkingProviderFactory(false, null, robotModel.getControllerDT());
            break;
         default:
               throw new RuntimeException("no such walkingProvider");
      }
      controllerFactory.setVariousWalkingProviderFactory(variousWalkingProviderFactory);
      return controllerFactory;
   }

   public static void main(String args[]) throws JSAPException, IOException, JAXBException
   {
      LogTools.setGlobalLogLevel(Level.CONFIG);
      new StepprControllerFactory();
   }
}
