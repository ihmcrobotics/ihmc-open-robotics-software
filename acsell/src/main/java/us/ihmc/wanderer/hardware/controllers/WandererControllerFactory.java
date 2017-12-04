package us.ihmc.wanderer.hardware.controllers;

import java.io.IOException;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

import javax.xml.bind.JAXBException;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.acsell.CostOfTransportCalculator;
import us.ihmc.acsell.hardware.AcsellAffinity;
import us.ihmc.acsell.hardware.AcsellSetup;
import us.ihmc.avatar.DRCEstimatorThread;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.*;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.packets.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicator;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.util.PeriodicRealtimeThreadScheduler;
import us.ihmc.util.PeriodicRealtimeThreadSchedulerFactory;
import us.ihmc.wanderer.hardware.output.WandererOutputWriter;
import us.ihmc.wanderer.hardware.sensorReader.WandererSensorReaderFactory;
import us.ihmc.wanderer.parameters.WandererRobotModel;
import us.ihmc.wholeBodyController.DRCControllerThread;
import us.ihmc.wholeBodyController.DRCOutputProcessor;
import us.ihmc.wholeBodyController.DRCOutputProcessorWithAccelerationIntegration;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizer;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingControllerStateFactory;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.*;

public class WandererControllerFactory
{
   private static final double gravity = -9.80; // From xsens

   //Use DATA_PRODUCER for ui control. Use VELOCITY_HEADING_COMPONENT for joystick control. Use YOVARIABLE for editing step variables by hand.
   private static final WalkingProvider walkingProvider = WalkingProvider.VELOCITY_HEADING_COMPONENT;

   private final PriorityParameters estimatorPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 1);
   private final PriorityParameters controllerPriority = new PriorityParameters(PriorityParameters.getMaximumPriority() - 5);
   private final PriorityParameters loggerPriority = new PriorityParameters(40);
   private final PriorityParameters poseCommunicatorPriority = new PriorityParameters(45);

   public WandererControllerFactory() throws IOException, JAXBException
   {

      /*
       * Create registries
       */
      AcsellAffinity wandererAffinity = new AcsellAffinity();
      WandererRobotModel robotModel = new WandererRobotModel(true, true);
      DRCRobotSensorInformation sensorInformation = robotModel.getSensorInformation();

      /*
       * Create network servers/clients
       */
      PacketCommunicator controllerPacketCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.CONTROLLER_PORT, new IHMCCommunicationKryoNetClassList());
      YoVariableServer yoVariableServer = new YoVariableServer(getClass(), new PeriodicRealtimeThreadSchedulerFactory(loggerPriority), robotModel.getLogModelProvider(), robotModel.getLogSettings(),
            robotModel.getEstimatorDT());
      HumanoidGlobalDataProducer dataProducer = new HumanoidGlobalDataProducer(controllerPacketCommunicator);

      /*
       * Create cost of transport calculator
       */

      double mass = 91.100;
      CostOfTransportCalculator costOfTransportCalculator = new CostOfTransportCalculator(mass, gravity, 10.0, robotModel.getEstimatorDT(), yoVariableServer);

      /*
       * Create controllers
       */
      HighLevelHumanoidControllerFactory controllerFactory = createDRCControllerFactory(robotModel, controllerPacketCommunicator, sensorInformation);

      /*
       * Create sensors
       */

      WandererSensorReaderFactory sensorReaderFactory = new WandererSensorReaderFactory(robotModel, costOfTransportCalculator);

      /*
       * Create output writer
       */

      WandererOutputWriter wandererOutputWriter = new WandererOutputWriter(robotModel);
      controllerFactory.attachControllerStateChangedListener(wandererOutputWriter);
      controllerFactory.attachControllerFailureListener(wandererOutputWriter);
      DRCOutputProcessor drcOutputWriter = wandererOutputWriter;

      boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = true;
      if (INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES)
      {
         DRCOutputProcessorWithAccelerationIntegration wandererOutputWriterWithAccelerationIntegration = new DRCOutputProcessorWithAccelerationIntegration(
               drcOutputWriter, new LegJointName[] { LegJointName.KNEE_PITCH, LegJointName.ANKLE_PITCH }, null, null, robotModel.getControllerDT(), true);
         wandererOutputWriterWithAccelerationIntegration.setAlphaDesiredVelocity(0.0, 0.0);
         wandererOutputWriterWithAccelerationIntegration.setAlphaDesiredPosition(0.0, 0.0);
         wandererOutputWriterWithAccelerationIntegration.setVelocityGains(0.0, 0.0);
         wandererOutputWriterWithAccelerationIntegration.setPositionGains(0.0, 0.0);

         drcOutputWriter = wandererOutputWriterWithAccelerationIntegration;
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
            robotModel, robotModel.getStateEstimatorParameters(), sensorReaderFactory, threadDataSynchronizer, new PeriodicRealtimeThreadScheduler(poseCommunicatorPriority), dataProducer, null, costOfTransportCalculator, gravity);
      estimatorThread.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
      DRCControllerThread controllerThread = new DRCControllerThread(robotModel, robotModel.getSensorInformation(), controllerFactory, threadDataSynchronizer,
            drcOutputWriter, dataProducer, yoVariableServer, gravity, robotModel.getEstimatorDT());
//      WandererOutputProcessor outputProcessor = new WandererOutputProcessor(threadDataSynchronizer.getControllerFullRobotModel());
//      controllerThread.addOutputProcessorToController(outputProcessor);

      MultiThreadedRealTimeRobotController robotController = new MultiThreadedRealTimeRobotController(estimatorThread);
      if (wandererAffinity.setAffinity())
      {
         robotController.addController(controllerThread, controllerPriority, wandererAffinity.getControlThreadProcessor());
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

      AcsellSetup wandererSetup = new AcsellSetup(yoVariableServer);


      AcsellSetup.startStreamingData();
      wandererSetup.start();

      robotController.start();
      WandererRunner runner = new WandererRunner(estimatorPriority, sensorReaderFactory, robotController);
      runner.start();

      ThreadTools.sleep(2000);
      yoVariableServer.start();

      runner.join();

      System.exit(0);
   }

   private HighLevelHumanoidControllerFactory createDRCControllerFactory(DRCRobotModel robotModel, PacketCommunicator packetCommunicator,
         DRCRobotSensorInformation sensorInformation)
   {
      ContactableBodiesFactory contactableBodiesFactory = robotModel.getContactPointParameters().getContactableBodiesFactory();

      HighLevelControllerParameters highLevelControllerParameters = robotModel.getHighLevelControllerParameters();
      WalkingControllerParameters walkingControllerParamaters = robotModel.getWalkingControllerParameters();
      ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters = robotModel.getCapturePointPlannerParameters();

      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory, feetForceSensorNames,
                                                                                                    feetContactSensorNames, wristForceSensorNames, highLevelControllerParameters,
                                                                                                    walkingControllerParamaters, capturePointPlannerParameters);
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();

      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);

      HighLevelControllerName fallbackControllerState = highLevelControllerParameters.getFallbackControllerState();
      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, fallbackControllerState);
      controllerFactory.addControllerFailureTransition(WALKING, fallbackControllerState);
      controllerFactory.setInitialState(highLevelControllerParameters.getDefaultInitialControllerState());

      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList();
      humanoidJointPoseList.createPoseSettersJustLegs();

      boolean useArms = false;
      boolean robotIsHanging = true;

      DiagnosticsWhenHangingControllerStateFactory diagnosticsWhenHangingHighLevelBehaviorFactory = new DiagnosticsWhenHangingControllerStateFactory(humanoidJointPoseList, useArms, robotIsHanging, null);
      // Configure the HighLevelHumanoidControllerFactory so we start with the diagnostic controller
      diagnosticsWhenHangingHighLevelBehaviorFactory.setTransitionRequested(true);
      controllerFactory.addCustomControlState(diagnosticsWhenHangingHighLevelBehaviorFactory);
      controllerFactory.createControllerNetworkSubscriber(new PeriodicRealtimeThreadScheduler(poseCommunicatorPriority), packetCommunicator);

      if (walkingProvider == WalkingProvider.VELOCITY_HEADING_COMPONENT)
         controllerFactory.createComponentBasedFootstepDataMessageGenerator();

      return controllerFactory;
   }

   public static void main(String args[]) throws JSAPException, IOException, JAXBException
   {
      Logger.getLogger("").setLevel(Level.CONFIG);
      ConsoleHandler handler = new ConsoleHandler();
      handler.setLevel(Level.CONFIG);
      Logger.getLogger("").addHandler(handler);
      new WandererControllerFactory();
   }
}
