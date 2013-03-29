package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.GazeboStateCommunicator.GazeboRobot;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReaderAndWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.HandStatePacket;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.referenceFrames.ReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaElipsoidsVisualizer;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.handControl.SandiaHandModel;
import us.ihmc.darpaRoboticsChallenge.handControl.SimulatedUnderactuatedSandiaHandController;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCPerfectPoseEstimator;
import us.ihmc.projectM.R2Sim02.initialSetup.GuiInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.ScsInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.KryoObjectServer;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.InverseDynamicsMechanismReferenceFrameVisualizer;
import com.yobotics.simulationconstructionset.PlaybackListener;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.AbstractModularRobotController;
import com.yobotics.simulationconstructionset.robotController.DelayedThreadedModularRobotController;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.robotController.ModularSensorProcessor;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCSimulationFactory
{
   private static final boolean SHOW_REFERENCE_FRAMES = false;
   public static boolean SHOW_INERTIA_ELLIPSOIDS = false;

   public static HumanoidRobotSimulation<SDFRobot> createSimulation(ControllerFactory controllerFactory,
         CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, DRCRobotInterface robotInterface, RobotInitialSetup<SDFRobot> robotInitialSetup,
         ScsInitialSetup scsInitialSetup, GuiInitialSetup guiInitialSetup, KryoObjectServer networkServer, ObjectCommunicator networkProccesorCommunicator)
   {
      GUISetterUpperRegistry guiSetterUpperRegistry = new GUISetterUpperRegistry();
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      DRCRobotJointMap jointMap = robotInterface.getJointMap();

      double simulateDT = robotInterface.getSimulateDT();
      double controlDT = controllerFactory.getControlDT();
      int simulationTicksPerControlTick = (int) (controlDT / simulateDT);

      SDFRobot simulatedRobot = robotInterface.getRobot();


      simulatedRobot.setDynamicIntegrationMethod(scsInitialSetup.getDynamicIntegrationMethod());

      SDFFullRobotModel fullRobotModelForSimulation = robotInterface.getFullRobotModel();


      SDFFullRobotModel fullRobotModelForController = fullRobotModelForSimulation;
      CommonWalkingReferenceFrames referenceFramesForController = new ReferenceFrames(fullRobotModelForSimulation, jointMap, jointMap.getAnkleHeight());

      SideDependentList<FootSwitchInterface> footSwitches = robotInterface.getFootSwitches();

      SideDependentList<HandControllerInterface> handControllers = null;
      if (jointMap.getSelectedModel() == DRCRobotModel.ATLAS_SANDIA_HANDS)
      {
         handControllers = new SideDependentList<HandControllerInterface>();
         for (RobotSide robotSide : RobotSide.values())
         {

            SandiaHandModel handModel = new SandiaHandModel(fullRobotModelForController, robotSide);
            SimulatedUnderactuatedSandiaHandController simulatedUnderactuatedSandiaHandController = new SimulatedUnderactuatedSandiaHandController(simulatedRobot.getYoTime(), handModel);
            handControllers.put(robotSide, simulatedUnderactuatedSandiaHandController);
            if (networkServer != null)
            {
               networkServer.attachListener(HandStatePacket.class, simulatedUnderactuatedSandiaHandController);
            }

         }
      }

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModelForController.getElevator());
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModelForController.getElevator());

      SDFPerfectSimulatedSensorReaderAndWriter sensorReaderAndOutputWriter = new SDFPerfectSimulatedSensorReaderAndWriter(simulatedRobot, fullRobotModelForController, referenceFramesForController, DRCConfigParameters.INTRODUCE_FILTERED_GAUSSIAN_POSITIONING_ERROR);
      sensorReaderAndOutputWriter.setNoiseFilterAlpha(DRCConfigParameters.NOISE_FILTER_ALPHA);
      sensorReaderAndOutputWriter.setPositionNoiseStd(DRCConfigParameters.POSITION_NOISE_STD);
      sensorReaderAndOutputWriter.setQuaternionNoiseStd(DRCConfigParameters.QUATERNION_NOISE_STD);


      OneDoFJoint lidarJoint = fullRobotModelForController.getOneDoFJointByName(jointMap.getLidarJointName());
      // PathTODO: Build LIDAR here
      RobotController robotController = controllerFactory.getController(fullRobotModelForController, referenceFramesForController, controlDT, simulatedRobot.getYoTime(), dynamicGraphicObjectsListRegistry, guiSetterUpperRegistry, twistCalculator, centerOfMassJacobian, footSwitches, handControllers, lidarJoint);

      AbstractModularRobotController modularRobotController;


      if (robotInterface.simulateDelay())
      {
         System.err.println("INFO: Creating controller with simulated delay");
         modularRobotController = new DelayedThreadedModularRobotController("ModularRobotController");
      }
      else
      {
         modularRobotController = new ModularRobotController("ModularRobotController");
      }

      final ModularSensorProcessor sensorProcessor = createSensorProcessor(twistCalculator, centerOfMassJacobian);
      modularRobotController.setRawSensorReader(sensorReaderAndOutputWriter);
      modularRobotController.setSensorProcessor(sensorProcessor);
      modularRobotController.addRobotController(robotController);

      if (SHOW_INERTIA_ELLIPSOIDS)
      {
         modularRobotController.addRobotController(new CommonInertiaElipsoidsVisualizer(fullRobotModelForSimulation.getElevator(), dynamicGraphicObjectsListRegistry));
      }

      if (SHOW_REFERENCE_FRAMES)
      {
         modularRobotController.addRobotController(new InverseDynamicsMechanismReferenceFrameVisualizer(fullRobotModelForSimulation.getElevator(), dynamicGraphicObjectsListRegistry, 0.1));
      }

      modularRobotController.setRawOutputWriter(sensorReaderAndOutputWriter);

      final HumanoidRobotSimulation<SDFRobot> humanoidRobotSimulation = new HumanoidRobotSimulation<SDFRobot>(simulatedRobot, modularRobotController, simulationTicksPerControlTick, fullRobotModelForSimulation, commonAvatarEnvironmentInterface, simulatedRobot.getAllExternalForcePoints(), robotInitialSetup, scsInitialSetup, guiInitialSetup, guiSetterUpperRegistry, dynamicGraphicObjectsListRegistry);
      if (networkServer != null)
      {
         DRCLidar.setupDRCRobotLidar(humanoidRobotSimulation, networkServer, lidarJoint);
         setUpRemoteSCSListening(humanoidRobotSimulation, networkServer);
      }

      if (simulatedRobot instanceof GazeboRobot)
      {
         ((GazeboRobot) simulatedRobot).registerWithSCS(humanoidRobotSimulation.getSimulationConstructionSet());
      }      
      
      
      if(networkProccesorCommunicator != null)
      {
         sensorProcessor.addSensorProcessor(new DRCPerfectPoseEstimator(simulatedRobot, networkProccesorCommunicator, robotInterface.getTimeStampProvider()));
      }
      return humanoidRobotSimulation;
   }

   private static void setUpRemoteSCSListening(HumanoidRobotSimulation<SDFRobot> humanoidRobotSimulation, KryoObjectServer networkServer)
   {
      PlaybackListener playbackListener = new SCSPlaybackListener(networkServer);
      humanoidRobotSimulation.getSimulationConstructionSet().attachPlaybackListener(playbackListener);
   }

   private static ModularSensorProcessor createSensorProcessor(TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian)
   {
      ModularSensorProcessor modularSensorProcessor = new ModularSensorProcessor("ModularSensorProcessor", "");
      modularSensorProcessor.addSensorProcessor(new TwistUpdater(twistCalculator));
      modularSensorProcessor.addSensorProcessor(new CenterOfMassJacobianUpdater(centerOfMassJacobian));
      return modularSensorProcessor;
   }
}
