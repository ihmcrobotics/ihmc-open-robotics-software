package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFPerfectSimulatedSensorReaderAndWriter;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.CenterOfMassJacobianUpdater;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.commonWalkingControlModules.sensors.TwistUpdater;
import us.ihmc.darpaRoboticsChallenge.sensors.PerfectFootswitch;
import us.ihmc.projectM.R2Sim02.initialSetup.GuiInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;
import us.ihmc.projectM.R2Sim02.initialSetup.ScsInitialSetup;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.ModularRobotController;
import com.yobotics.simulationconstructionset.robotController.ModularSensorProcessor;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.robotController.SensorProcessor;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCSimulationFactory
{
   public static HumanoidRobotSimulation<SDFRobot> createSimulation(ControllerFactory controllerFactory,
           CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, RobotInitialSetup<SDFRobot> robotInitialSetup, ScsInitialSetup scsInitialSetup,
           GuiInitialSetup<SDFRobot> guiInitialSetup)
   {
      GUISetterUpperRegistry guiSetterUpperRegistry = new GUISetterUpperRegistry();
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      double simulateDT = scsInitialSetup.getDT();
      int simulationTicksPerControlTick = controllerFactory.getSimulationTicksPerControlTick();
      double controlDT = simulateDT * simulationTicksPerControlTick;

      JaxbSDFLoader jaxbSDFLoader = DRCRobotSDFLoader.loadDRCRobot();
      SDFRobot robot = jaxbSDFLoader.getRobot();
      FullRobotModel fullRobotModel = jaxbSDFLoader.getFullRobotModel();
      CommonWalkingReferenceFrames referenceFrames = jaxbSDFLoader.getReferenceFrames();

      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();
      for (RobotSide robotSide : RobotSide.values())
      {
         footSwitches.put(robotSide, new PerfectFootswitch(robot, robotSide));
      }

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());

      SDFPerfectSimulatedSensorReaderAndWriter sensorReaderAndOutputWriter = new SDFPerfectSimulatedSensorReaderAndWriter(robot, fullRobotModel,
                                                                                referenceFrames);

      RobotController robotController = controllerFactory.getController(fullRobotModel, referenceFrames, controlDT, robot.getYoTime(),
                                           dynamicGraphicObjectsListRegistry, guiSetterUpperRegistry, twistCalculator, centerOfMassJacobian, footSwitches);

      ModularRobotController modularRobotController = new ModularRobotController("ModularRobotController");
      modularRobotController.setRawSensorReader(sensorReaderAndOutputWriter);
      modularRobotController.setSensorProcessor(createSensorProcessor(twistCalculator, centerOfMassJacobian));
      modularRobotController.addRobotController(robotController);
      modularRobotController.setRawOutputWriter(sensorReaderAndOutputWriter);

      return new HumanoidRobotSimulation<SDFRobot>(robot, modularRobotController, simulationTicksPerControlTick, fullRobotModel, commonAvatarEnvironmentInterface,
                                         robotInitialSetup, scsInitialSetup, guiInitialSetup, guiSetterUpperRegistry, dynamicGraphicObjectsListRegistry);
   }

   private static SensorProcessor createSensorProcessor(TwistCalculator twistCalculator, CenterOfMassJacobian centerOfMassJacobian)
   {
      ModularSensorProcessor modularSensorProcessor = new ModularSensorProcessor("ModularSensorProcessor", "");
      modularSensorProcessor.addSensorProcessor(new TwistUpdater(twistCalculator));
      modularSensorProcessor.addSensorProcessor(new CenterOfMassJacobianUpdater(centerOfMassJacobian));

      return modularSensorProcessor;
   }
}
