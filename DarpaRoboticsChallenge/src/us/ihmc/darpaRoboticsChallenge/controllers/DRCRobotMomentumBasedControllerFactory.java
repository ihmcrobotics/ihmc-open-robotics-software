package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.controllers.LidarControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.darpaRoboticsChallenge.sensors.WrenchBasedFootSwitch;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.gui.GUISetterUpperRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCRobotMomentumBasedControllerFactory implements ControllerFactory
{
   private final HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory;
   private final boolean useGazeboPhysics;

   public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory)
   {
      this(highLevelHumanoidControllerFactory, false);
   }

   public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory, boolean useGazeboPhysics)
   {
      this.highLevelHumanoidControllerFactory = highLevelHumanoidControllerFactory;
      this.useGazeboPhysics = useGazeboPhysics;
   }

   public RobotController getController(RigidBody estimationLink, ReferenceFrame estimationFrame, FullRobotModel fullRobotModel,
           CommonWalkingReferenceFrames referenceFrames, double controlDT, DoubleYoVariable yoTime,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GUISetterUpperRegistry guiSetterUpperRegistry, TwistCalculator twistCalculator,
           CenterOfMassJacobian centerOfMassJacobian, ForceSensorDataHolder forceSensorDataHolder, SideDependentList<HandControllerInterface> handControllers,
           LidarControllerInterface lidarControllerInterface, StateEstimationDataFromController stateEstimationDataFromControllerSink)
   {
      double footForward = DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD;
      double footBack = DRCRobotParameters.DRC_ROBOT_FOOT_BACK;
      double footWidth = DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH;

      YoVariableRegistry specificRegistry = new YoVariableRegistry("specific");


      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
      }

      SideDependentList<FootSwitchInterface> footSwitches = createFootSwitches(bipedFeet, forceSensorDataHolder, dynamicGraphicObjectsListRegistry, specificRegistry);

      double gravityZ = 9.81;

      RobotController highLevelHumanoidController = highLevelHumanoidControllerFactory.create(estimationLink, estimationFrame, fullRobotModel,
                                                               referenceFrames, null, yoTime, gravityZ, twistCalculator, centerOfMassJacobian, bipedFeet,
                                                               controlDT, footSwitches, handControllers, lidarControllerInterface,
                                                               stateEstimationDataFromControllerSink, dynamicGraphicObjectsListRegistry, specificRegistry,
                                                               guiSetterUpperRegistry, null);
      highLevelHumanoidController.getYoVariableRegistry().addChild(specificRegistry);


      return highLevelHumanoidController;
   }

   private SideDependentList<FootSwitchInterface> createFootSwitches(SideDependentList<ContactablePlaneBody> bipedFeet,
                                                                            ForceSensorDataHolder forceSensorDataHolder, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry registry)
   {
      SideDependentList<FootSwitchInterface> footSwitches = new SideDependentList<FootSwitchInterface>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ForceSensorData footForceSensor = forceSensorDataHolder.getByName(robotSide.getShortLowerCaseName() + "_leg_lax");
         WrenchBasedFootSwitch wrenchBasedFootSwitch = new WrenchBasedFootSwitch(footForceSensor, 0.02, bipedFeet.get(robotSide),
                                                          dynamicGraphicObjectsListRegistry, useGazeboPhysics, registry);
         footSwitches.put(robotSide, wrenchBasedFootSwitch);
      }

      return footSwitches;
   }

   public double getControlDT()
   {
      return 0.005;
   }

}
