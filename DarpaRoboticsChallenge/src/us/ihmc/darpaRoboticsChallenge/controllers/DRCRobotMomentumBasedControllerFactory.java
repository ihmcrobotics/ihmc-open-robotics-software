package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.RectangularContactableBody;
import us.ihmc.commonWalkingControlModules.controllers.ControllerFactory;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.remote.DataObjectServer;
import us.ihmc.utilities.remote.serialization.JointConfigurationDataSender;
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
   private final boolean setUpServer;

   public DRCRobotMomentumBasedControllerFactory(HighLevelHumanoidControllerFactory highLevelHumanoidControllerFactory, boolean setUpServer)
   {
      this.highLevelHumanoidControllerFactory = highLevelHumanoidControllerFactory;
      this.setUpServer = setUpServer;
   }

   public RobotController getController(FullRobotModel fullRobotModel, CommonWalkingReferenceFrames referenceFrames, double controlDT, DoubleYoVariable yoTime,
           DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, GUISetterUpperRegistry guiSetterUpperRegistry, TwistCalculator twistCalculator,
           CenterOfMassJacobian centerOfMassJacobian, SideDependentList<FootSwitchInterface> footSwitches, SideDependentList<HandControllerInterface> handControllers)
   {
      double footForward = DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD;
      double footBack = DRCRobotParameters.DRC_ROBOT_FOOT_BACK;
      double footWidth = DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH;

      YoVariableRegistry specificRegistry = new YoVariableRegistry("specific");

      if (setUpServer)
         createJointPositionServer(fullRobotModel);

      SideDependentList<ContactablePlaneBody> bipedFeet = new SideDependentList<ContactablePlaneBody>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody footBody = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         double left = footWidth / 2.0;
         double right = -footWidth / 2.0;

         ContactablePlaneBody foot = new RectangularContactableBody(footBody, soleFrame, footForward, -footBack, left, right);
         bipedFeet.put(robotSide, foot);
         
         specificRegistry.addChild(handControllers.get(robotSide).getYoVariableRegistry());
      }

      double gravityZ = 9.81;

      RobotController highLevelHumanoidController = highLevelHumanoidControllerFactory.create(fullRobotModel, referenceFrames, null, yoTime, gravityZ,
                                                       twistCalculator, centerOfMassJacobian, bipedFeet, controlDT, footSwitches, handControllers,
                                                       dynamicGraphicObjectsListRegistry, specificRegistry, guiSetterUpperRegistry, null);
      highLevelHumanoidController.getYoVariableRegistry().addChild(specificRegistry);


      return highLevelHumanoidController;
   }

   private void createJointPositionServer(FullRobotModel fullRobotModel)
   {
      int port = DRCConfigParameters.ROBOT_DATA_RECEIVER_PORT_NUMBER;
      long identifier = DRCConfigParameters.JOINT_DATA_IDENTIFIER;
      long updatePeriodInMilliseconds = DRCConfigParameters.ROBOT_JOINT_SERVER_UPDATE_MILLIS;
      DataObjectServer server = new DataObjectServer(port);
      JointConfigurationDataSender jointConfigurationDataSender = new JointConfigurationDataSender(identifier, fullRobotModel.getElevator());
      jointConfigurationDataSender.registerConsumer(server);
      jointConfigurationDataSender.startUpdateThread(updatePeriodInMilliseconds);
   }

   public int getSimulationTicksPerControlTick()
   {
      return 50;
   }
}
