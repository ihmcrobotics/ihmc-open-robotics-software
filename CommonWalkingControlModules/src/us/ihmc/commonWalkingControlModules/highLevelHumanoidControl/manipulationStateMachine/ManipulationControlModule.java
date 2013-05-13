package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.sensors.ManipulableToroidUpdater;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import java.util.Map;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<IndividualHandControlStateMachine> individualHandControlStateMachines =
      new SideDependentList<IndividualHandControlStateMachine>();
   private final SideDependentList<GeometricJacobian> jacobians = new SideDependentList<GeometricJacobian>();
   private final ManipulableToroidUpdater manipulableToroidUpdater;

   public ManipulationControlModule(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                    WalkingControllerParameters walkingControllerParameters, DesiredHandPoseProvider handPoseProvider,
                                    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                    YoVariableRegistry parentRegistry)
   {
      double controlDT = momentumBasedController.getControlDT();
      double gravityZ = momentumBasedController.getGravityZ();
      SideDependentList<ReferenceFrame> handPositionControlFrames = new SideDependentList<ReferenceFrame>();

      for (RobotSide robotSide : RobotSide.values())
      {
         HandControllerInterface handControllerInterface = null;
         if (handControllers != null)
         {
            handControllerInterface = handControllers.get(robotSide);
         }

         RigidBody endEffector = fullRobotModel.getHand(robotSide);

         String frameName = endEffector.getName() + "PositionControlFrame";
         final ReferenceFrame frameAfterJoint = endEffector.getParentJoint().getFrameAfterJoint();
         ReferenceFrame handPositionControlFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, frameAfterJoint,
               walkingControllerParameters.getHandControlFramesWithRespectToFrameAfterWrist().get(robotSide));
         handPositionControlFrames.put(robotSide, handPositionControlFrame);

         // TODO: create manipulationControlParameters, have current walking parameters class implement it

         GeometricJacobian jacobian = new GeometricJacobian(fullRobotModel.getChest(), endEffector, endEffector.getBodyFixedFrame());
         jacobians.put(robotSide, jacobian);

         Map<OneDoFJoint, Double> defaultArmJointPositions = walkingControllerParameters.getDefaultArmJointPositions(fullRobotModel, robotSide);
         Map<OneDoFJoint, Double> minTaskSpacePositions = walkingControllerParameters.getMinTaskspaceArmJointPositions(fullRobotModel, robotSide);
         Map<OneDoFJoint, Double> maxTaskSpacePositions = walkingControllerParameters.getMaxTaskspaceArmJointPositions(fullRobotModel, robotSide);

         individualHandControlStateMachines.put(robotSide,
                 new IndividualHandControlStateMachine(yoTime, robotSide, fullRobotModel, twistCalculator, handPositionControlFrame, handPoseProvider,
                    dynamicGraphicObjectsListRegistry, handControllerInterface, gravityZ, controlDT, momentumBasedController, jacobian,
                    defaultArmJointPositions, minTaskSpacePositions, maxTaskSpacePositions, registry));
      }

      this.manipulableToroidUpdater = new ManipulableToroidUpdater(fullRobotModel, handPositionControlFrames, dynamicGraphicObjectsListRegistry, yoTime,
              controlDT, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      for (IndividualHandControlStateMachine stateMachine : individualHandControlStateMachines)
      {
         stateMachine.initialize();
      }
   }

   public void doControl()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         jacobians.get(robotSide).compute();
         individualHandControlStateMachines.get(robotSide).doControl();
      }
      manipulableToroidUpdater.update();
   }
}
