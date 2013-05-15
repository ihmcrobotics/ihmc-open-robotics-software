package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.*;
import us.ihmc.commonWalkingControlModules.configurations.ManipulationControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final StateMachine<ManipulationState> stateMachine;

   private final HighLevelDirectControlManipulationState directControlManipulationState;
   private final State<ManipulationState> toroidManipulationState;
   private final DesiredHandPoseProvider handPoseProvider;
   private final TorusPoseProvider torusPoseProvider;

   public ManipulationControlModule(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                    ManipulationControllerParameters parameters, DesiredHandPoseProvider handPoseProvider, final TorusPoseProvider torusPoseProvider,
                                    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                    YoVariableRegistry parentRegistry)
   {
      stateMachine = new StateMachine<ManipulationState>("manipulationState", "manipulationStateSwitchTime", ManipulationState.class, yoTime, registry);
      this.handPoseProvider = handPoseProvider;
      this.torusPoseProvider = torusPoseProvider;

      SideDependentList<ReferenceFrame> handPositionControlFrames = new SideDependentList<ReferenceFrame>();
      SideDependentList<GeometricJacobian> jacobians = new SideDependentList<GeometricJacobian>();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody endEffector = fullRobotModel.getHand(robotSide);

         GeometricJacobian jacobian = new GeometricJacobian(fullRobotModel.getChest(), endEffector, endEffector.getBodyFixedFrame());
         jacobians.put(robotSide, jacobian);

         String frameName = endEffector.getName() + "PositionControlFrame";
         final ReferenceFrame frameAfterJoint = endEffector.getParentJoint().getFrameAfterJoint();
         ReferenceFrame handPositionControlFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, frameAfterJoint,
                                                      parameters.getHandControlFramesWithRespectToFrameAfterWrist().get(robotSide));
         handPositionControlFrames.put(robotSide, handPositionControlFrame);
      }

      directControlManipulationState = new HighLevelDirectControlManipulationState(yoTime, fullRobotModel, twistCalculator, parameters, handPoseProvider,
              dynamicGraphicObjectsListRegistry, handControllers, handPositionControlFrames, jacobians, momentumBasedController, registry);
      stateMachine.addState(directControlManipulationState);

      toroidManipulationState = new HighLevelToroidManipulationState(yoTime, fullRobotModel, twistCalculator, handPositionControlFrames, handControllers,
              jacobians, torusPoseProvider, momentumBasedController, dynamicGraphicObjectsListRegistry, parentRegistry);
      stateMachine.addState(toroidManipulationState);


      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return torusPoseProvider.checkForNewPose();
         }
      };

      StateTransition<ManipulationState> directControlToToroidManipulation = new StateTransition<ManipulationState>(toroidManipulationState.getStateEnum(),
                                                                                stateTransitionCondition);
      directControlManipulationState.addStateTransition(directControlToToroidManipulation);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      stateMachine.setCurrentState(directControlManipulationState.getStateEnum());
   }

   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }
}
