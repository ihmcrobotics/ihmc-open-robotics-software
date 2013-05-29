package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;
import us.ihmc.commonWalkingControlModules.configurations.ManipulationControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.HandControllerInterface;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.VariousWalkingProviders;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.direct.HighLevelDirectControlManipulationState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.fingerToroidManipulation.HighLevelFingerToroidManipulationState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import java.util.ArrayList;
import java.util.List;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StateMachine<ManipulationState> stateMachine;
   private final List<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();
   private HighLevelDirectControlManipulationState directControlManipulationState;

   public ManipulationControlModule(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                    ManipulationControllerParameters parameters, final VariousWalkingProviders variousWalkingProviders,
                                    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                    YoVariableRegistry parentRegistry)
   {
      stateMachine = new StateMachine<ManipulationState>("manipulationState", "manipulationStateSwitchTime", ManipulationState.class, yoTime, registry);

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

         if (dynamicGraphicObjectsListRegistry != null)
         {
            DynamicGraphicObjectsList list = new DynamicGraphicObjectsList("handPositionControlFrames");

            DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(handPositionControlFrame, registry, 0.5);
            dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
            list.add(dynamicGraphicReferenceFrame);

            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);
            list.hideDynamicGraphicObjects();
         }
      }

      setUpStateMachine(yoTime, fullRobotModel, twistCalculator, parameters, variousWalkingProviders,
                        dynamicGraphicObjectsListRegistry, handControllers, momentumBasedController, parentRegistry, handPositionControlFrames, jacobians);

      parentRegistry.addChild(registry);
   }

   private void setUpStateMachine(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                  ManipulationControllerParameters parameters, final VariousWalkingProviders variousWalkingProviders,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                  SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                  YoVariableRegistry parentRegistry, SideDependentList<ReferenceFrame> handPositionControlFrames,
                                  SideDependentList<GeometricJacobian> jacobians)
   {
      final DesiredHandPoseProvider handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      final TorusPoseProvider torusPoseProvider = variousWalkingProviders.getTorusPoseProvider();
      final TorusManipulationProvider torusManipulationProvider = variousWalkingProviders.getTorusManipulationProvider();

      directControlManipulationState = new HighLevelDirectControlManipulationState(yoTime, fullRobotModel,
                                                                                  twistCalculator, parameters, handPoseProvider,
                                                                                  dynamicGraphicObjectsListRegistry, handControllers,
                                                                                  handPositionControlFrames, jacobians, momentumBasedController, registry);

//      HighLevelToroidManipulationState toroidManipulationState = new HighLevelToroidManipulationState(yoTime, fullRobotModel, twistCalculator,
//                                                                    handPositionControlFrames, handControllers, jacobians, torusPoseProvider,
//                                                                    momentumBasedController, dynamicGraphicObjectsListRegistry, parentRegistry);


      final State<ManipulationState> fingerToroidManipulationState = new HighLevelFingerToroidManipulationState(twistCalculator, jacobians,
                                                                        momentumBasedController, fullRobotModel.getElevator(), torusPoseProvider,
                                                                        torusManipulationProvider, handControllers, registry,
                                                                        dynamicGraphicObjectsListRegistry);

      addTransitionFromDirectToToroid(directControlManipulationState, torusPoseProvider, fingerToroidManipulationState);
      addTransitionFromToroidToDirectBackToDefault(directControlManipulationState, handPoseProvider, fingerToroidManipulationState);
      addTransitionFromToroidToDirectWhenDone(directControlManipulationState, fingerToroidManipulationState);

      stateMachine.addState(directControlManipulationState);
      stateMachine.addState(fingerToroidManipulationState);
   }

   private static void addTransitionFromDirectToToroid(HighLevelDirectControlManipulationState directControlManipulationState,
                                                       final TorusPoseProvider torusPoseProvider, State<ManipulationState> fingerToroidManipulationState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return torusPoseProvider.checkForNewPose();
         }
      };

      StateTransition<ManipulationState> toToroidManipulation = new StateTransition<ManipulationState>(fingerToroidManipulationState.getStateEnum(),
                                                                   stateTransitionCondition);
      directControlManipulationState.addStateTransition(toToroidManipulation);
   }

   private static void addTransitionFromToroidToDirectBackToDefault(HighLevelDirectControlManipulationState directControlManipulationState,
                                                                    final DesiredHandPoseProvider
         handPoseProvider, State<ManipulationState> fingerToroidManipulationState)
   {
      StateTransitionCondition toDirectManipulationCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            // TODO: hack
            boolean defaultRequested = handPoseProvider.checkForNewPose(RobotSide.LEFT) &&!handPoseProvider.isRelativeToWorld();
            return defaultRequested;
         }
      };
      StateTransition<ManipulationState> toDirectManipulation = new StateTransition<ManipulationState>(directControlManipulationState.getStateEnum(),
            toDirectManipulationCondition);
      fingerToroidManipulationState.addStateTransition(toDirectManipulation);
   }

   private static void addTransitionFromToroidToDirectWhenDone(HighLevelDirectControlManipulationState directControlManipulationState,
                                                               final State<ManipulationState> fingerToroidManipulationState)
   {
      StateTransitionCondition doneWithFingerToroidManipulationCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return fingerToroidManipulationState.isDone();
         }
      };
      StateTransition<ManipulationState> fingerToroidManipulationDoneToDirectManipulation =
            new StateTransition<ManipulationState>(directControlManipulationState.getStateEnum(), doneWithFingerToroidManipulationCondition);
      fingerToroidManipulationState.addStateTransition(fingerToroidManipulationDoneToDirectManipulation);
   }

   public void goToDefaultState()
   {
      stateMachine.setCurrentState(ManipulationState.DIRECT_CONTROL);
      directControlManipulationState.goToDefaultState();
   }

   public void doControl()
   {
      updateGraphics();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   private void updateGraphics()
   {
      for (int i = 0; i < dynamicGraphicReferenceFrames.size(); i++)
      {
         dynamicGraphicReferenceFrames.get(i).update();
      }
   }

   public void prepareForLocomotion()
   {
      directControlManipulationState.prepareForLocomotion();
   }
}
