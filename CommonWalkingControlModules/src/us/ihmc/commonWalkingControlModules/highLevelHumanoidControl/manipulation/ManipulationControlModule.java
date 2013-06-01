package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
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
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.states.direct.HighLevelDirectControlManipulationState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.HighLevelFingerValveManipulationState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandLoadBearingProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.DesiredHandPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.TorusManipulationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.TorusPoseProvider;
import us.ihmc.commonWalkingControlModules.packets.TorusManipulationPacket;
import us.ihmc.commonWalkingControlModules.packets.TorusPosePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;
import java.util.List;

/**
 * @author twan
 *         Date: 5/13/13
 */
public class ManipulationControlModule
{
   private static final boolean DEBUG_TOROID = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StateMachine<ManipulationState> stateMachine;
   private final List<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();
   private final DoubleYoVariable time;
   private final VariousWalkingProviders variousWalkingProviders;
   private HighLevelDirectControlManipulationState directControlManipulationState;
   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("onFirstTick", registry);
   private boolean haveSentDummyTorusPacket = false;

   public ManipulationControlModule(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                    ManipulationControllerParameters parameters, final VariousWalkingProviders variousWalkingProviders,
                                    DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                    SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                    YoVariableRegistry parentRegistry)
   {
      stateMachine = new StateMachine<ManipulationState>("manipulationState", "manipulationStateSwitchTime", ManipulationState.class, yoTime, registry);

      this.variousWalkingProviders = variousWalkingProviders;

      this.time = momentumBasedController.getYoTime();

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

      setUpStateMachine(yoTime, fullRobotModel, twistCalculator, parameters, variousWalkingProviders, dynamicGraphicObjectsListRegistry, handControllers,
                        momentumBasedController, handPositionControlFrames, jacobians);

      parentRegistry.addChild(registry);
   }

   private void setUpStateMachine(DoubleYoVariable yoTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
                                  ManipulationControllerParameters parameters, final VariousWalkingProviders variousWalkingProviders,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
                                  SideDependentList<HandControllerInterface> handControllers, MomentumBasedController momentumBasedController,
                                  SideDependentList<ReferenceFrame> handPositionControlFrames,
                                  SideDependentList<GeometricJacobian> jacobians)
   {
      final DesiredHandPoseProvider handPoseProvider = variousWalkingProviders.getDesiredHandPoseProvider();
      final DesiredHandLoadBearingProvider handLoadBearingProvider = variousWalkingProviders.getDesiredHandLoadBearingProvider();
      final TorusPoseProvider torusPoseProvider = variousWalkingProviders.getTorusPoseProvider();
      final TorusManipulationProvider torusManipulationProvider = variousWalkingProviders.getTorusManipulationProvider();

      directControlManipulationState = new HighLevelDirectControlManipulationState(yoTime, fullRobotModel, twistCalculator, parameters, handPoseProvider,
              handLoadBearingProvider, dynamicGraphicObjectsListRegistry, handControllers, handPositionControlFrames, jacobians, momentumBasedController,
              registry);

//    HighLevelToroidManipulationState toroidManipulationState = new HighLevelToroidManipulationState(yoTime, fullRobotModel, twistCalculator,
//                                                                  handPositionControlFrames, handControllers, jacobians, torusPoseProvider,
//                                                                  momentumBasedController, dynamicGraphicObjectsListRegistry, parentRegistry);


      final HighLevelFingerValveManipulationState fingerToroidManipulationState = new HighLevelFingerValveManipulationState(twistCalculator, jacobians,
                                                                        momentumBasedController, fullRobotModel.getElevator(), torusPoseProvider,
                                                                        torusManipulationProvider, handControllers, registry,
                                                                        dynamicGraphicObjectsListRegistry);

      if (DEBUG_TOROID)
         fingerToroidManipulationState.setUpForDebugging();

      addTransitionFromDirectToToroid(directControlManipulationState, torusManipulationProvider, fingerToroidManipulationState);
      addTransitionFromToroidToDirectBackToDefault(directControlManipulationState, handPoseProvider, fingerToroidManipulationState);
      addTransitionFromToroidToDirectWhenDone(directControlManipulationState, fingerToroidManipulationState);

      stateMachine.addState(directControlManipulationState);
      stateMachine.addState(fingerToroidManipulationState);
   }

   private static void addTransitionFromDirectToToroid(HighLevelDirectControlManipulationState directControlManipulationState,
           final TorusManipulationProvider torusManipulationProvider, State<ManipulationState> fingerToroidManipulationState)
   {
      StateTransitionCondition stateTransitionCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return torusManipulationProvider.checkForNewData();
         }
      };

      StateTransition<ManipulationState> toToroidManipulation = new StateTransition<ManipulationState>(fingerToroidManipulationState.getStateEnum(),
                                                                   stateTransitionCondition);
      directControlManipulationState.addStateTransition(toToroidManipulation);
   }

   private static void addTransitionFromToroidToDirectBackToDefault(HighLevelDirectControlManipulationState directControlManipulationState,
           final DesiredHandPoseProvider handPoseProvider, State<ManipulationState> fingerToroidManipulationState)
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
      doDebugStuff();

      if (!hasBeenInitialized.getBooleanValue())
      {
         goToDefaultState();
         hasBeenInitialized.set(true);
      }

      updateGraphics();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   private void doDebugStuff()
   {
      if (DEBUG_TOROID && time.getDoubleValue() >= 1.0 && !haveSentDummyTorusPacket)
      {
         Quat4d orientation = new Quat4d();
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(orientation, -Math.PI / 2.0, 0.0, Math.PI / 2.0);

         Quat4d postRotation = new Quat4d();
         double initialTorusAngle = 0.0; // 3.0 * Math.PI / 4.0;
         RotationFunctions.setQuaternionBasedOnYawPitchRoll(postRotation, initialTorusAngle, 0.0, 0.0);
         orientation.mul(postRotation);

         TorusPosePacket dummyTorusPacket = new TorusPosePacket(new Point3d(0.6, 0.0, 1.2), orientation, 0.1);
         variousWalkingProviders.getTorusPoseProvider().consumeObject(dummyTorusPacket);

         double rotationAmountSigned = 5.0;
         variousWalkingProviders.getTorusManipulationProvider().consumeObject(new TorusManipulationPacket(rotationAmountSigned));
         haveSentDummyTorusPacket = true;
      }
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
