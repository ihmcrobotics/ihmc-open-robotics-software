package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.ManipulableToroid;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states.HandControlState;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states.RotateToroidState;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states.ToroidManipulationState;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states.ToroidManipulationStateInterface;
import us.ihmc.tools.FormattingTools;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantConfigurationProvider;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.SE3ConfigurationProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.FilteredDiscreteVelocityYoVariable;
import us.ihmc.yoUtilities.math.trajectories.ConstantPositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.ProviderBasedConstantOrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionAction;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;


public class ToroidManipulationStateMachine
{
   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final BooleanYoVariable grabSteeringWheel = new BooleanYoVariable("grabSteeringWheel", registry);
   private final BooleanYoVariable releaseSteeringWheel = new BooleanYoVariable("releaseSteeringWheel", registry);

   private final StateMachine<ToroidManipulationState> stateMachine;

   private final SideDependentList<TaskspaceConstraintData> taskspaceConstraintData = new SideDependentList<TaskspaceConstraintData>();

   private final EnumMap<ToroidManipulationState, ToroidManipulationStateInterface<ToroidManipulationState>> manipulationStateMap =
      new EnumMap<ToroidManipulationState, ToroidManipulationStateInterface<ToroidManipulationState>>(ToroidManipulationState.class);

   private final SideDependentList<RigidBody> hands = new SideDependentList<RigidBody>();

   private final SideDependentList<RigidBodySpatialAccelerationControlModule> individualHandSpatialAccelerationControlModules =
      new SideDependentList<RigidBodySpatialAccelerationControlModule>();

   private final MomentumBasedController momentumBasedController;
   private final SideDependentList<Integer> jacobianIds;
   private final ManipulableToroid toroidUpdater;

   private final EnumYoVariable<ToroidManipulationState> requestedState = new EnumYoVariable<ToroidManipulationState>("requestedToroidManipulationState", "",
                                                                             registry, ToroidManipulationState.class, true);

   private final DoubleYoVariable moveToToroid1ExtraRadius = new DoubleYoVariable("moveToToroid1ExtraRadius", registry);
   private final DoubleYoVariable moveToToroid2ExtraRadius = new DoubleYoVariable("moveToToroid2ExtraRadius", registry);
   private final DoubleYoVariable moveToToroidOutwardRotation = new DoubleYoVariable("outwardRotation", registry);
   private final DoubleYoVariable moveToToroidPitch = new DoubleYoVariable("moveToToroidPitch", registry);
   private final DoubleYoVariable moveToToroidZ = new DoubleYoVariable("moveToToroidZ", registry);



   private final DoubleYoVariable estimatedToroidWheelAngle = new DoubleYoVariable("estimatedToroidAngle", registry);
   private final DoubleYoVariable estimatedToroidAngularVelocityAlpha = new DoubleYoVariable("estimatedToroidAngularVelocityAlpha", registry);
   private final FilteredDiscreteVelocityYoVariable estimatedToroidAngularVelocity;

   private final BooleanYoVariable grabbedToroid = new BooleanYoVariable("grabbedToroid", registry);



   private final CylindricalCoordinatesCalculator cylindricalCoordinatesCalculator = new CylindricalCoordinatesCalculator();
   private final SideDependentList<ReferenceFrame> handPositionControlFrames;


   public ToroidManipulationStateMachine(DoubleYoVariable simulationTime, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
           final ManipulableToroid toroidUpdater, SideDependentList<ReferenceFrame> handPositionControlFrames,
           SideDependentList<Integer> jacobianIds, double gravityZ, MomentumBasedController momentumBasedController,
           double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      stateMachine = new StateMachine<ToroidManipulationState>(name, name + "SwitchTime", ToroidManipulationState.class, simulationTime, registry);
      moveToToroid1ExtraRadius.set(0.2);
      moveToToroid2ExtraRadius.set(0.07);    // 0.08);
      moveToToroidOutwardRotation.set(-0.3);
      moveToToroidZ.set(0.09);    // 0.15);
      moveToToroidPitch.set(-0.6);    // 0.0

      estimatedToroidAngularVelocity = new FilteredDiscreteVelocityYoVariable("estimatedToroidAngularVelocity", "", estimatedToroidAngularVelocityAlpha,
              estimatedToroidWheelAngle, momentumBasedController.getYoTime(), registry);
      estimatedToroidAngularVelocityAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(10.0, momentumBasedController.getControlDT()));

      this.handPositionControlFrames = handPositionControlFrames;

      this.momentumBasedController = momentumBasedController;
      this.jacobianIds = jacobianIds;

      for (RobotSide robotSide : RobotSide.values)
      {
         TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
         this.taskspaceConstraintData.put(robotSide, taskspaceConstraintData);
         RigidBody hand = fullRobotModel.getHand(robotSide);
         hands.put(robotSide, hand);

         ReferenceFrame endEffectorFrame = handPositionControlFrames.get(robotSide);
         RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule = new RigidBodySpatialAccelerationControlModule(hand.getName(),
                                                                                             twistCalculator, hand, endEffectorFrame, controlDT, registry);
         individualHandSpatialAccelerationControlModules.put(robotSide, handSpatialAccelerationControlModule);
      }

      this.toroidUpdater = toroidUpdater;
      requestedState.set(null);

      RigidBody elevator = fullRobotModel.getElevator();

      SideDependentList<SE3ConfigurationProvider> currentHandConfigurationProviders = new SideDependentList<SE3ConfigurationProvider>();
      SideDependentList<SE3ConfigurationProvider> moveToWheel1ConfigurationProviders = new SideDependentList<SE3ConfigurationProvider>();
      SideDependentList<SE3ConfigurationProvider> moveToWheel2ConfigurationProviders = new SideDependentList<SE3ConfigurationProvider>();

      for (RobotSide robotSide : RobotSide.values)
      {
         currentHandConfigurationProviders.put(robotSide, new ConstantConfigurationProvider(handPositionControlFrames.get(robotSide)));    // should only be used at init of default state
         moveToWheel1ConfigurationProviders.put(robotSide,
                 new MoveToToroidPoseProvider(moveToToroid1ExtraRadius, moveToToroidZ, moveToToroidOutwardRotation, robotSide));
         moveToWheel2ConfigurationProviders.put(robotSide,
                 new MoveToToroidPoseProvider(moveToToroid2ExtraRadius, moveToToroidZ, moveToToroidOutwardRotation, robotSide));
      }

      HandControlState<ToroidManipulationState> moveToSteeringWheel1State = createIndividualHandControlState(1.0, ToroidManipulationState.MOVE_TO_TOROID_1,
                                                                               toroidUpdater.getStaticToroidReferenceFrame(), elevator,
                                                                               currentHandConfigurationProviders, moveToWheel1ConfigurationProviders,
                                                                               yoGraphicsListRegistry);


      HandControlState<ToroidManipulationState> moveToSteeringWheel2State = createIndividualHandControlState(1.0, ToroidManipulationState.MOVE_TO_TOROID_2,
                                                                               toroidUpdater.getStaticToroidReferenceFrame(), elevator,
                                                                               moveToWheel1ConfigurationProviders, moveToWheel2ConfigurationProviders,
                                                                               yoGraphicsListRegistry);




      HandControlState<ToroidManipulationState> grabSteeringWheelState = createIndividualHoldHandPositionState(ToroidManipulationState.GRAB_TOROID, elevator,
                                                                            moveToWheel2ConfigurationProviders, yoGraphicsListRegistry);


      RotateToroidState<ToroidManipulationState> moveSteeringWheel = new RotateToroidState<ToroidManipulationState>(ToroidManipulationState.MOVE_TOROID,
                                                                        toroidUpdater, hands, gravityZ, registry);

      HandControlState<ToroidManipulationState> releaseSteeringWheelState = createIndividualHoldHandPositionState(ToroidManipulationState.RELEASE_TOROID,
                                                                               elevator, moveToWheel2ConfigurationProviders, yoGraphicsListRegistry);

      HandControlState<ToroidManipulationState> moveAwayFromSteeringWheel1State = createIndividualHandControlState(1.0,
                                                                                     ToroidManipulationState.MOVE_AWAY_FROM_TOROID_1,
                                                                                     toroidUpdater.getStaticToroidReferenceFrame(), elevator,
                                                                                     moveToWheel2ConfigurationProviders, moveToWheel1ConfigurationProviders,
                                                                                     yoGraphicsListRegistry);



      addState(moveToSteeringWheel1State);
      addState(moveToSteeringWheel2State);
      addState(grabSteeringWheelState);
      addState(moveSteeringWheel);
      addState(releaseSteeringWheelState);
      addState(moveAwayFromSteeringWheel1State);

      addStateTransition(moveToSteeringWheel1State, moveToSteeringWheel2State, grabSteeringWheel, null);
      StateTransitionAction closeFingersAction = new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            if (grabSteeringWheel.getBooleanValue())
            {
//               for (RobotSide robotSide : RobotSide.values)
//               {
//                  handControllers.get(robotSide).closeFingers();
//               }
            }
         }
      };
      addStateTransition(moveToSteeringWheel2State, grabSteeringWheelState, grabSteeringWheel, closeFingersAction);


      StateTransitionCondition handControllerIsOpenCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
//            for (RobotSide robotSide : RobotSide.values)
//            {
//               if (!handControllers.get(robotSide).isOpen())
//               {
//                  return false;
//               }
//            }

            return true;
         }
      };
      grabSteeringWheelState.addStateTransition(new StateTransition<ToroidManipulationState>(ToroidManipulationState.MOVE_TOROID,
              handControllerIsOpenCondition, new StateTransitionAction()
      {
         public void doTransitionAction()
         {
            grabbedToroid();
         }
      }));



      addStateTransition(moveSteeringWheel, releaseSteeringWheelState, releaseSteeringWheel, new StateTransitionAction()
      {
         public void doTransitionAction()
         {
//            for (RobotSide robotSide : RobotSide.values)
//            {
//               handControllers.get(robotSide).openFingers();
//            }

            releasedToroid();
         }
      });

      releaseSteeringWheelState.addStateTransition(new StateTransition<ToroidManipulationState>(ToroidManipulationState.MOVE_AWAY_FROM_TOROID_1,
              handControllerIsOpenCondition));

      parentRegistry.addChild(registry);


      grabSteeringWheel.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            releaseSteeringWheel.set(!((BooleanYoVariable) v).getBooleanValue());
         }
      });

      releaseSteeringWheel.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            grabSteeringWheel.set(!((BooleanYoVariable) v).getBooleanValue());
         }
      });
   }

   private void addStateTransition(ToroidManipulationStateInterface<ToroidManipulationState> fromState,
                                   ToroidManipulationStateInterface<ToroidManipulationState> toState, BooleanYoVariable yoVariable,
                                   StateTransitionAction action)
   {
      StateTransitionCondition condition = new SteeringTransitionCondition(fromState, toState, yoVariable);
      fromState.addStateTransition(new StateTransition<ToroidManipulationState>(toState.getStateEnum(), condition, action));
   }

   public void initialize()
   {
      stateMachine.setCurrentState(ToroidManipulationState.MOVE_TO_TOROID_1);
   }

   private void addState(ToroidManipulationStateInterface<ToroidManipulationState> state)
   {
      stateMachine.addState(state);
      manipulationStateMap.put(state.getStateEnum(), state);
   }

   public void doControl()
   {
      updateToroidState();
      stateMachine.checkTransitionConditionsThoroughly();
      stateMachine.doAction();

      ToroidManipulationStateInterface<ToroidManipulationState> manipulationState = manipulationStateMap.get(stateMachine.getCurrentStateEnum());
      for (RobotSide robotSide : RobotSide.values)
      {
         TaskspaceConstraintData taskspaceConstraintData = this.taskspaceConstraintData.get(robotSide);
         taskspaceConstraintData.set(manipulationState.getDesiredHandAcceleration(robotSide));
         int jacobian = jacobianIds.get(robotSide);
         RigidBody hand = hands.get(robotSide);
         momentumBasedController.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
         momentumBasedController.setExternalWrenchToCompensateFor(hand, manipulationState.getHandExternalWrench(robotSide));
      }
   }

   private HandControlState<ToroidManipulationState> createIndividualHoldHandPositionState(ToroidManipulationState steeringState, RigidBody base,
           SideDependentList<SE3ConfigurationProvider> configurationProviders, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      SideDependentList<OrientationTrajectoryGenerator> orientationTrajectoryGenerators = new SideDependentList<OrientationTrajectoryGenerator>();
      SideDependentList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new SideDependentList<PositionTrajectoryGenerator>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + FormattingTools.underscoredToCamelCase(steeringState.toString(), true);

         String orientationNamePrefix = namePrefix + "OrientationGenerator";
         SE3ConfigurationProvider configurationProvider = configurationProviders.get(robotSide);
         ProviderBasedConstantOrientationTrajectoryGenerator orientationTrajectoryGenerator =
            new ProviderBasedConstantOrientationTrajectoryGenerator(orientationNamePrefix, base.getBodyFixedFrame(), configurationProvider, 0.0, registry);
         orientationTrajectoryGenerators.put(robotSide, orientationTrajectoryGenerator);

         String positionNamePrefix = namePrefix + "PositionGenerator";
         ConstantPositionTrajectoryGenerator positionTrajectoryGenerator = new ConstantPositionTrajectoryGenerator(positionNamePrefix,
                                                                              base.getBodyFixedFrame(), configurationProvider, 0.0, registry);
         positionTrajectoryGenerators.put(robotSide, positionTrajectoryGenerator);

      }

      final HandControlState<ToroidManipulationState> ret = new HandControlState<ToroidManipulationState>(steeringState, base, positionTrajectoryGenerators,
                                                               orientationTrajectoryGenerators, individualHandSpatialAccelerationControlModules,
                                                               yoGraphicsListRegistry, registry);

      return ret;
   }

   private HandControlState<ToroidManipulationState> createIndividualHandControlState(double trajectoryTime, ToroidManipulationState steeringState,
           ReferenceFrame referenceFrame, RigidBody base, SideDependentList<SE3ConfigurationProvider> initialConfigurationProviders,
           SideDependentList<SE3ConfigurationProvider> finalConfigurationProviders, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ConstantDoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      SideDependentList<PositionTrajectoryGenerator> positionTrajectoryGenerators = new SideDependentList<PositionTrajectoryGenerator>();
      SideDependentList<OrientationTrajectoryGenerator> orientationTrajectoryGenerators = new SideDependentList<OrientationTrajectoryGenerator>();
      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + FormattingTools.underscoredToCamelCase(steeringState.toString(), true);
         StraightLinePositionTrajectoryGenerator positionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator(namePrefix, referenceFrame,
                                                                                  trajectoryTimeProvider, initialConfigurationProviders.get(robotSide),
                                                                                  finalConfigurationProviders.get(robotSide), registry);
         positionTrajectoryGenerator.setContinuouslyUpdateFinalPosition(true);
         positionTrajectoryGenerators.put(robotSide, positionTrajectoryGenerator);


         OrientationInterpolationTrajectoryGenerator orientationTrajectory = new OrientationInterpolationTrajectoryGenerator(namePrefix, referenceFrame,
                                                                                trajectoryTimeProvider, initialConfigurationProviders.get(robotSide),
                                                                                finalConfigurationProviders.get(robotSide), registry);
         orientationTrajectoryGenerators.put(robotSide, orientationTrajectory);
         orientationTrajectory.setContinuouslyUpdateFinalOrientation(true);
      }

      final HandControlState<ToroidManipulationState> ret = new HandControlState<ToroidManipulationState>(steeringState, base, positionTrajectoryGenerators,
                                                               orientationTrajectoryGenerators, individualHandSpatialAccelerationControlModules,
                                                               yoGraphicsListRegistry, registry);

      return ret;
   }

   public void setIndividualHandPositionControlGains(double positionKp, double positionKd, double orientationKp, double orientationKd)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule = individualHandSpatialAccelerationControlModules.get(robotSide);
         rigidBodySpatialAccelerationControlModule.setPositionProportionalGains(positionKp, positionKp, positionKp);
         rigidBodySpatialAccelerationControlModule.setPositionDerivativeGains(positionKd, positionKd, positionKd);
         rigidBodySpatialAccelerationControlModule.setOrientationProportionalGains(orientationKp, orientationKp, orientationKp);
         rigidBodySpatialAccelerationControlModule.setOrientationDerivativeGains(orientationKd, orientationKd, orientationKd);
      }
   }

   private FramePose getMoveToWheelConfiguration(RobotSide robotSide, ReferenceFrame steeringWheelBaseFrame, double radius, double z, double outwardRotation,
           double pitchRotation)
   {
      double aroundTheCLock = 2.0 * Math.PI;
      double deltaAngle = robotSide.negateIfRightSide(3.0 / 12.0 * aroundTheCLock);
      double twelveOClockAngle = Math.PI / 2.0;
      double radiansFromXAxis = twelveOClockAngle + deltaAngle;

      return cylindricalCoordinatesCalculator.getPoseFromCylindricalCoordinates(robotSide, steeringWheelBaseFrame, radiansFromXAxis, radius, z, outwardRotation,
              pitchRotation);
   }



   private class MoveToToroidPoseProvider implements SE3ConfigurationProvider
   {
      private final DoubleYoVariable extraRadius;
      private final RobotSide robotSide;
      private final DoubleYoVariable z;
      private final DoubleYoVariable outwardRotation;

      private MoveToToroidPoseProvider(DoubleYoVariable extraRadius, DoubleYoVariable z, DoubleYoVariable outwardRotation, RobotSide robotSide)
      {
         this.extraRadius = extraRadius;
         this.z = z;
         this.outwardRotation = outwardRotation;
         this.robotSide = robotSide;
      }

      public void get(FrameOrientation orientationToPack)
      {
         FramePose framePose = getFramePose();
         framePose.getOrientationIncludingFrame(orientationToPack);
      }

      public void get(FramePoint positionToPack)
      {
         FramePose framePose = getFramePose();
         framePose.getPositionIncludingFrame(positionToPack);
      }

      private FramePose getFramePose()
      {
         ReferenceFrame toroidBaseFrame = toroidUpdater.getStaticToroidReferenceFrame();
         double radius = toroidUpdater.getToroidRadius() + extraRadius.getDoubleValue();

         return getMoveToWheelConfiguration(robotSide, toroidBaseFrame, radius, z.getDoubleValue(), outwardRotation.getDoubleValue(),
                                            moveToToroidPitch.getDoubleValue());
      }
   }


   private class SteeringTransitionCondition implements StateTransitionCondition
   {
      private final ToroidManipulationStateInterface<ToroidManipulationState> fromState;
      private final ToroidManipulationStateInterface<ToroidManipulationState> toState;
      private final BooleanYoVariable yoVariable;

      public SteeringTransitionCondition(ToroidManipulationStateInterface<ToroidManipulationState> fromState,
                                         ToroidManipulationStateInterface<ToroidManipulationState> toState, BooleanYoVariable yoVariable)
      {
         this.fromState = fromState;
         this.toState = toState;
         this.yoVariable = yoVariable;
      }

      public boolean checkCondition()
      {
         boolean forcedTransition = requestedState.getEnumValue() == toState.getStateEnum();
         boolean regularTransition = yoVariable.getBooleanValue() && fromState.isDone();

         return forcedTransition || regularTransition;
      }
   }


   private void grabbedToroid()
   {
      grabbedToroid.set(true);
   }

   private void releasedToroid()
   {
      grabbedToroid.set(false);
   }

   public void updateToroidState()
   {
      if (grabbedToroid.getBooleanValue())
      {
         double toroidAngle = 0.0;
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePoint handPosition = new FramePoint(handPositionControlFrames.get(robotSide));
            handPosition.changeFrame(toroidUpdater.getStaticToroidReferenceFrame());

            double angle = Math.atan2(handPosition.getX(), handPosition.getY());
            toroidAngle -= angle;

         }

         toroidAngle /= 2.0;

         toroidUpdater.setQ(toroidAngle);
      }

      estimatedToroidAngularVelocity.update();
      toroidUpdater.setQd(estimatedToroidAngularVelocity.getDoubleValue());
   }
}
