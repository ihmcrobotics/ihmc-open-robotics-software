package us.ihmc.simpleWholeBodyWalking;

import java.util.Arrays;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.ParameterProvider;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.dataStructures.parameters.FrameParameterVector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class SimpleFootControlModule
{
   private final YoRegistry registry;
   private final ContactableFoot contactableFoot;

   public enum ConstraintType
   {
      FULL, SWING;

      public boolean isLoadBearing()
      {
         switch (this)
         {
            case FULL:
            default:
               return false;
         }
      }
   }

   private static final double defaultCoefficientOfFriction = 0.8;
   private final DoubleParameter coefficientOfFriction;

   private final StateMachine<ConstraintType, SimpleFootControlState> stateMachine;
   private final YoEnum<ConstraintType> requestedState;
   private final EnumMap<ConstraintType, boolean[]> contactStatesMap = new EnumMap<ConstraintType, boolean[]>(ConstraintType.class);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final RobotSide robotSide;

   private final SimpleSwingState swingState;
   private final SimpleSupportState supportState;

   private final YoBoolean resetFootPolygon;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();

   public SimpleFootControlModule(RobotSide robotSide,
                                  WalkingControllerParameters walkingControllerParameters,
                                  PIDSE3GainsReadOnly swingFootControlGains,
                                  PIDSE3GainsReadOnly holdPositionFootControlGains,
                                  HighLevelHumanoidControllerToolbox controllerToolbox,
                                  YoRegistry parentRegistry)
   {
      contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      controllerToolbox.setFootContactStateFullyConstrained(robotSide);

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Foot";
      registry = new YoRegistry(sidePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      this.controllerToolbox = controllerToolbox;
      this.robotSide = robotSide;

      requestedState = new YoEnum<>(namePrefix + "RequestedState", "", registry, ConstraintType.class, true);
      requestedState.set(null);

      setupContactStatesMap();

      Vector3D defaultTouchdownVelocity = new Vector3D(0.0, 0.0, swingTrajectoryParameters.getDesiredTouchdownVelocity());
      FrameParameterVector3D touchdownVelocity = new FrameParameterVector3D(namePrefix + "TouchdownVelocity",
                                                                            ReferenceFrame.getWorldFrame(),
                                                                            defaultTouchdownVelocity,
                                                                            registry);

      Vector3D defaultTouchdownAcceleration = new Vector3D(0.0, 0.0, swingTrajectoryParameters.getDesiredTouchdownAcceleration());
      FrameParameterVector3D touchdownAcceleration = new FrameParameterVector3D(namePrefix + "TouchdownAcceleration",
                                                                                ReferenceFrame.getWorldFrame(),
                                                                                defaultTouchdownAcceleration,
                                                                                registry);

      supportState = new SimpleSupportState(contactableFoot,
                                            controllerToolbox,
                                            robotSide,
                                            controllerToolbox.getFullRobotModel(),
                                            holdPositionFootControlGains,
                                            registry);
      swingState = new SimpleSwingState(contactableFoot,
                                        controllerToolbox,
                                        robotSide,
                                        controllerToolbox.getFullRobotModel(),
                                        walkingControllerParameters,
                                        touchdownVelocity,
                                        touchdownAcceleration,
                                        swingFootControlGains,
                                        registry);

      stateMachine = setupStateMachine(namePrefix);

      resetFootPolygon = new YoBoolean(namePrefix + "ResetFootPolygon", registry);

      String targetRegistryName = SimpleFeetManager.class.getSimpleName();
      String parameterRegistryName = SimpleFootControlModule.class.getSimpleName() + "Parameters";
      coefficientOfFriction = ParameterProvider.getOrCreateParameter(targetRegistryName,
                                                                     parameterRegistryName,
                                                                     "CoefficientOfFriction",
                                                                     registry,
                                                                     defaultCoefficientOfFriction);
   }

   private void setupWrenchCommand(ContactWrenchCommand command)
   {
      command.setRigidBody(contactableFoot.getRigidBody());
      command.getSelectionMatrix().clearSelection();
      command.getSelectionMatrix().setSelectionFrame(ReferenceFrame.getWorldFrame());
      command.getSelectionMatrix().selectLinearZ(true);
      command.getWrench().setToZero(contactableFoot.getRigidBody().getBodyFixedFrame(), ReferenceFrame.getWorldFrame());
   }

   private void setupContactStatesMap()
   {
      boolean[] falses = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(falses, false);
      boolean[] trues = new boolean[contactableFoot.getTotalNumberOfContactPoints()];
      Arrays.fill(trues, true);

      contactStatesMap.put(ConstraintType.SWING, falses);
      contactStatesMap.put(ConstraintType.FULL, trues);
   }

   private StateMachine<ConstraintType, SimpleFootControlState> setupStateMachine(String namePrefix)
   {
      StateMachineFactory<ConstraintType, SimpleFootControlState> factory = new StateMachineFactory<>(ConstraintType.class);
      factory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(controllerToolbox.getYoTime());
      factory.addState(ConstraintType.FULL, supportState);
      factory.addState(ConstraintType.SWING, swingState);

      for (ConstraintType from : ConstraintType.values())
      {
         factory.addRequestedTransition(from, requestedState);
         factory.addRequestedTransition(from, from, requestedState);
      }

      return factory.build(ConstraintType.FULL);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight,
                          Vector3DReadOnly loadedFootLinearWeight,
                          Vector3DReadOnly footAngularWeight,
                          Vector3DReadOnly footLinearWeight)
   {
      swingState.setWeights(footAngularWeight, footLinearWeight);
      supportState.setWeights(loadedFootAngularWeight, loadedFootLinearWeight);
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, double swingTime)
   {
      swingState.setAdjustedFootstepAndTime(adjustedFootstep, swingTime);
   }

   public void setContactState(ConstraintType constraintType)
   {
      setContactState(constraintType, null);
   }

   public void setContactState(ConstraintType constraintType, FrameVector3D normalContactVector)
   {
      if (constraintType == ConstraintType.FULL)
      {
         // FIXME add this in for the support state
         //         footControlHelper.setFullyConstrainedNormalContactVector(normalContactVector);
      }

      controllerToolbox.setFootContactState(robotSide, contactStatesMap.get(constraintType), normalContactVector);

      if (getCurrentConstraintType() == constraintType) // Use resetCurrentState() for such case
         return;

      requestedState.set(constraintType);
   }

   public ConstraintType getCurrentConstraintType()
   {
      return stateMachine.getCurrentStateKey();
   }

   public void initialize()
   {
      stateMachine.resetToInitialState();
   }

   public void doControl()
   {
      controllerToolbox.setFootContactCoefficientOfFriction(robotSide, coefficientOfFriction.getValue());

      if (resetFootPolygon.getBooleanValue())
      {
         resetFootPolygon();
      }

      stateMachine.doTransitions();

      stateMachine.doAction();
   }

   // Used to restart the current state reseting the current state time
   public void resetCurrentState()
   {
      stateMachine.resetCurrentState();
   }

   public boolean isInFlatSupportState()
   {
      ConstraintType currentConstraintType = getCurrentConstraintType();
      return currentConstraintType == ConstraintType.FULL;
   }

   public void setFootstep(Footstep footstep, double swingTime)
   {
      swingState.setFootstep(footstep, swingTime);
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpFactor)
   {
      return swingState.requestSwingSpeedUp(speedUpFactor);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(stateMachine.getCurrentState().getInverseDynamicsCommand());

      return inverseDynamicsCommandList;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return stateMachine.getCurrentState().getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (ConstraintType constraintType : ConstraintType.values())
      {
         SimpleFootControlState state = stateMachine.getState(constraintType);
         if (state != null && state.getFeedbackControlCommand() != null)
            ret.addCommand(state.getFeedbackControlCommand());
      }
      return ret;
   }

   private void resetFootPolygon()
   {
      if (!isInFlatSupportState())
         return;
      resetFootPolygon.set(false);
      controllerToolbox.resetFootSupportPolygon(robotSide);
   }

   public Object pollStatusToReport()
   {
      return stateMachine.getCurrentState().pollStatusToReport();
   }
}