package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.SpatialAccelerationCommand;
import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.tools.FormattingTools;

/**
 * The unconstrained state is used if the foot is moved free in space without constrains. Depending on the type of trajectory
 * this can either be a movement along a straight line or (in case of walking) a swing motion.
 *
 * E.g. the MoveStraightState and the SwingState extend this class and implement the trajectory related methods.
 */
public abstract class AbstractUnconstrainedState extends AbstractFootControlState
{
   private static final boolean USE_ALL_LEG_JOINT_SWING_CORRECTOR = false;
   private static final boolean CONTROL_WITH_RESPECT_TO_PELVIS = false;

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   protected boolean trajectoryWasReplanned;

   protected final YoSE3PIDGains gains;

   protected final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;
   private final LegJointLimitAvoidanceControlModule legJointLimitAvoidanceControlModule;

   private final YoFramePoint yoDesiredPosition;
   private final YoFrameVector yoDesiredLinearVelocity;
   private final BooleanYoVariable yoSetDesiredAccelerationToZero;
   private final BooleanYoVariable yoSetDesiredVelocityToZero;

   private final RigidBody pelvis;

   public AbstractUnconstrainedState(ConstraintType constraintType, FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(constraintType, footControlHelper, registry);

      this.legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();
      this.gains = gains;

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName() + FormattingTools.underscoredToCamelCase(constraintType.toString().toLowerCase(), true);
      yoDesiredLinearVelocity = new YoFrameVector(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      yoDesiredLinearVelocity.setToNaN();
      yoDesiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", worldFrame, registry);
      yoDesiredPosition.setToNaN();
      yoSetDesiredAccelerationToZero = new BooleanYoVariable(namePrefix + "SetDesiredAccelerationToZero", registry);
      yoSetDesiredVelocityToZero = new BooleanYoVariable(namePrefix + "SetDesiredVelocityToZero", registry);
      pelvis = momentumBasedController.getFullRobotModel().getPelvis();

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
         legJointLimitAvoidanceControlModule = new LegJointLimitAvoidanceControlModule(namePrefix, registry, momentumBasedController, robotSide);
      else
         legJointLimitAvoidanceControlModule = null;
   }

   /**
    * initializes all the trajectories
    */
   protected abstract void initializeTrajectory();

   /**
    * compute the and pack the following variables:
    * desiredPosition, desiredLinearVelocity, desiredLinearAcceleration,
    * trajectoryOrientation, desiredAngularVelocity, desiredAngularAcceleration
    */
   protected abstract void computeAndPackTrajectory();

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(true);

      initializeTrajectory();

      footControlHelper.setGains(gains);
      footControlHelper.resetSelectionMatrix();
   }

   @Override
   public void doSpecificAction()
   {
      footControlHelper.setGains(gains);

      //TODO: If we reinitialize after singularity escape, we 
      // need to do it smartly. The stuff below causes really
      // bad swing tracking due to a huge discontinuity in the desireds.
      //      if (footControlHelper.isDoingSingularityEscape())
      //      {
      //         initializeTrajectory();
      //      }

      computeAndPackTrajectory();

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
      {
         legJointLimitAvoidanceControlModule.correctSwingFootTrajectory(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
               desiredLinearAcceleration, desiredAngularAcceleration);
      }

      legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectory(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);

      if (yoSetDesiredVelocityToZero.getBooleanValue())
      {
         desiredLinearVelocity.setToZero();
      }

      if (yoSetDesiredAccelerationToZero.getBooleanValue())
      {
         desiredLinearAcceleration.setToZero();
      }

      RigidBody baseForControl = CONTROL_WITH_RESPECT_TO_PELVIS ? pelvis : rootBody;
      RigidBodySpatialAccelerationControlModule accelerationControlModule = footControlHelper.getAccelerationControlModule();
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, desiredLinearAcceleration,
            desiredAngularAcceleration, baseForControl);
      accelerationControlModule.packAcceleration(footAcceleration);

      footControlHelper.submitTaskspaceConstraint(footAcceleration, spatialAccelerationCommand);

      desiredPosition.changeFrame(worldFrame);
      yoDesiredPosition.set(desiredPosition);
      desiredLinearVelocity.changeFrame(worldFrame);
      yoDesiredLinearVelocity.set(desiredLinearVelocity);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      yoDesiredPosition.setToNaN();
      yoDesiredLinearVelocity.setToNaN();
      trajectoryWasReplanned = false;
      footControlHelper.resetSelectionMatrix();
   }

   @Override
   public SpatialAccelerationCommand getInverseDynamicsCommand()
   {
      return spatialAccelerationCommand;
   }
}
