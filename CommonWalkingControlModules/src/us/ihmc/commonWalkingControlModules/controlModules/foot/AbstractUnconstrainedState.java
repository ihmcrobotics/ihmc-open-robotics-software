package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.robotics.FormattingTools;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

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

   protected boolean trajectoryWasReplanned;

   protected final YoSE3PIDGains gains;

   protected final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;
   private final LegJointLimitAvoidanceControlModule legJointLimitAvoidanceControlModule;

   private final OneDoFJoint hipYawJoint, anklePitchJoint, ankleRollJoint;

   private final YoFramePoint yoDesiredPosition;
   private final YoFramePoint yoAdjustedDesiredPosition;
   private final YoFrameVector yoDesiredLinearVelocity;
   private final BooleanYoVariable yoSetDesiredAccelerationToZero;
   private final BooleanYoVariable yoSetDesiredVelocityToZero;

   private final RigidBody pelvis;

   public AbstractUnconstrainedState(ConstraintType constraintType, FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(constraintType, footControlHelper, registry);

      this.legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();
      this.gains = gains;

      this.hipYawJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.HIP_YAW);
      this.ankleRollJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_ROLL);
      this.anklePitchJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.ANKLE_PITCH);

      RigidBody foot = contactableFoot.getRigidBody();
      String namePrefix = foot.getName() + FormattingTools.underscoredToCamelCase(constraintType.toString().toLowerCase(), true);
      yoDesiredLinearVelocity = new YoFrameVector(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      yoDesiredLinearVelocity.setToNaN();
      yoDesiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", worldFrame, registry);
      yoAdjustedDesiredPosition = new YoFramePoint(namePrefix + "AdjustedDesiredPosition", worldFrame, registry);
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
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
              desiredLinearAcceleration, desiredAngularAcceleration, baseForControl);
      accelerationControlModule.packAcceleration(footAcceleration);

      footControlHelper.updateSelectionMatrixToHandleAnkleRollAndHipYawAlignment();
      footControlHelper.submitTaskspaceConstraint(footAcceleration);

      desiredPosition.changeFrame(worldFrame);
      yoDesiredPosition.set(desiredPosition);
      desiredLinearVelocity.changeFrame(worldFrame);
      yoDesiredLinearVelocity.set(desiredLinearVelocity);
   }

   private final double[] desiredYawPitchRoll = new double[3];
   private final double epsilon = 1e-3;

   // TODO Pretty much hackish...
   private void correctInputsAccordingToJointLimits()
   {
      ReferenceFrame frameBeforeHipYawJoint = hipYawJoint.getFrameBeforeJoint();
      desiredOrientation.changeFrame(frameBeforeHipYawJoint);
      desiredOrientation.getYawPitchRoll(desiredYawPitchRoll);

      if (desiredYawPitchRoll[0] > hipYawJoint.getJointLimitUpper() - epsilon)
      {
         desiredYawPitchRoll[0] = hipYawJoint.getJointLimitUpper();
         desiredAngularVelocity.changeFrame(frameBeforeHipYawJoint);
         desiredAngularVelocity.setZ(Math.min(0.0, desiredAngularVelocity.getZ()));
         desiredAngularAcceleration.changeFrame(frameBeforeHipYawJoint);
         desiredAngularAcceleration.setZ(Math.min(0.0, desiredAngularVelocity.getZ()));
      }
      else if (desiredYawPitchRoll[0] < hipYawJoint.getJointLimitLower() + epsilon)
      {
         desiredYawPitchRoll[0] = hipYawJoint.getJointLimitLower();
         desiredAngularVelocity.changeFrame(frameBeforeHipYawJoint);
         desiredAngularVelocity.setZ(Math.max(0.0, desiredAngularVelocity.getZ()));
         desiredAngularAcceleration.changeFrame(frameBeforeHipYawJoint);
         desiredAngularAcceleration.setZ(Math.max(0.0, desiredAngularVelocity.getZ()));
      }

      desiredOrientation.setYawPitchRoll(desiredYawPitchRoll);

      ReferenceFrame frameBeforeAnklePitchJoint = anklePitchJoint.getFrameBeforeJoint();
      desiredOrientation.changeFrame(frameBeforeAnklePitchJoint);
      desiredOrientation.getYawPitchRoll(desiredYawPitchRoll);

      if (desiredYawPitchRoll[1] > anklePitchJoint.getJointLimitUpper() - epsilon)
      {
         desiredYawPitchRoll[1] = anklePitchJoint.getJointLimitUpper();
         desiredAngularVelocity.changeFrame(frameBeforeAnklePitchJoint);
         desiredAngularVelocity.setY(Math.min(0.0, desiredAngularVelocity.getY()));
         desiredAngularAcceleration.changeFrame(frameBeforeAnklePitchJoint);
         desiredAngularAcceleration.setY(Math.min(0.0, desiredAngularVelocity.getY()));
      }
      else if (desiredYawPitchRoll[1] < anklePitchJoint.getJointLimitLower() + epsilon)
      {
         desiredYawPitchRoll[1] = anklePitchJoint.getJointLimitLower();
         desiredAngularVelocity.changeFrame(frameBeforeAnklePitchJoint);
         desiredAngularVelocity.setY(Math.max(0.0, desiredAngularVelocity.getY()));
         desiredAngularAcceleration.changeFrame(frameBeforeAnklePitchJoint);
         desiredAngularAcceleration.setY(Math.max(0.0, desiredAngularVelocity.getY()));
      }

      desiredOrientation.setYawPitchRoll(desiredYawPitchRoll);

      ReferenceFrame frameBeforeAnkleRollJoint = ankleRollJoint.getFrameBeforeJoint();
      desiredOrientation.changeFrame(frameBeforeAnkleRollJoint);
      desiredOrientation.getYawPitchRoll(desiredYawPitchRoll);

      if (desiredYawPitchRoll[2] > ankleRollJoint.getJointLimitUpper() - epsilon)
      {
         desiredYawPitchRoll[2] = ankleRollJoint.getJointLimitUpper();
         desiredAngularVelocity.changeFrame(frameBeforeAnkleRollJoint);
         desiredAngularVelocity.setX(Math.min(0.0, desiredAngularVelocity.getX()));
         desiredAngularAcceleration.changeFrame(frameBeforeAnkleRollJoint);
         desiredAngularAcceleration.setX(Math.min(0.0, desiredAngularVelocity.getX()));
      }
      else if (desiredYawPitchRoll[2] < ankleRollJoint.getJointLimitLower() + epsilon)
      {
         desiredYawPitchRoll[2] = ankleRollJoint.getJointLimitLower();
         desiredAngularVelocity.changeFrame(frameBeforeAnkleRollJoint);
         desiredAngularVelocity.setX(Math.max(0.0, desiredAngularVelocity.getX()));
         desiredAngularAcceleration.changeFrame(frameBeforeAnkleRollJoint);
         desiredAngularAcceleration.setX(Math.max(0.0, desiredAngularVelocity.getX()));
      }

      desiredOrientation.setYawPitchRoll(desiredYawPitchRoll);

      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      desiredAngularAcceleration.changeFrame(worldFrame);
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
}
