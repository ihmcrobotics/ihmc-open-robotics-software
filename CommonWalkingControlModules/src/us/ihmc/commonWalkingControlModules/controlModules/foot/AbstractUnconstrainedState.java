package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

/**
 * The unconstrained state is used if the foot is moved free in space without constrains. Depending on the type of trajectory
 * this can either be a movement along a straight line or (in case of walking) a swing motion.
 * 
 * E.g. the MoveStraightState and the SwingState extend this class and implement the trajectory related methods.
 */
public abstract class AbstractUnconstrainedState extends AbstractFootControlState
{
   private static final boolean CORRECT_SWING_CONSIDERING_JOINT_LIMITS = true;
   
   protected final BooleanYoVariable isTrajectoryDone;
   protected final BooleanYoVariable isUnconstrained;
   protected boolean trajectoryWasReplanned;
   
   protected double swingKpXY, swingKpZ, swingKpOrientation, swingZetaXYZ, swingZetaOrientation;
   
   public AbstractUnconstrainedState(ConstraintType constraintType,
         YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity, YoFrameVector yoDesiredLinearAcceleration,
         RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody,
         EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange,
         BooleanYoVariable doSingularityEscape, BooleanYoVariable forceFootAccelerateIntoGround,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule,
         RobotSide robotSide,
         
         BooleanYoVariable isTrajectoryDone, BooleanYoVariable isUnconstrained)
   {
      super(constraintType, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration,
            accelerationControlModule, momentumBasedController, contactableBody,
            requestedState, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange,
            doSingularityEscape, forceFootAccelerateIntoGround, legSingularityAndKneeCollapseAvoidanceControlModule,
            robotSide);
      
      this.isTrajectoryDone = isTrajectoryDone;
      this.isUnconstrained = isUnconstrained;
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

   public void doTransitionIntoAction()
   {
      legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(true);

      accelerationControlModule.reset();

      isCoPOnEdge = false;
      initializeTrajectory();

      isTrajectoryDone.set(false);
      isUnconstrained.set(true);

      setSwingControlGains(swingKpXY, swingKpZ, swingKpOrientation, swingZetaXYZ, swingZetaOrientation);

//      if (visualize)
//      {
//         desiredOrientationFrameGraphic.showGraphicObject();
//         correctedDesiredOrientationFrameGraphic.showGraphicObject();
//      }
   }

   public void doSpecificAction()
   {
      if (doSingularityEscape.getBooleanValue())
      {
         initializeTrajectory();
      }

      computeAndPackTrajectory();

      yoDesiredPosition.set(desiredPosition);

      desiredOrientation.setAndChangeFrame(trajectoryOrientation);

      if (CORRECT_SWING_CONSIDERING_JOINT_LIMITS)
         correctInputsAccordingToJointLimits();

      legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectoryForSingularityAvoidance(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);

      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
      accelerationControlModule.packAcceleration(footAcceleration);

      setTaskspaceConstraint(footAcceleration);

//      updateVisualizers();
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

//   private final void updateVisualizers()
//   {
//      if (!visualize)
//         return;
//
//      desiredOrientationFrame.update();
//      correctedDesiredOrientationFrame.update();
//      desiredOrientationFrameGraphic.update();
//      correctedDesiredOrientationFrameGraphic.update();
//   }

   public void doTransitionOutOfAction()
   {
      yoDesiredPosition.setToNaN();
      trajectoryWasReplanned = false;
      isUnconstrained.set(false);

      accelerationControlModule.reset();

//      if (visualize)
//      {
//         desiredOrientationFrameGraphic.hideGraphicObject();
//         correctedDesiredOrientationFrameGraphic.hideGraphicObject();
//      }
   }
   
   public void setSwingGains(double swingKpXY, double swingKpZ, double swingKpOrientation,
         double swingZetaXYZ, double swingZetaOrientation)
   {
      this.swingKpXY = swingKpXY;
      this.swingKpZ = swingKpZ;
      this.swingKpOrientation = swingKpOrientation;
      this.swingZetaXYZ = swingZetaXYZ;
      this.swingZetaOrientation = swingZetaOrientation;
   }
   
   private void setSwingControlGains(double kxyPosition, double kzPosition,
         double kOrientation, double zetaXYZ, double zetaOrientation)
   {
      double dxyPosition = GainCalculator.computeDerivativeGain(kxyPosition, zetaXYZ);
      double dzPosition = GainCalculator.computeDerivativeGain(kzPosition, zetaXYZ);
      double dOrientation = GainCalculator.computeDerivativeGain(kOrientation, zetaOrientation);

      accelerationControlModule.setPositionProportionalGains(kxyPosition, kxyPosition, kzPosition);
      accelerationControlModule.setPositionDerivativeGains(dxyPosition, dxyPosition, dzPosition);
      accelerationControlModule.setOrientationProportionalGains(kOrientation, kOrientation, kOrientation);
      accelerationControlModule.setOrientationDerivativeGains(dOrientation, dOrientation, dOrientation);
   }
}
