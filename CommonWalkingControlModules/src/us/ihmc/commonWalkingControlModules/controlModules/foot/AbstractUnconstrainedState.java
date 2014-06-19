package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
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
   protected boolean trajectoryWasReplanned;

   protected double swingKpXY, swingKpZ, swingKpOrientation, swingZetaXYZ, swingZetaOrientation;

   public AbstractUnconstrainedState(ConstraintType constraintType, YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity,
         YoFrameVector yoDesiredLinearAcceleration, RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody, EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule, RobotSide robotSide,
         YoVariableRegistry registry)
   {
      super(constraintType, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration, accelerationControlModule, momentumBasedController,
            contactableBody, requestedState, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape,
            legSingularityAndKneeCollapseAvoidanceControlModule, robotSide, registry);
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

      setSwingControlGains(swingKpXY, swingKpZ, swingKpOrientation, swingZetaXYZ, swingZetaOrientation);
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

      legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectoryForSingularityAvoidance(desiredPosition, desiredLinearVelocity,
            desiredLinearAcceleration);

      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
      accelerationControlModule.packAcceleration(footAcceleration);

      setTaskspaceConstraint(footAcceleration);
   }

   public void doTransitionOutOfAction()
   {
      yoDesiredPosition.setToNaN();
      trajectoryWasReplanned = false;

      accelerationControlModule.reset();
   }

   public void setSwingGains(double swingKpXY, double swingKpZ, double swingKpOrientation, double swingZetaXYZ, double swingZetaOrientation)
   {
      this.swingKpXY = swingKpXY;
      this.swingKpZ = swingKpZ;
      this.swingKpOrientation = swingKpOrientation;
      this.swingZetaXYZ = swingZetaXYZ;
      this.swingZetaOrientation = swingZetaOrientation;
   }

   private void setSwingControlGains(double kxyPosition, double kzPosition, double kOrientation, double zetaXYZ, double zetaOrientation)
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
