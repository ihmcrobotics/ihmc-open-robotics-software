package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.controllers.SE3PIDController;
import us.ihmc.robotics.controllers.SE3PIDGains;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;


public class RigidBodySpatialAccelerationControlModule
{
   private static final boolean VISUALIZE = true;

   private final YoVariableRegistry registry;
   private final TwistCalculator twistCalculator;
   private final SE3PIDController se3pdController;
   private final SpatialAccelerationVector acceleration;
   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;

   private final YoFrameVector desiredAccelerationLinearViz, desiredAccelerationAngularViz;

   public RigidBodySpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame,
         double dt, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, twistCalculator, endEffector, endEffectorFrame, dt, null, parentRegistry);
   }

   public RigidBodySpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame,
         double dt, YoSE3PIDGains taskspaceGains, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.twistCalculator = twistCalculator;
      this.endEffector = endEffector;
      this.endEffectorFrame = endEffectorFrame;
      this.se3pdController = new SE3PIDController(namePrefix, endEffectorFrame, VISUALIZE, dt, taskspaceGains, registry);
      this.acceleration = new SpatialAccelerationVector();

      desiredAccelerationLinearViz = new YoFrameVector(namePrefix + "LinearAccelViz", endEffectorFrame, registry);
      desiredAccelerationAngularViz = new YoFrameVector(namePrefix + "AngularAccelViz", endEffectorFrame, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      desiredAccelerationLinearViz.setToZero();
      desiredAccelerationAngularViz.setToZero();
      se3pdController.reset();
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public void packAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(acceleration);
   }

   private final Twist currentTwist = new Twist();

   public void doPositionControl(FramePose desiredEndEffectorPose, Twist desiredEndEffectorTwist,
         SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration, RigidBody base)
   {
      twistCalculator.packRelativeTwist(currentTwist, base, endEffector);
      currentTwist.changeBodyFrameNoRelativeTwist(endEffectorFrame);
      currentTwist.changeFrame(endEffectorFrame);

      se3pdController.compute(acceleration, desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, currentTwist);

      acceleration.getExpressedInFrame().checkReferenceFrameMatch(desiredAccelerationLinearViz.getReferenceFrame());
      desiredAccelerationLinearViz.set(acceleration.getLinearPartX(), acceleration.getLinearPartY(), acceleration.getLinearPartZ());
      desiredAccelerationAngularViz.set(acceleration.getAngularPartX(), acceleration.getAngularPartY(), acceleration.getAngularPartZ());
   }

   private final FramePose desiredEndEffectorPose = new FramePose();
   private final Twist desiredEndEffectorTwist = new Twist();
   private final SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration = new SpatialAccelerationVector();

   public void doPositionControl(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocityOfOrigin,
         FrameVector desiredAngularVelocity, FrameVector desiredLinearAccelerationOfOrigin, FrameVector desiredAngularAcceleration, RigidBody base)
   {
      packDesiredEndEffectorPoseFromDesiredPositions(desiredEndEffectorPose, desiredPosition, desiredOrientation);
      packDesiredEndEffectorTwist(desiredEndEffectorTwist, desiredLinearVelocityOfOrigin, desiredAngularVelocity, base);
      calculateDesiredEndEffectorSpatialAcceleration(feedForwardEndEffectorSpatialAcceleration, desiredLinearAccelerationOfOrigin, desiredAngularAcceleration, base);
      doPositionControl(desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, base);
   }

   private final FramePoint endEffectorPosition = new FramePoint();
   private final FrameOrientation endEffectorOrientation = new FrameOrientation();

   public void packDesiredEndEffectorPoseFromDesiredPositions(FramePose poseToPack, FramePoint endEffectorPositionIn, FrameOrientation endEffectorOrientationIn)
   {
      endEffectorPosition.setIncludingFrame(endEffectorPositionIn);
      endEffectorPosition.changeFrame(endEffectorFrame);

      endEffectorOrientation.setIncludingFrame(endEffectorOrientationIn);
      endEffectorOrientation.changeFrame(endEffectorFrame);

      poseToPack.setPoseIncludingFrame(endEffectorPosition, endEffectorOrientation);
   }

   public void packDesiredEndEffectorTwist(Twist twistToPack, FrameVector linearVelocityOfOrigin, FrameVector angularVelocity, RigidBody base)
   {
      angularVelocity.changeFrame(endEffectorFrame);
      linearVelocityOfOrigin.changeFrame(endEffectorFrame);

      twistToPack.set(endEffectorFrame, base.getBodyFixedFrame(), endEffectorFrame, linearVelocityOfOrigin.getVector(), angularVelocity.getVector());
   }

   private final Twist twistOfEndEffectorWithRespectToElevator = new Twist();

   public void calculateDesiredEndEffectorSpatialAcceleration(SpatialAccelerationVector toPack, FrameVector linearAccelerationOfOrigin,
         FrameVector angularAcceleration, RigidBody base)
   {
      angularAcceleration.changeFrame(endEffectorFrame);

      linearAccelerationOfOrigin.changeFrame(endEffectorFrame);
      twistCalculator.packRelativeTwist(twistOfEndEffectorWithRespectToElevator, base, endEffector);
      twistOfEndEffectorWithRespectToElevator.changeBodyFrameNoRelativeTwist(endEffectorFrame);

      toPack.setBasedOnOriginAcceleration(endEffectorFrame, base.getBodyFixedFrame(), endEffectorFrame, angularAcceleration, linearAccelerationOfOrigin,
            twistOfEndEffectorWithRespectToElevator);
   }

   public void setPositionProportionalGains(double kpx, double kpy, double kpz)
   {
      se3pdController.setPositionProportionalGains(kpx, kpy, kpz);
   }

   public void setPositionDerivativeGains(double kdx, double kdy, double kdz)
   {
      se3pdController.setPositionDerivativeGains(kdx, kdy, kdz);
   }

   public void setPositionIntegralGains(double kix, double kiy, double kiz, double maxIntegralError)
   {
      se3pdController.setPositionIntegralGains(kix, kiy, kiz, maxIntegralError);
   }

   public void setOrientationProportionalGains(double kpx, double kpy, double kpz)
   {
      se3pdController.setOrientationProportionalGains(kpx, kpy, kpz);
   }

   public void setOrientationDerivativeGains(double kdx, double kdy, double kdz)
   {
      se3pdController.setOrientationDerivativeGains(kdx, kdy, kdz);
   }

   public void setOrientationIntegralGains(double kix, double kiy, double kiz, double maxIntegralError)
   {
      se3pdController.setOrientationIntegralGains(kix, kiy, kiz, maxIntegralError);
   }

   public ReferenceFrame getTrackingFrame()
   {
      return endEffectorFrame;
   }

   public void setGains(SE3PIDGains gains)
   {
      se3pdController.setGains(gains);
   }

   public void setGains(YoSE3PIDGains gains)
   {
      se3pdController.setGains(gains);
   }

   public void setPositionGains(YoPositionPIDGains gains)
   {
      se3pdController.setPositionGains(gains);
   }

   public void setOrientationGains(YoOrientationPIDGains gains)
   {
      se3pdController.setOrientationGains(gains);
   }

   public void setPositionMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      se3pdController.setPositionMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void setOrientationMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      se3pdController.setOrientationMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }
   
   public void packEndeffectorVelocity(FrameVector vectorToPack)
   {
      currentTwist.packBodyOriginLinearPartInBaseFrame(vectorToPack);
   }
   
   public void packEndeffectorPosition(FramePoint pointToPack)
   {
      pointToPack.setIncludingFrame(endEffectorPosition);
   }
}
