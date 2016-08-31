package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class BodyFixedPointSpatialAccelerationControlModule
{
   private final YoVariableRegistry registry;
   private final YoSE3OffsetFrame bodyFixedControlFrame;
   private final RigidBodySpatialAccelerationControlModule accelerationControlModule;

   public BodyFixedPointSpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, double dt,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, twistCalculator, endEffector, dt, null, parentRegistry);
   }

   public BodyFixedPointSpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, double dt,
         YoSE3PIDGainsInterface taskspaceGains, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      bodyFixedControlFrame = new YoSE3OffsetFrame("bodyFixedControlFrame", endEffector.getBodyFixedFrame(), registry);

      accelerationControlModule = new RigidBodySpatialAccelerationControlModule(namePrefix, twistCalculator, endEffector, bodyFixedControlFrame, dt,
            taskspaceGains, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      accelerationControlModule.reset();
   }

   public void doPositionControl(FramePose desiredEndEffectorPose, Twist desiredEndEffectorTwist,
         SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration, RigidBody base)
   {
      accelerationControlModule.doPositionControl(desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, base);
   }

   public void doPositionControl(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocityOfOrigin,
         FrameVector desiredAngularVelocity, FrameVector desiredLinearAccelerationOfOrigin, FrameVector desiredAngularAcceleration, RigidBody base)
   {
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocityOfOrigin, desiredAngularVelocity,
            desiredLinearAccelerationOfOrigin, desiredAngularAcceleration, base);
   }

   public void setBodyFixedControlFrame(FramePoint position, FrameOrientation orientation)
   {
      bodyFixedControlFrame.setOffsetToParent(position, orientation);
   }

   public void setGains(SE3PIDGainsInterface gains)
   {
      accelerationControlModule.setGains(gains);
   }

   public void setToControlBodyFixedFrame()
   {
      bodyFixedControlFrame.setToZero();
   }

   public ReferenceFrame getTrackingFrame()
   {
      return bodyFixedControlFrame;
   }

   public RigidBody getEndEffector()
   {
      return accelerationControlModule.getEndEffector();
   }

   public void getEndEffectorCurrentLinearVelocity(FrameVector endEffectorLinearVelocityToPack)
   {
      accelerationControlModule.getEndEffectorCurrentLinearVelocity(endEffectorLinearVelocityToPack);
   }

   public void getEndEffectorCurrentAngularVelocity(FrameVector endEffectorAngularVelocityToPack)
   {
      accelerationControlModule.getEndEffectorCurrentAngularVelocity(endEffectorAngularVelocityToPack);
   }

   public void getAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationControlModule.getAcceleration(accelerationToPack);
   }
}
