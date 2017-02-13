package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.controllers.SE3PIDGains;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class SpatialFeedbackControlCommand implements FeedbackControlCommand<SpatialFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3d controlFrameOriginInEndEffectorFrame = new Point3d();
   private final Quat4d controlFrameOrientationInEndEffectorFrame = new Quat4d();

   private final Point3d desiredPositionInWorld = new Point3d();
   private final Vector3d desiredLinearVelocityInWorld = new Vector3d();
   private final Vector3d feedForwardLinearAccelerationInWorld = new Vector3d();

   private final Quat4d desiredOrientationInWorld = new Quat4d();
   private final Vector3d desiredAngularVelocityInWorld = new Vector3d();
   private final Vector3d feedForwardAngularAccelerationInWorld = new Vector3d();

   private final SE3PIDGains gains = new SE3PIDGains();

   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   public SpatialFeedbackControlCommand()
   {
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
   }

   @Override
   public void set(SpatialFeedbackControlCommand other)
   {
      spatialAccelerationCommand.set(other.spatialAccelerationCommand);

      controlFrameOriginInEndEffectorFrame.set(other.controlFrameOriginInEndEffectorFrame);
      controlFrameOrientationInEndEffectorFrame.set(other.controlFrameOrientationInEndEffectorFrame);

      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredLinearVelocityInWorld.set(other.desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationInWorld.set(other.feedForwardLinearAccelerationInWorld);

      desiredOrientationInWorld.set(other.desiredOrientationInWorld);
      desiredAngularVelocityInWorld.set(other.desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationInWorld.set(other.feedForwardAngularAccelerationInWorld);
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      spatialAccelerationCommand.set(base, endEffector);
   }

   public void setPrimaryBase(RigidBody primaryBase)
   {
      spatialAccelerationCommand.setPrimaryBase(primaryBase);
   }

   public void setGains(SE3PIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   public void setGains(OrientationPIDGainsInterface orientationGains)
   {
      this.gains.set(orientationGains);
   }

   public void setGains(PositionPIDGainsInterface positionGains)
   {
      this.gains.set(positionGains);
   }

   public void set(FramePoint desiredPosition, FrameVector desiredLinearVelocity, FrameVector feedForwardLinearAcceleration)
   {
      desiredPosition.checkReferenceFrameMatch(worldFrame);
      desiredLinearVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardLinearAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredPosition.get(desiredPositionInWorld);
      desiredLinearVelocity.get(desiredLinearVelocityInWorld);
      feedForwardLinearAcceleration.get(feedForwardLinearAccelerationInWorld);
   }

   public void set(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.checkReferenceFrameMatch(worldFrame);
      desiredAngularVelocity.checkReferenceFrameMatch(worldFrame);
      feedForwardAngularAcceleration.checkReferenceFrameMatch(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocity.get(desiredAngularVelocityInWorld);
      feedForwardAngularAcceleration.get(feedForwardAngularAccelerationInWorld);
   }

   public void changeFrameAndSet(FramePoint desiredPosition, FrameVector desiredLinearVelocity, FrameVector feedForwardLinearAcceleration)
   {
      desiredPosition.changeFrame(worldFrame);
      desiredLinearVelocity.changeFrame(worldFrame);
      feedForwardLinearAcceleration.changeFrame(worldFrame);

      desiredPosition.get(desiredPositionInWorld);
      desiredLinearVelocity.get(desiredLinearVelocityInWorld);
      feedForwardLinearAcceleration.get(feedForwardLinearAccelerationInWorld);
   }

   public void changeFrameAndSet(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      feedForwardAngularAcceleration.changeFrame(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocity.get(desiredAngularVelocityInWorld);
      feedForwardAngularAcceleration.get(feedForwardAngularAccelerationInWorld);
   }

   public void resetBodyFixedPoint()
   {
      controlFrameOriginInEndEffectorFrame.set(0.0, 0.0, 0.0);
      controlFrameOrientationInEndEffectorFrame.set(0.0, 0.0, 0.0, 1.0);
   }

   public void setControlFrameFixedInEndEffector(FramePoint position, FrameOrientation orientation)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      position.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      orientation.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      position.get(controlFrameOriginInEndEffectorFrame);
      orientation.getQuaternion(controlFrameOrientationInEndEffectorFrame);
   }

   public void changeFrameAndSetControlFrameFixedInEndEffector(FramePoint position, FrameOrientation orientation)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      position.changeFrame(endEffector.getBodyFixedFrame());
      orientation.changeFrame(endEffector.getBodyFixedFrame());
      position.get(controlFrameOriginInEndEffectorFrame);
      orientation.getQuaternion(controlFrameOrientationInEndEffectorFrame);
   }

   public void setControlFrameFixedInEndEffector(FramePose pose)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      pose.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      pose.getPose(controlFrameOriginInEndEffectorFrame, controlFrameOrientationInEndEffectorFrame);
   }

   public void changeFrameAndSetControlFrameFixedInEndEffector(FramePose pose)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      pose.changeFrame(endEffector.getBodyFixedFrame());
      pose.getPose(controlFrameOriginInEndEffectorFrame, controlFrameOrientationInEndEffectorFrame);
   }

   public void setSelectionMatrixToIdentity()
   {
      spatialAccelerationCommand.setSelectionMatrixToIdentity();
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      spatialAccelerationCommand.setSelectionMatrix(selectionMatrix);
   }

   public void setWeightForSolver(double weight)
   {
      spatialAccelerationCommand.setWeight(weight);
   }

   public void setWeightsForSolver(Vector3d angular, Vector3d linear)
   {
      spatialAccelerationCommand.setWeights(angular, linear);
   }

   public void setAlphaTaskPriorityForSolver(double alpha)
   {
      spatialAccelerationCommand.setAlphaTaskPriority(alpha);
   }

   public void getIncludingFrame(FramePoint desiredPositionToPack, FrameVector desiredLinearVelocityToPack, FrameVector feedForwardLinearAccelerationToPack)
   {
      desiredPositionToPack.setIncludingFrame(worldFrame, desiredPositionInWorld);
      desiredLinearVelocityToPack.setIncludingFrame(worldFrame, desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationToPack.setIncludingFrame(worldFrame, feedForwardLinearAccelerationInWorld);
   }

   public void getIncludingFrame(FrameOrientation desiredOrientationToPack, FrameVector desiredAngularVelocityToPack, FrameVector feedForwardAngularAccelerationToPack)
   {
      desiredOrientationToPack.setIncludingFrame(worldFrame, desiredOrientationInWorld);
      desiredAngularVelocityToPack.setIncludingFrame(worldFrame, desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationToPack.setIncludingFrame(worldFrame, feedForwardAngularAccelerationInWorld);
   }

   public void getControlFramePoseIncludingFrame(FramePoint position, FrameOrientation orientation)
   {
      RigidBody endEffector = spatialAccelerationCommand.getEndEffector();
      position.setIncludingFrame(endEffector.getBodyFixedFrame(), controlFrameOriginInEndEffectorFrame);
      orientation.setIncludingFrame(endEffector.getBodyFixedFrame(), controlFrameOrientationInEndEffectorFrame);
   }

   public RigidBody getBase()
   {
      return spatialAccelerationCommand.getBase();
   }

   public RigidBody getEndEffector()
   {
      return spatialAccelerationCommand.getEndEffector();
   }

   public SpatialAccelerationCommand getSpatialAccelerationCommand()
   {
      return spatialAccelerationCommand;
   }

   public SE3PIDGainsInterface getGains()
   {
      return gains;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.TASKSPACE;
   }
}
