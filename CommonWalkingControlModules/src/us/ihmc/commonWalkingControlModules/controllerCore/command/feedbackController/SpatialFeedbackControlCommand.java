package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.robotics.controllers.SE3PIDGains;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

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

   private final DenseMatrix64F selectionMatrix = CommonOps.identity(SpatialAccelerationVector.SIZE);
   private final DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(0, 0);
   private long jacobianForNullspaceId = GeometricJacobianHolder.NULL_JACOBIAN_ID;

   private RigidBody base;
   private RigidBody endEffector;

   private String baseName;
   private String endEffectorName;

   private final SE3PIDGains gains = new SE3PIDGains();
   private double weightForSolver = Double.POSITIVE_INFINITY;

   public SpatialFeedbackControlCommand()
   {
      setSelectionMatrixToIdentity();
   }

   @Override
   public void set(SpatialFeedbackControlCommand other)
   {
      base = other.base;
      endEffector = other.endEffector;
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;
      setSelectionMatrix(other.selectionMatrix);
      setGains(other.gains);
      setWeightForSolver(other.weightForSolver);
      nullspaceMultipliers.set(nullspaceMultipliers);

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
      setBase(base);
      setEndEffector(endEffector);
   }

   public void setBase(RigidBody base)
   {
      this.base = base;
      baseName = base.getName();
   }

   public void setEndEffector(RigidBody endEffector)
   {
      this.endEffector = endEffector;
      endEffectorName = endEffector.getName();
   }

   public void setGains(SE3PIDGainsInterface gains)
   {
      this.gains.set(gains);
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
      position.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      orientation.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      position.get(controlFrameOriginInEndEffectorFrame);
      orientation.getQuaternion(controlFrameOrientationInEndEffectorFrame);
   }

   public void changeFrameAndSetControlFrameFixedInEndEffector(FramePoint position, FrameOrientation orientation)
   {
      position.changeFrame(endEffector.getBodyFixedFrame());
      orientation.changeFrame(endEffector.getBodyFixedFrame());
      position.get(controlFrameOriginInEndEffectorFrame);
      orientation.getQuaternion(controlFrameOrientationInEndEffectorFrame);
   }

   public void setControlFrameFixedInEndEffector(FramePose pose)
   {
      pose.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      pose.getPose(controlFrameOriginInEndEffectorFrame, controlFrameOrientationInEndEffectorFrame);
   }

   public void changeFrameAndSetControlFrameFixedInEndEffector(FramePose pose)
   {
      pose.changeFrame(endEffector.getBodyFixedFrame());
      pose.getPose(controlFrameOriginInEndEffectorFrame, controlFrameOrientationInEndEffectorFrame);
   }

   public void resetNullspaceMultpliers()
   {
      nullspaceMultipliers.reshape(0, 1);
   }

   public void setNullspaceMultipliers(DenseMatrix64F nullspaceMultipliers)
   {
      this.nullspaceMultipliers.set(nullspaceMultipliers);
   }

   public void setJacobianForNullspaceId(long jacobianId)
   {
      jacobianForNullspaceId = jacobianId;
   }

   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(SpatialAccelerationVector.SIZE, SpatialAccelerationVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > SpatialAccelerationVector.SIZE)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != SpatialAccelerationVector.SIZE)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }

   public void setWeightForSolver(double weight)
   {
      weightForSolver = weight;
   }

   public void setWeightLevelForSolver(SolverWeightLevels weightLevel)
   {
      weightForSolver = weightLevel.getWeightValue();
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
      position.setIncludingFrame(endEffector.getBodyFixedFrame(), controlFrameOriginInEndEffectorFrame);
      orientation.setIncludingFrame(endEffector.getBodyFixedFrame(), controlFrameOrientationInEndEffectorFrame);
   }

   public RigidBody getBase()
   {
      return base;
   }

   public String getBaseName()
   {
      return baseName;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public String getEndEffectorName()
   {
      return endEffectorName;
   }

   public DenseMatrix64F getNullspaceMultipliers()
   {
      return nullspaceMultipliers;
   }

   public long getJacobianForNullspaceId()
   {
      return jacobianForNullspaceId;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public double getWeightForSolver()
   {
      return weightForSolver;
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
