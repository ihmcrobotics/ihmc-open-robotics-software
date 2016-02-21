package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.robotics.controllers.SE3PIDGains;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class SpatialFeedbackControlCommand extends FeedbackControlCommand<SpatialFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3d bodyFixedPointToControlInEndEffectorFrame = new Point3d();

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

   private final SE3PIDGains gains = new SE3PIDGains();
   private double weightForSolver = Double.POSITIVE_INFINITY;

   public SpatialFeedbackControlCommand()
   {
      super(FeedbackControlCommandType.SPATIAL_CONTROL);
      setSelectionMatrixToIdentity();
   }

   @Override
   public void set(SpatialFeedbackControlCommand other)
   {
      set(other.base, other.endEffector);
      setSelectionMatrix(other.selectionMatrix);
      setGains(other.gains);
      setWeightForSolver(other.weightForSolver);
      nullspaceMultipliers.set(nullspaceMultipliers);

      bodyFixedPointToControlInEndEffectorFrame.set(other.bodyFixedPointToControlInEndEffectorFrame);

      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredLinearVelocityInWorld.set(other.desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationInWorld.set(other.feedForwardLinearAccelerationInWorld);

      desiredOrientationInWorld.set(other.desiredOrientationInWorld);
      desiredAngularVelocityInWorld.set(other.desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationInWorld.set(other.feedForwardAngularAccelerationInWorld);
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      this.base = base;
      this.endEffector = endEffector;
   }

   public void setBase(RigidBody base)
   {
      this.base = base;
   }

   public void setEndEffector(RigidBody endEffector)
   {
      this.endEffector = endEffector;
   }

   public void setGains(SE3PIDGains gains)
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

   public void resetBodyFixedPoint()
   {
      bodyFixedPointToControlInEndEffectorFrame.set(0.0, 0.0, 0.0);
   }

   public void setBodyFixedPointToControl(FramePoint bodyFixedPointInEndEffectorFrame)
   {
      bodyFixedPointInEndEffectorFrame.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());

      bodyFixedPointInEndEffectorFrame.get(bodyFixedPointToControlInEndEffectorFrame);
   }

   public void resetNullspaceMultpliers()
   {
      nullspaceMultipliers.reshape(0, 1);
   }

   public void setNullspaceMultpliers(DenseMatrix64F nullspaceMultipliers)
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

   public void getBodyFixedPointIncludingFrame(FramePoint bodyFixedPointToControl)
   {
      bodyFixedPointToControl.setIncludingFrame(endEffector.getBodyFixedFrame(), bodyFixedPointToControlInEndEffectorFrame);
   }

   public RigidBody getBase()
   {
      return base;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
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
}
