package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.controllers.PositionPIDGains;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointFeedbackControlCommand extends FeedbackControlCommand<PointFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3d bodyFixedPointToControlInEndEffectorFrame = new Point3d();

   private final Point3d desiredPositionInWorld = new Point3d();
   private final Vector3d desiredLinearVelocityInWorld = new Vector3d();
   private final Vector3d feedForwardLinearAccelerationInWorld = new Vector3d();

   private final DenseMatrix64F selectionMatrix = CommonOps.identity(3);

   private RigidBody base;
   private RigidBody endEffector;

   private final PositionPIDGains gains = new PositionPIDGains();
   private double weightForSolver = Double.POSITIVE_INFINITY;

   public PointFeedbackControlCommand()
   {
      super(FeedbackControlCommandType.POINT_CONTROL);
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

   public void setGains(PositionPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   @Override
   public void set(PointFeedbackControlCommand other)
   {
      set(other.base, other.endEffector);
      setSelectionMatrix(other.selectionMatrix);
      setGains(other.gains);
      setWeightForSolver(other.weightForSolver);

      bodyFixedPointToControlInEndEffectorFrame.set(other.bodyFixedPointToControlInEndEffectorFrame);

      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredLinearVelocityInWorld.set(other.desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationInWorld.set(other.feedForwardLinearAccelerationInWorld);
   }

   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(3, 3);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > 3)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != 3)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }
   public void setWeightForSolver(double weight)
   {
      weightForSolver = weight;
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

   public void resetBodyFixedPoint()
   {
      bodyFixedPointToControlInEndEffectorFrame.set(0.0, 0.0, 0.0);
   }

   public void setBodyFixedPointToControl(FramePoint bodyFixedPointInEndEffectorFrame)
   {
      bodyFixedPointInEndEffectorFrame.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());

      bodyFixedPointInEndEffectorFrame.get(bodyFixedPointToControlInEndEffectorFrame);
   }

   public void getIncludingFrame(FramePoint desiredPositionToPack, FrameVector desiredLinearVelocityToPack, FrameVector feedForwardLinearAccelerationToPack)
   {
      desiredPositionToPack.setIncludingFrame(worldFrame, desiredPositionInWorld);
      desiredLinearVelocityToPack.setIncludingFrame(worldFrame, desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationToPack.setIncludingFrame(worldFrame, feedForwardLinearAccelerationInWorld);
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

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }


   public double getWeightForSolver()
   {
      return weightForSolver;
   }

   public PositionPIDGainsInterface getGains()
   {
      return gains;
   }
}
