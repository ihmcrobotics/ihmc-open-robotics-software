package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.robotics.controllers.PositionPIDGains;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointFeedbackControlCommand implements FeedbackControlCommand<PointFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3d bodyFixedPointToControlInEndEffectorFrame = new Point3d();

   private final Point3d desiredPositionInWorld = new Point3d();
   private final Vector3d desiredLinearVelocityInWorld = new Vector3d();
   private final Vector3d feedForwardLinearAccelerationInWorld = new Vector3d();

   private final DenseMatrix64F selectionMatrix = CommonOps.identity(3);

   private RigidBody base;
   private RigidBody endEffector;

   private String baseName;
   private String endEffectorName;

   private final PositionPIDGains gains = new PositionPIDGains();
   private double weightForSolver = Double.POSITIVE_INFINITY;

   public PointFeedbackControlCommand()
   {
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

   public void setGains(PositionPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   @Override
   public void set(PointFeedbackControlCommand other)
   {
      base = other.base;
      endEffector = other.endEffector;
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;
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

   public void setWeightLevelForSolver(SolverWeightLevels weightLevel)
   {
      weightForSolver = weightLevel.getWeightValue();
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

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.POINT;
   }
}
