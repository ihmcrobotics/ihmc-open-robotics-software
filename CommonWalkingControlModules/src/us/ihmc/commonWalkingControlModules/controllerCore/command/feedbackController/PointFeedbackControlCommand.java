package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PointAccelerationCommand;
import us.ihmc.robotics.controllers.PositionPIDGains;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointFeedbackControlCommand implements FeedbackControlCommand<PointFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Point3d desiredPositionInWorld = new Point3d();
   private final Vector3d desiredLinearVelocityInWorld = new Vector3d();
   private final Vector3d feedForwardLinearAccelerationInWorld = new Vector3d();

   private final PositionPIDGains gains = new PositionPIDGains();

   private final PointAccelerationCommand pointAccelerationCommand = new PointAccelerationCommand();

   public PointFeedbackControlCommand()
   {
      pointAccelerationCommand.setSelectionMatrixToIdentity();
   }

   public void set(RigidBody base, RigidBody endEffector)
   {
      pointAccelerationCommand.set(base, endEffector);
   }

   public void setPrimaryBase(RigidBody primaryBase)
   {
      pointAccelerationCommand.setPrimaryBase(primaryBase);
   }

   public void setGains(PositionPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   @Override
   public void set(PointFeedbackControlCommand other)
   {
      desiredPositionInWorld.set(other.desiredPositionInWorld);
      desiredLinearVelocityInWorld.set(other.desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationInWorld.set(other.feedForwardLinearAccelerationInWorld);
      setGains(other.gains);

      pointAccelerationCommand.set(other.pointAccelerationCommand);
   }

   public void setSelectionMatrixToIdentity()
   {
      pointAccelerationCommand.setSelectionMatrixToIdentity();
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      pointAccelerationCommand.setSelectionMatrix(selectionMatrix);
   }

   public void setWeightForSolver(double weight)
   {
      pointAccelerationCommand.setWeight(weight);
   }

   public void setWeightsForSolver(Vector3d weight)
   {
      pointAccelerationCommand.setWeights(weight);
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
      pointAccelerationCommand.resetBodyFixedPoint();
   }

   public void setBodyFixedPointToControl(FramePoint bodyFixedPointInEndEffectorFrame)
   {
      pointAccelerationCommand.setBodyFixedPointToControl(bodyFixedPointInEndEffectorFrame);
   }

   public void getIncludingFrame(FramePoint desiredPositionToPack, FrameVector desiredLinearVelocityToPack, FrameVector feedForwardLinearAccelerationToPack)
   {
      desiredPositionToPack.setIncludingFrame(worldFrame, desiredPositionInWorld);
      desiredLinearVelocityToPack.setIncludingFrame(worldFrame, desiredLinearVelocityInWorld);
      feedForwardLinearAccelerationToPack.setIncludingFrame(worldFrame, feedForwardLinearAccelerationInWorld);
   }

   public void getBodyFixedPointIncludingFrame(FramePoint bodyFixedPointToControlToPack)
   {
      pointAccelerationCommand.getBodyFixedPointIncludingFrame(bodyFixedPointToControlToPack);
   }

   public RigidBody getBase()
   {
      return pointAccelerationCommand.getBase();
   }

   public RigidBody getEndEffector()
   {
      return pointAccelerationCommand.getEndEffector();
   }

   public PointAccelerationCommand getPointAccelerationCommand()
   {
      return pointAccelerationCommand;
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
