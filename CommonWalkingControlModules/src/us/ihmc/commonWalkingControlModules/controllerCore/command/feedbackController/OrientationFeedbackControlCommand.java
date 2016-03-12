package us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.robotics.controllers.OrientationPIDGains;
import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class OrientationFeedbackControlCommand implements FeedbackControlCommand<OrientationFeedbackControlCommand>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Quat4d desiredOrientationInWorld = new Quat4d();
   private final Vector3d desiredAngularVelocityInWorld = new Vector3d();
   private final Vector3d feedForwardAngularAccelerationInWorld = new Vector3d();
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(3, SpatialAccelerationVector.SIZE);

   private RigidBody base;
   private RigidBody endEffector;

   private String baseName;
   private String endEffectorName;

   private final OrientationPIDGains gains = new OrientationPIDGains();
   private double weightForSolver = Double.POSITIVE_INFINITY;

   public OrientationFeedbackControlCommand()
   {
      setSelectionMatrixToIdentity();
   }

   @Override
   public void set(OrientationFeedbackControlCommand other)
   {
      base = other.base;
      endEffector = other.endEffector;
      baseName = other.baseName;
      endEffectorName = other.endEffectorName;
      setSelectionMatrix(other.selectionMatrix);
      setGains(other.gains);
      desiredOrientationInWorld.set(other.desiredOrientationInWorld);
      desiredAngularVelocityInWorld.set(other.desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationInWorld.set(other.feedForwardAngularAccelerationInWorld);
      setWeightForSolver(other.weightForSolver);
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

   public void setGains(OrientationPIDGainsInterface gains)
   {
      this.gains.set(gains);
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

   public void changeFrameAndSet(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);
      feedForwardAngularAcceleration.changeFrame(worldFrame);

      desiredOrientation.getQuaternion(desiredOrientationInWorld);
      desiredAngularVelocity.get(desiredAngularVelocityInWorld);
      feedForwardAngularAcceleration.get(feedForwardAngularAccelerationInWorld);
   }

   public void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(3, SpatialAccelerationVector.SIZE);
      selectionMatrix.set(0, 0, 1.0);
      selectionMatrix.set(1, 1, 1.0);
      selectionMatrix.set(2, 2, 1.0);
   }

   public void setSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      if (selectionMatrix.getNumRows() > 3)
         throw new RuntimeException("Unexpected number of rows: " + selectionMatrix.getNumRows());
      if (selectionMatrix.getNumCols() != SpatialAccelerationVector.SIZE)
         throw new RuntimeException("Unexpected number of columns: " + selectionMatrix.getNumCols());

      this.selectionMatrix.set(selectionMatrix);
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public void setWeightForSolver(double weight)
   {
      weightForSolver = weight;
   }

   public void setWeightLevelForSolver(SolverWeightLevels weightLevel)
   {
      weightForSolver = weightLevel.getWeightValue();
   }

   public void getIncludingFrame(FrameOrientation desiredOrientationToPack, FrameVector desiredAngularVelocityToPack, FrameVector feedForwardAngularAccelerationToPack)
   {
      desiredOrientationToPack.setIncludingFrame(worldFrame, desiredOrientationInWorld);
      desiredAngularVelocityToPack.setIncludingFrame(worldFrame, desiredAngularVelocityInWorld);
      feedForwardAngularAccelerationToPack.setIncludingFrame(worldFrame, feedForwardAngularAccelerationInWorld);
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

   public double getWeightForSolver()
   {
      return weightForSolver;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.ORIENTATION;
   }

   public OrientationPIDGainsInterface getGains()
   {
      return gains;
   }
}
