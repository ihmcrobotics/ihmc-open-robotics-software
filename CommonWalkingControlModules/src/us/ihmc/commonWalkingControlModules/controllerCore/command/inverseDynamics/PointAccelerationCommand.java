package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointAccelerationCommand implements InverseDynamicsCommand<PointAccelerationCommand>
{
   private boolean hasWeight;
   private final FramePoint bodyFixedPointToControl = new FramePoint();
   private final FrameVector desiredAcceleration = new FrameVector();
   private final DenseMatrix64F weightVector = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(3);

   private RigidBody base;
   private RigidBody endEffector;

   private String baseName;
   private String endEffectorName;

   public PointAccelerationCommand()
   {
      setSelectionMatrixToIdentity();
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

   public void setBodyFixedPointToControl(FramePoint bodyFixedPointInEndEffectorFrame)
   {
      bodyFixedPointInEndEffectorFrame.checkReferenceFrameMatch(endEffector.getBodyFixedFrame());
      bodyFixedPointToControl.setIncludingFrame(bodyFixedPointInEndEffectorFrame);
   }

   public void resetBodyFixedPoint()
   {
      bodyFixedPointToControl.setToZero(endEffector.getBodyFixedFrame());
   }

   public void setLinearAcceleration(FrameVector desiredAccelerationWithRespectToBase)
   {
      desiredAcceleration.setIncludingFrame(desiredAccelerationWithRespectToBase);
   }

   @Override
   public void set(PointAccelerationCommand other)
   {
      bodyFixedPointToControl.setIncludingFrame(other.bodyFixedPointToControl);
      desiredAcceleration.setIncludingFrame(other.desiredAcceleration);
      selectionMatrix.set(selectionMatrix);
      setWeights(other.getWeightVector());

      base = other.base;
      endEffector = other.endEffector;

      baseName = other.baseName;
      endEffectorName = other.endEffectorName;
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

   public void setWeight(double weight)
   {
      for (int i = 0; i < 3; i++)
         weightVector.set(i, 0, weight);
      hasWeight = weight != HARD_CONSTRAINT;
   }

   public void setWeights(DenseMatrix64F weight)
   {
      hasWeight = true;

      for (int i = 0; i < 3; i++)
      {
         weightVector.set(i, 0, weight.get(i, 0));
         if (weight.get(i, 0) == HARD_CONSTRAINT)
            hasWeight = false;
      }
   }

   public void setWeights( double linearX, double linearY, double linearZ)
   {
      weightVector.set(0, 0, linearX);
      weightVector.set(1, 0, linearY);
      weightVector.set(2, 0, linearZ);

      hasWeight = linearX != HARD_CONSTRAINT && linearY != HARD_CONSTRAINT && linearZ != HARD_CONSTRAINT;
   }

   public void setWeights(Vector3D weight)
   {
      weightVector.set(0, 0, weight.getX());
      weightVector.set(1, 0, weight.getY());
      weightVector.set(2, 0, weight.getZ());

      hasWeight = weight.getX() != HARD_CONSTRAINT && weight.getY() != HARD_CONSTRAINT && weight.getZ() != HARD_CONSTRAINT;
   }

   public void removeWeight()
   {
      setWeight(HARD_CONSTRAINT);
   }

   public boolean getHasWeight()
   {
      return hasWeight;
   }

   public void getWeightMatrix(DenseMatrix64F weightMatrixToPack)
   {
      weightMatrixToPack.reshape(3, 3);
      CommonOps.setIdentity(weightMatrixToPack);
      for (int i = 0; i < 3; i++)
         weightMatrixToPack.set(i, i, weightVector.get(i, 0));
   }

   public DenseMatrix64F getWeightVector()
   {
      return weightVector;
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

   public void getBodyFixedPointIncludingFrame(FramePoint bodyFixedPointToControlToPack)
   {
      bodyFixedPointToControlToPack.setIncludingFrame(this.bodyFixedPointToControl);
   }

   public FrameVector getDesiredAcceleration()
   {
      return desiredAcceleration;
   }

   public DenseMatrix64F getSelectionMatrix()
   {
      return selectionMatrix;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.POINT;
   }

   @Override
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + "endEffector = " + endEffector.getName() + ", acceleration = " + desiredAcceleration;
      return ret;
   }
}
