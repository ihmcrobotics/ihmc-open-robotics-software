package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointAccelerationCommand implements InverseDynamicsCommand<PointAccelerationCommand>
{
   private boolean hasWeight;
   private double weight;
   private final FramePoint bodyFixedPointToControl = new FramePoint();
   private final FrameVector desiredAcceleration = new FrameVector();
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
      setWeight(other.weight);

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
      this.weight = weight;
      hasWeight = weight != HARD_CONSTRAINT;
   }

   public void removeWeight()
   {
      setWeight(HARD_CONSTRAINT);
   }

   public boolean getHasWeight()
   {
      return hasWeight;
   }

   public double getWeight()
   {
      return weight;
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
