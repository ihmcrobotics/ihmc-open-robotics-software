package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels.HARD_CONSTRAINT;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.SolverWeightLevels;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointAccelerationCommand implements InverseDynamicsCommand<PointAccelerationCommand>
{
   private boolean hasWeight;
   private double weight;
   private final FramePoint contactPoint = new FramePoint();
   private final FrameVector desiredAcceleration = new FrameVector();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(3);

   private RigidBody base;
   private RigidBody endEffector;

   private String baseName;
   private String endEffectorName;

   public PointAccelerationCommand()
   {
   }

   public PointAccelerationCommand(FramePoint bodyFixedPoint, FrameVector desiredAcceleration, DenseMatrix64F selectionMatrix)
   {
      set(bodyFixedPoint, desiredAcceleration, selectionMatrix);
   }

   public PointAccelerationCommand(FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase)
   {
      this(bodyFixedPoint, desiredAccelerationWithRespectToBase, null);
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

   public void set(FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase)
   {
      set(bodyFixedPoint, desiredAccelerationWithRespectToBase, null);
   }

   @Override
   public void set(PointAccelerationCommand other)
   {
      contactPoint.setIncludingFrame(other.contactPoint);
      desiredAcceleration.setIncludingFrame(other.desiredAcceleration);
      selectionMatrix.set(selectionMatrix);
      setWeight(other.weight);

      base = other.base;
      endEffector = other.endEffector;

      baseName = other.baseName;
      endEffectorName = other.endEffectorName;
   }

   public void set(FramePoint bodyFixedPoint, FrameVector desiredAcceleration, DenseMatrix64F selectionMatrix)
   {
      this.contactPoint.setIncludingFrame(bodyFixedPoint);
      this.desiredAcceleration.setIncludingFrame(desiredAcceleration);
      if (selectionMatrix != null)
         this.selectionMatrix.set(selectionMatrix);
      else
         setSelectionMatrixToIdentity();
      removeWeight();
   }

   private void setSelectionMatrixToIdentity()
   {
      selectionMatrix.reshape(3, 3);
      CommonOps.setIdentity(selectionMatrix);
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
      hasWeight = weight != HARD_CONSTRAINT.getWeightValue();
   }

   public void setWeightLevel(SolverWeightLevels weightLevel)
   {
      setWeight(weightLevel.getWeightValue());
   }

   public void removeWeight()
   {
      setWeight(HARD_CONSTRAINT.getWeightValue());
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

   public FramePoint getContactPoint()
   {
      return contactPoint;
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
