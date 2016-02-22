package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;

public class PointAccelerationCommand extends InverseDynamicsCommand<PointAccelerationCommand>
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
      super(InverseDynamicsCommandType.TASKSPACE_POINT_MOTION);
   }

   public PointAccelerationCommand(FramePoint bodyFixedPoint, FrameVector desiredAcceleration, DenseMatrix64F selectionMatrix)
   {
      super(InverseDynamicsCommandType.TASKSPACE_POINT_MOTION);
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
      hasWeight = weight != Double.POSITIVE_INFINITY;
   }

   public void removeWeight()
   {
      setWeight(Double.POSITIVE_INFINITY);
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
   public String toString()
   {
      String ret = getClass().getSimpleName() + ": base = " + base.getName() + "endEffector = " + endEffector.getName() + ", acceleration = " + desiredAcceleration;
      return ret;
   }
}
