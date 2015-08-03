package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

public class DesiredJointAccelerationCommand
{
   private final boolean hasWeight;
   private final double weight;

   private final InverseDynamicsJoint joint;
   private final DenseMatrix64F desiredAcceleration;

   public DesiredJointAccelerationCommand(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration)
   {
      this.joint = joint;
      this.desiredAcceleration = new DenseMatrix64F(desiredAcceleration);
      this.hasWeight = false;
      this.weight = Double.POSITIVE_INFINITY;
   }

   public DesiredJointAccelerationCommand(InverseDynamicsJoint joint, DenseMatrix64F desiredAcceleration, double weight)
   {
      this.joint = joint;
      this.desiredAcceleration = new DenseMatrix64F(desiredAcceleration);
      this.hasWeight = true;
      this.weight = weight;
   }

   public void setDesiredAcceleration(DenseMatrix64F desiredAcceleration)
   {
      this.desiredAcceleration.reshape(desiredAcceleration.numRows, desiredAcceleration.numCols);
      this.desiredAcceleration.set(desiredAcceleration);
   }

   public boolean getHasWeight()
   {
      return hasWeight;
   }

   public double getWeight()
   {
      return weight;
   }

   public InverseDynamicsJoint getJoint()
   {
      return joint;
   }

   public DenseMatrix64F getDesiredAcceleration()
   {
      return desiredAcceleration;
   }

   public String toString()
   {
      return "OneDoFJointAccelerationCommand: " + joint.getName();
   }

}
