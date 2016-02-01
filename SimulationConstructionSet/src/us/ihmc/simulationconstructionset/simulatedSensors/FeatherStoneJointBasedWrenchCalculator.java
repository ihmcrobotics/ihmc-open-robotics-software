package us.ihmc.simulationconstructionset.simulatedSensors;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.simulationconstructionset.JointWrenchSensor;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;

public class FeatherStoneJointBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private final String forceSensorName;
   private final OneDegreeOfFreedomJoint forceTorqueSensorJoint;
   private boolean doWrenchCorruption = false;

   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F corruptionMatrix = new DenseMatrix64F(Wrench.SIZE, 1);

   public FeatherStoneJointBasedWrenchCalculator(String forceSensorName, OneDegreeOfFreedomJoint forceTorqueSensorJoint)
   {
      this.forceSensorName = forceSensorName;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
   }

   @Override
   public String getName()
   {
      return forceSensorName;
   }

   private Vector3d force = new Vector3d();
   private Vector3d tau = new Vector3d();

   @Override
   public void calculate()
   {
      JointWrenchSensor sensor = forceTorqueSensorJoint.getJointWrenchSensor();

      sensor.getJointForce(force);
      sensor.getJointTorque(tau);

      wrenchMatrix.zero();

      wrenchMatrix.set(0, 0, tau.x);
      wrenchMatrix.set(1, 0, tau.y);
      wrenchMatrix.set(2, 0, tau.z);

      wrenchMatrix.set(3, 0, force.x);
      wrenchMatrix.set(4, 0, force.y);
      wrenchMatrix.set(5, 0, force.z);

      if (doWrenchCorruption)
      {
         for (int i = 0; i < SpatialForceVector.SIZE; i++)
         {
            wrenchMatrix.add(i, 0, corruptionMatrix.get(i, 0));
         }
      }
   }

   @Override
   public OneDegreeOfFreedomJoint getJoint()
   {
      return forceTorqueSensorJoint;
   }

   @Override
   public DenseMatrix64F getWrench()
   {
      return wrenchMatrix;
   }

   @Override
   public void corruptWrenchElement(int row, double value)
   {
      corruptionMatrix.add(row, 0, value);
   }

   @Override
   public String toString()
   {
      return forceSensorName;
   }

   @Override
   public void getTransformToParentJoint(RigidBodyTransform transformToPack)
   {
      forceTorqueSensorJoint.getJointWrenchSensor().getTransformToParentJoint(transformToPack);
   }

   @Override
   public void setDoWrenchCorruption(boolean value)
   {
      this.doWrenchCorruption = value;
   }
}
