package us.ihmc.simulationconstructionset.simulatedSensors;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.JointWrenchSensor;

public class FeatherStoneJointBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private static final int WRENCH_SIZE = 6;

   private final String forceSensorName;
   private final Joint forceTorqueSensorJoint;
   private boolean doWrenchCorruption = false;

   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(WRENCH_SIZE, 1);
   private final DenseMatrix64F corruptionMatrix = new DenseMatrix64F(WRENCH_SIZE, 1);

   public FeatherStoneJointBasedWrenchCalculator(String forceSensorName, Joint forceTorqueSensorJoint)
   {
      this.forceSensorName = forceSensorName;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
   }

   @Override
   public String getName()
   {
      return forceSensorName;
   }

   private Vector3D force = new Vector3D();
   private Vector3D tau = new Vector3D();

   @Override
   public void calculate()
   {
      JointWrenchSensor sensor = forceTorqueSensorJoint.getJointWrenchSensor();

      sensor.getJointForce(force);
      sensor.getJointTorque(tau);

      wrenchMatrix.zero();

      wrenchMatrix.set(0, 0, tau.getX());
      wrenchMatrix.set(1, 0, tau.getY());
      wrenchMatrix.set(2, 0, tau.getZ());

      wrenchMatrix.set(3, 0, force.getX());
      wrenchMatrix.set(4, 0, force.getY());
      wrenchMatrix.set(5, 0, force.getZ());

      if (doWrenchCorruption)
      {
         for (int i = 0; i < WRENCH_SIZE; i++)
         {
            wrenchMatrix.add(i, 0, corruptionMatrix.get(i, 0));
         }
      }
   }

   @Override
   public Joint getJoint()
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
