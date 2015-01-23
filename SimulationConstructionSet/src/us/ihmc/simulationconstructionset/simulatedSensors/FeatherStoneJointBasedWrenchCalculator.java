package us.ihmc.simulationconstructionset.simulatedSensors;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.simulationconstructionset.JointWrenchSensor;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.utilities.screwTheory.Wrench;

public class FeatherStoneJointBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private final String forceSensorName;
   private final OneDegreeOfFreedomJoint forceTorqueSensorJoint;
   
   
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   
   public FeatherStoneJointBasedWrenchCalculator(String forceSensorName,
         OneDegreeOfFreedomJoint forceTorqueSensorJoint)
   {
      this.forceSensorName = forceSensorName;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
   }
   
   public String getName()
   {
      return forceSensorName;
   }

   private Vector3d force = new Vector3d();
   private Vector3d tau = new Vector3d();
   
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
   }


   public OneDegreeOfFreedomJoint getJoint()
   {
      return forceTorqueSensorJoint;
   }


   public DenseMatrix64F getWrench()
   {
      return wrenchMatrix;
   }
   
   public String toString()
   {
      return forceSensorName;
   }
}
