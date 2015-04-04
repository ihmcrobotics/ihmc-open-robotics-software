package us.ihmc.simulationconstructionset.simulatedSensors;

import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.Wrench;

public class GroundContactPointBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private final String forceSensorName;
   private final List<GroundContactPoint> contactPoints;
   private final OneDegreeOfFreedomJoint forceTorqueSensorJoint;
   
   private final RigidBodyTransform transformToParentJoint;
   
   private boolean doWrenchCorruption = false;
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F corruptionMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   
   public GroundContactPointBasedWrenchCalculator(String forceSensorName, List<GroundContactPoint> contactPoints, 
         OneDegreeOfFreedomJoint forceTorqueSensorJoint, RigidBodyTransform transformToParentJoint)
   {
      this.forceSensorName = forceSensorName;
      this.contactPoints = contactPoints;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
      this.transformToParentJoint = new RigidBodyTransform(transformToParentJoint);
   }
   
   public String getName()
   {
      return forceSensorName;
   }
   
   private final RigidBodyTransform transformToOriginFrame = new RigidBodyTransform();
   private final Vector3d force = new Vector3d();
   private final Point3d contactPointOriginFrame = new Point3d();
   private final Vector3d contactVectorOriginFrame = new Vector3d();
   private final Vector3d tau = new Vector3d();
   
   public void calculate()
   {
      //OriginaFrame : sensorFrame
      wrenchMatrix.zero();
      forceTorqueSensorJoint.getTransformToWorld(transformToOriginFrame);
      transformToOriginFrame.multiply(transformToParentJoint);
      transformToOriginFrame.invert();
      
      for(int i = 0; i <  contactPoints.size(); i++)
      {
         GroundContactPoint contactPoint = contactPoints.get(i);
         contactPoint.getForce(force);
         
         transformToOriginFrame.transform(force);

         contactPointOriginFrame.set(0.0, 0.0, 0.0);
         transformToOriginFrame.transform(contactPoint.getPositionPoint(), contactPointOriginFrame);
         contactVectorOriginFrame.set(contactPointOriginFrame);
         tau.cross(contactVectorOriginFrame, force);
         
         wrenchMatrix.set(0, 0, wrenchMatrix.get(0, 0) + tau.x);
         wrenchMatrix.set(1, 0, wrenchMatrix.get(1, 0) + tau.y);
         wrenchMatrix.set(2, 0, wrenchMatrix.get(2, 0) + tau.z);
         
         wrenchMatrix.set(3, 0, wrenchMatrix.get(3, 0) + force.x);
         wrenchMatrix.set(4, 0, wrenchMatrix.get(4, 0) + force.y);
         wrenchMatrix.set(5, 0, wrenchMatrix.get(5, 0) + force.z);
      }
      
      if(doWrenchCorruption)
      {
         for(int i = 0; i < Wrench.SIZE; i++)
         {
            wrenchMatrix.add(i, 0, corruptionMatrix.get(i,0));
         }
      }
   }


   public OneDegreeOfFreedomJoint getJoint()
   {
      return forceTorqueSensorJoint;
   }


   public DenseMatrix64F getWrench()
   {
      return wrenchMatrix;
   }
   
   public void setDoWrenchCorruption(boolean value)
   {
      doWrenchCorruption = value;
   }
   
   public void corruptWrenchElement(int row, double value)
   {
      this.corruptionMatrix.add(row, 0, value);
   }
   
   public String toString()
   {
      return forceSensorName + " " + contactPoints;
   }
}
