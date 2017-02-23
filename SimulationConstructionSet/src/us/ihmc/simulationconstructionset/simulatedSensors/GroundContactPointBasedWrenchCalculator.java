package us.ihmc.simulationconstructionset.simulatedSensors;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class GroundContactPointBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private final String forceSensorName;
   private final List<GroundContactPoint> contactPoints;
   private final OneDegreeOfFreedomJoint forceTorqueSensorJoint;

   private final RigidBodyTransform transformToParentJoint;

   private boolean doWrenchCorruption = false;
   private final DenseMatrix64F wrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F corruptionMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final Map<String, YoFrameVector> yoContactForceInSensorFrame = new HashMap<>();
   private final PoseReferenceFrame sensorFrame;

   public GroundContactPointBasedWrenchCalculator(String forceSensorName, List<GroundContactPoint> contactPoints,
                                                  OneDegreeOfFreedomJoint forceTorqueSensorJoint, RigidBodyTransform transformToParentJoint,
                                                  YoVariableRegistry registry)
   {
      this.forceSensorName = forceSensorName;
      this.contactPoints = contactPoints;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
      this.transformToParentJoint = new RigidBodyTransform(transformToParentJoint);
      this.sensorFrame = new PoseReferenceFrame(forceSensorName + "Frame", ReferenceFrame.getWorldFrame());

      for (int i = 0; i < this.contactPoints.size(); i++)
      {
         if (registry != null)
         {
            String contactPointName = contactPoints.get(i).getName();
            String namePrefix = contactPointName + "_ForceInSensorFrame";
            // Checking if that variable does already exist for some reason.
            if (registry.getVariable(namePrefix + "X") == null)
               this.yoContactForceInSensorFrame.put(contactPointName, new YoFrameVector(namePrefix, sensorFrame, registry));
         }
      }
   }

   @Override
   public String getName()
   {
      return forceSensorName;
   }

   private final RigidBodyTransform transformToOriginFrame = new RigidBodyTransform();
   private final Vector3D force = new Vector3D();
   private final Point3D contactPointOriginFrame = new Point3D();
   private final Vector3D contactVectorOriginFrame = new Vector3D();
   private final Vector3D tau = new Vector3D();

   private final Point3D tempContactPoint = new Point3D();

   @Override
   public void calculate()
   {
      //OriginaFrame : sensorFrame
      wrenchMatrix.zero();
      forceTorqueSensorJoint.getTransformToWorld(transformToOriginFrame);
      transformToOriginFrame.multiply(transformToParentJoint);
      sensorFrame.setPoseAndUpdate(transformToOriginFrame);
      transformToOriginFrame.invert();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         GroundContactPoint contactPoint = contactPoints.get(i);
         contactPoint.getForce(force);

         transformToOriginFrame.transform(force);

         contactPointOriginFrame.set(0.0, 0.0, 0.0);

         contactPoint.getPosition(tempContactPoint);
         transformToOriginFrame.transform(tempContactPoint, contactPointOriginFrame);
         contactVectorOriginFrame.set(contactPointOriginFrame);
         tau.cross(contactVectorOriginFrame, force);

         wrenchMatrix.set(0, 0, wrenchMatrix.get(0, 0) + tau.getX());
         wrenchMatrix.set(1, 0, wrenchMatrix.get(1, 0) + tau.getY());
         wrenchMatrix.set(2, 0, wrenchMatrix.get(2, 0) + tau.getZ());

         wrenchMatrix.set(3, 0, wrenchMatrix.get(3, 0) + force.getX());
         wrenchMatrix.set(4, 0, wrenchMatrix.get(4, 0) + force.getY());
         wrenchMatrix.set(5, 0, wrenchMatrix.get(5, 0) + force.getZ());

         if (yoContactForceInSensorFrame.containsKey(contactPoint.getName()))
         {
            yoContactForceInSensorFrame.get(contactPoint.getName()).set(force);
         }
      }

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
   public void setDoWrenchCorruption(boolean value)
   {
      doWrenchCorruption = value;
   }

   @Override
   public void corruptWrenchElement(int row, double value)
   {
      this.corruptionMatrix.add(row, 0, value);
   }

   @Override
   public void getTransformToParentJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformToParentJoint);
   }

   @Override
   public String toString()
   {
      return forceSensorName + " " + contactPoints;
   }
}
