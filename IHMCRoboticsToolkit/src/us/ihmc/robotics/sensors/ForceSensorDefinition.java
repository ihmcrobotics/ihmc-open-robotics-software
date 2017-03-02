package us.ihmc.robotics.sensors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ForceSensorDefinition
{
   private final String sensorName;
   private final String parentJointName;
   private final RigidBody rigidBody;

   private final RigidBodyTransform transformFromSensorToParentJoint;
   private final ReferenceFrame sensorFrame;

   public ForceSensorDefinition(String sensorName, RigidBody rigidBody, RigidBodyTransform transformFromSensorToParentJoint)
   {
      this.sensorName = sensorName;
      this.rigidBody = rigidBody;
      InverseDynamicsJoint parentJoint = rigidBody.getParentJoint();
      this.parentJointName = parentJoint.getName();
      this.transformFromSensorToParentJoint = new RigidBodyTransform(transformFromSensorToParentJoint);
      ReferenceFrame frameAfterJoint = parentJoint.getFrameAfterJoint();
      sensorFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(sensorName + "Frame", frameAfterJoint, transformFromSensorToParentJoint);
   }

   public String getSensorName()
   {
      return sensorName;
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public String getParentJointName()
   {
      return parentJointName;
   }

   public void getTransformFromSensorToParentJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformFromSensorToParentJoint);
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }
   
   @Override
   public String toString()
   {
      return "ForceSensorDefinition: " + sensorName + " attached to " + rigidBody.getName(); 
   }
   
   @Override
   public int hashCode()
   {
      return 17 + (31 * sensorName.hashCode()) + parentJointName.hashCode();
   }
   
   @Override
   public boolean equals(Object other)
   {
      if(other instanceof ForceSensorDefinition)
      {
         ForceSensorDefinition otherSensor = (ForceSensorDefinition) other;
         return otherSensor.getSensorName().equals(getSensorName()) && otherSensor.getParentJointName().equals(getParentJointName());
      }
      
      return false;
   }
}
