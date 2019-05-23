package us.ihmc.robotics.sensors;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class ForceSensorDefinition implements Settable<ForceSensorDefinition>
{
   private String sensorName;
   private RigidBodyBasics rigidBody;
   private ReferenceFrame sensorFrame;

   public static ReferenceFrame createSensorFrame(String sensorName, RigidBodyBasics rigidBody, RigidBodyTransform transformFromSensorToParentJoint)
   {
      ReferenceFrame frameAfterJoint = rigidBody.getParentJoint().getFrameAfterJoint();
      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(sensorName + "Frame", frameAfterJoint, transformFromSensorToParentJoint);
   }

   public ForceSensorDefinition()
   {
   }

   public ForceSensorDefinition(String sensorName, RigidBodyBasics rigidBody, ReferenceFrame sensorFrame)
   {
      set(sensorName, rigidBody, sensorFrame);
   }

   @Override
   public void set(ForceSensorDefinition other)
   {
      set(other.sensorName, other.rigidBody, other.sensorFrame);
   }

   public void set(String sensorName, RigidBodyBasics rigidBody, ReferenceFrame sensorFrame)
   {
      this.sensorName = sensorName;
      this.rigidBody = rigidBody;
      this.sensorFrame = sensorFrame;
   }

   public String getSensorName()
   {
      return sensorName;
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
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
      return sensorName.hashCode();
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof ForceSensorDefinition)
      {
         ForceSensorDefinition other = (ForceSensorDefinition) obj;
         if (!getSensorName().equals(other.getSensorName()))
            return false;
         if (rigidBody != other.rigidBody)
            return false;
         if (sensorFrame != other.sensorFrame)
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
