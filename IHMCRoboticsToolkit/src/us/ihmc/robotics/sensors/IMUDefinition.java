package us.ihmc.robotics.sensors;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class IMUDefinition
{
   private final String name;
   private final RigidBody rigidBody;
   private final RigidBodyTransform transformFromIMUToJoint;
   private final ReferenceFrame imuFrame;

   public IMUDefinition(String name, RigidBody rigidBody, RigidBodyTransform transformFromIMUToJoint)
   {
      this.name = name;
      this.rigidBody = rigidBody;
      this.transformFromIMUToJoint = new RigidBodyTransform(transformFromIMUToJoint);

      ReferenceFrame frameAfterJoint = rigidBody.getParentJoint().getFrameAfterJoint();
      imuFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(name, frameAfterJoint, transformFromIMUToJoint);
   }

   public String getName()
   {
      return name;
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }

   public void getTransformFromIMUToJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformFromIMUToJoint);
   }

   public ReferenceFrame getIMUFrame()
   {
      return imuFrame;
   }

   @Override
   public String toString()
   {
      return "IMUDefinition: " + name + " attached to " + rigidBody.getName();
   }
}
