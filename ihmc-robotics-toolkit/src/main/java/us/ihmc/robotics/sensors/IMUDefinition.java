package us.ihmc.robotics.sensors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class IMUDefinition
{
   private final String name;
   private final RigidBodyBasics rigidBody;
   private final RigidBodyTransform transformFromIMUToJoint;
   private final ReferenceFrame imuFrame;

   public IMUDefinition(String name, RigidBodyBasics rigidBody, RigidBodyTransformReadOnly transformFromIMUToJoint)
   {
      this.name = name;
      this.rigidBody = rigidBody;
      this.transformFromIMUToJoint = new RigidBodyTransform(transformFromIMUToJoint);

      ReferenceFrame frameAfterJoint = rigidBody.getParentJoint().getFrameAfterJoint();
      imuFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(name, frameAfterJoint, transformFromIMUToJoint);
   }

   public String getName()
   {
      return name;
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   public RigidBodyTransformReadOnly getTransformFromIMUToJoint()
   {
      return transformFromIMUToJoint;
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
