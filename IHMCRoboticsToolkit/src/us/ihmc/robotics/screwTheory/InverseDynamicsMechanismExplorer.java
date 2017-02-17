package us.ihmc.robotics.screwTheory;

import java.util.List;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class InverseDynamicsMechanismExplorer
{
   private final RigidBody rootbody;

   public InverseDynamicsMechanismExplorer(RigidBody rootbody)
   {
      super();
      this.rootbody = rootbody;
   }

   public void getRobotInformationAsStringBuffer(StringBuffer buffer)
   {
      printLinkInformation(rootbody, buffer);
   }

   private void printJointInformation(InverseDynamicsJoint joint, StringBuffer buffer)
   {
      String jointName = joint.getName();

      buffer.append("Joint name = " + jointName + "\n");
      InverseDynamicsJoint parentJoint = joint.getPredecessor().getParentJoint();
      if (parentJoint == null)
      {
         buffer.append("Root joint\n");
      } 
      else
      {
         Vector3D jointOffset = new Vector3D();
         ReferenceFrame frameBeforeJoint = joint.getFrameBeforeJoint();

         ReferenceFrame frameAfterParentJoint = parentJoint.getFrameAfterJoint();
         RigidBodyTransform comOffsetTransform = frameBeforeJoint.getTransformToDesiredFrame(frameAfterParentJoint);
         comOffsetTransform.getTranslation(jointOffset);

         buffer.append("Joint offset = " + jointOffset + "\n");
      }

      if (joint instanceof RevoluteJoint)
      {
         printPinJointInformation((RevoluteJoint) joint, buffer);
      }

      else if (joint instanceof PrismaticJoint)
      {
         printSliderJointInformation((PrismaticJoint) joint, buffer);
      }

      else if (joint instanceof SixDoFJoint)
      {
         printFloatingJointInformation((AbstractInverseDynamicsJoint) joint, buffer);
      }

      else
      {
         throw new RuntimeException("Only Pin and Slider implemented right now");
      }

      RigidBody linkRigidBody = joint.getSuccessor();
      printLinkInformation(linkRigidBody, buffer);

   }

   private void printPinJointInformation(RevoluteJoint revoluteJoint, StringBuffer buffer)
   {
      Twist twist = new Twist();
      revoluteJoint.getUnitJointTwist(twist);
      Vector3DReadOnly jointAxis = twist.getAngularPart();
      buffer.append("Joint axis = " + jointAxis + "\n");
      buffer.append("Joint is a Pin Joint.\n");

   }

   private void printSliderJointInformation(PrismaticJoint prismaticJoint, StringBuffer buffer)
   {
      Twist twist = new Twist();
      prismaticJoint.getUnitJointTwist(twist);
      Vector3DReadOnly jointAxis = twist.getLinearPart();
      buffer.append("Joint axis = " + jointAxis + "\n");
      buffer.append("Joint is a Slider Joint.\n");
   }

   private void printFloatingJointInformation(AbstractInverseDynamicsJoint floatingJoint, StringBuffer buffer)
   {
      buffer.append("Joint is a Floating Joint.\n");
   }

   private void printLinkInformation(RigidBody link, StringBuffer buffer)
   {
      RigidBodyInertia inertia = link.getInertia();
      InverseDynamicsJoint parentJoint = link.getParentJoint();
      if (inertia != null)
      {
         double mass = inertia.getMass();

         Vector3D comOffset = new Vector3D();
         RigidBodyTransform comOffsetTransform = link.getBodyFixedFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         comOffsetTransform.getTranslation(comOffset);

         Matrix3D momentOfInertia = inertia.getMassMomentOfInertiaPartCopy();

         buffer.append("Mass = " + mass + "\n");
         buffer.append("comOffset = " + comOffset + "\n");
         buffer.append("momentOfInertia = \n" + momentOfInertia + "\n");
      }

      List<InverseDynamicsJoint> childrenJoints = link.getChildrenJoints();

      for (InverseDynamicsJoint childJoint : childrenJoints)
      {
         String parentJointName;
         if (parentJoint != null)
            parentJointName = parentJoint.getName();
         else
            parentJointName = "root joint";

         buffer.append("Found Child Joint of " + parentJointName + ".\n");

         printJointInformation(childJoint, buffer);
      }

   }

   @Override
   public String toString()
   {
      StringBuffer buffer = new StringBuffer();
      getRobotInformationAsStringBuffer(buffer);
      return buffer.toString();
   }
}
