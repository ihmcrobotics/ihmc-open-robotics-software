package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.List;

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
         Vector3d jointOffset = new Vector3d();
         ReferenceFrame frameBeforeJoint = joint.getFrameBeforeJoint();

         ReferenceFrame frameAfterParentJoint = parentJoint.getFrameAfterJoint();
         RigidBodyTransform comOffsetTransform = frameBeforeJoint.getTransformToDesiredFrame(frameAfterParentJoint);
         comOffsetTransform.get(jointOffset);

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
      revoluteJoint.packUnitJointTwist(twist);
      Vector3d jointAxis = twist.getAngularPart();
      buffer.append("Joint axis = " + jointAxis + "\n");
      buffer.append("Joint is a Pin Joint.\n");

   }

   private void printSliderJointInformation(PrismaticJoint prismaticJoint, StringBuffer buffer)
   {
      Twist twist = new Twist();
      prismaticJoint.packUnitJointTwist(twist);
      Vector3d jointAxis = twist.getLinearPart();
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

         Vector3d comOffset = new Vector3d();
         RigidBodyTransform comOffsetTransform = link.getBodyFixedFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         comOffsetTransform.get(comOffset);

         Matrix3d momentOfInertia = inertia.getMassMomentOfInertiaPartCopy();

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

   public String toString()
   {
      StringBuffer buffer = new StringBuffer();
      getRobotInformationAsStringBuffer(buffer);
      return buffer.toString();
   }

}
