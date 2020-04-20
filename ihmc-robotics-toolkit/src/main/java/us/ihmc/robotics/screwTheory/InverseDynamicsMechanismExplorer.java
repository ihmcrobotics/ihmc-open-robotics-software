package us.ihmc.robotics.screwTheory;

import java.util.List;

import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;

public class InverseDynamicsMechanismExplorer
{
   private final RigidBodyBasics rootbody;

   public InverseDynamicsMechanismExplorer(RigidBodyBasics rootbody)
   {
      super();
      this.rootbody = rootbody;
   }

   public void getRobotInformationAsStringBuffer(StringBuffer buffer)
   {
      printLinkInformation(rootbody, buffer);
   }

   private void printJointInformation(JointBasics joint, StringBuffer buffer)
   {
      String jointName = joint.getName();

      buffer.append("Joint name = " + jointName + "\n");
      JointBasics parentJoint = joint.getPredecessor().getParentJoint();
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
         jointOffset.set(comOffsetTransform.getTranslation());

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
         printFloatingJointInformation((SixDoFJoint) joint, buffer);
      }

      else
      {
         throw new RuntimeException("Only Pin and Slider implemented right now");
      }

      RigidBodyBasics linkRigidBody = joint.getSuccessor();
      printLinkInformation(linkRigidBody, buffer);

   }

   private void printPinJointInformation(RevoluteJoint revoluteJoint, StringBuffer buffer)
   {
      Twist twist = new Twist();
      twist.setIncludingFrame(revoluteJoint.getUnitJointTwist());
      Vector3DReadOnly jointAxis = twist.getAngularPart();
      buffer.append("Joint axis = " + jointAxis + "\n");
      buffer.append("Joint is a Pin Joint.\n");

   }

   private void printSliderJointInformation(PrismaticJoint prismaticJoint, StringBuffer buffer)
   {
      Twist twist = new Twist();
      twist.setIncludingFrame(prismaticJoint.getUnitJointTwist());
      Vector3DReadOnly jointAxis = twist.getLinearPart();
      buffer.append("Joint axis = " + jointAxis + "\n");
      buffer.append("Joint is a Slider Joint.\n");
   }

   private void printFloatingJointInformation(SixDoFJoint floatingJoint, StringBuffer buffer)
   {
      buffer.append("Joint is a Floating Joint.\n");
   }

   private void printLinkInformation(RigidBodyBasics link, StringBuffer buffer)
   {
      SpatialInertiaBasics inertia = link.getInertia();
      JointBasics parentJoint = link.getParentJoint();
      if (inertia != null)
      {
         double mass = inertia.getMass();

         Vector3D comOffset = new Vector3D();
         RigidBodyTransform comOffsetTransform = link.getBodyFixedFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         comOffset.set(comOffsetTransform.getTranslation());

         Matrix3DBasics momentOfInertia = inertia.getMomentOfInertia();

         buffer.append("Mass = " + mass + "\n");
         buffer.append("comOffset = " + comOffset + "\n");
         buffer.append("momentOfInertia = \n" + momentOfInertia + "\n");
      }

      List<? extends JointBasics> childrenJoints = link.getChildrenJoints();

      for (JointBasics childJoint : childrenJoints)
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
