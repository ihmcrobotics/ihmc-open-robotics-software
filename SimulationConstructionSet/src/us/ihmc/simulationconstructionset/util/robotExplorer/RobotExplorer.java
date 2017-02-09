package us.ihmc.simulationconstructionset.util.robotExplorer;

import java.util.ArrayList;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class RobotExplorer
{
   private final Robot robot;

   public RobotExplorer(Robot robot)
   {
      this.robot = robot;
   }

   public void getRobotInformationAsStringBuffer(StringBuffer buffer)
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();

      for (Joint rootJoint : rootJoints)
      {
         buffer.append("Found Root Joint.\n");
         printJointInformation(rootJoint, buffer);
      }
   }

   private void printJointInformation(Joint joint, StringBuffer buffer)
   {
      String jointName = joint.getName();

      buffer.append("Joint name = " + jointName + "\n");

      Vector3d offset = new Vector3d();
      joint.getOffset(offset);

      buffer.append("Joint offset = " + offset + "\n");

      Vector3d jointAxis = new Vector3d();
      joint.getJointAxis(jointAxis);

      buffer.append("Joint axis = " + jointAxis + "\n");

      if (joint instanceof PinJoint)
      {
         printPinJointInformation((OneDegreeOfFreedomJoint) joint, buffer);
      }

      else if (joint instanceof SliderJoint)
      {
         printSliderJointInformation((SliderJoint) joint, buffer);
      }

      else if (joint instanceof FloatingJoint)
      {
         printFloatingJointInformation((FloatingJoint) joint, buffer);
      }

      else if (joint instanceof FloatingPlanarJoint)
      {
         printFloatingPlanarJointInformation((FloatingPlanarJoint) joint, buffer);
      }

      else
      {
         throw new RuntimeException("Only Pin and Slider implemented right now");
      }

      Link link = joint.getLink();
      printLinkInformation(link, buffer);

      ArrayList<Joint> childrenJoints = joint.getChildrenJoints();

      for (Joint childJoint : childrenJoints)
      {
         buffer.append("Found Child Joint of " + jointName + ".\n");
         printJointInformation(childJoint, buffer);
      }

   }

   private void printPinJointInformation(OneDegreeOfFreedomJoint pinJoint, StringBuffer buffer)
   {
      buffer.append("Joint is a Pin Joint.\n");
      DoubleYoVariable q = pinJoint.getQYoVariable();
      buffer.append("Its q variable is named " + q.getName() + "\n");
   }

   private void printSliderJointInformation(SliderJoint sliderJoint, StringBuffer buffer)
   {
      buffer.append("Joint is a Slider Joint.\n");
      DoubleYoVariable q = sliderJoint.getQYoVariable();
      buffer.append("Its q variable is named " + q.getName() + "\n");
   }

   private void printFloatingJointInformation(FloatingJoint floatingJoint, StringBuffer buffer)
   {
      buffer.append("Joint is a Floating Joint.\n");
   }

   private void printFloatingPlanarJointInformation(FloatingPlanarJoint floatingPlanarJoint, StringBuffer buffer)
   {
      buffer.append("Joint is a Floating Planar Joint.\n");
   }

   private void printLinkInformation(Link link, StringBuffer buffer)
   {
      double mass = link.getMass();

      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);

      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);

      buffer.append("Mass = " + mass + "\n");
      buffer.append("comOffset = " + comOffset + "\n");
      buffer.append("momentOfInertia = \n" + momentOfInertia + "\n");
   }
   
   @Override
   public String toString()
   {
      StringBuffer buffer = new StringBuffer();
      getRobotInformationAsStringBuffer(buffer);
      return buffer.toString();
   }

   public static void main(String[] args)
   {
   }
}
