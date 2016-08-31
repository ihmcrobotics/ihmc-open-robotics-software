package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import us.ihmc.robotics.robotDescription.FloatingPlanarJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class RobotConstructorFromRobotDescription
{
   private Robot robot;

   public RobotConstructorFromRobotDescription()
   {
   }

   public void constructRobotFromDescription(RobotDescription description)
   {
      robot = new Robot(description.getName());

      ArrayList<JointDescription> rootJointDescriptions = description.getRootJoints();

      for (JointDescription rootJointDescription : rootJointDescriptions)
      {
         if (rootJointDescription instanceof FloatingPlanarJointDescription)
         {
            FloatingPlanarJointDescription floatingPlanarJointDescription = (FloatingPlanarJointDescription) rootJointDescription;

            FloatingPlanarJoint rootJoint = new FloatingPlanarJoint(rootJointDescription.getName(), robot, floatingPlanarJointDescription.getPlane());
         }
         else if (rootJointDescription instanceof PinJointDescription)
         {

         }
      }
   }
}
