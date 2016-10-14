package us.ihmc.humanoidRobotics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;

public class HumanoidFloatingRootJointRobot extends FloatingRootJointRobot
{
   private final SideDependentList<String> jointsBeforeFeet = new SideDependentList<String>();

   private final SideDependentList<ArrayList<GroundContactPoint>> footGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();
   private final SideDependentList<ArrayList<GroundContactPoint>> handGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();

   public HumanoidFloatingRootJointRobot(RobotDescription robotDescription, HumanoidJointNameMap sdfJointNameMap)
   {
      super(robotDescription);

      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
         handGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
         if(sdfJointNameMap != null)
         {
            jointsBeforeFeet.put(robotSide, sdfJointNameMap.getJointBeforeFootName(robotSide));
         }
      }

      for(Joint joint : getOneDegreeOfFreedomJoints())
      {
         for(RobotSide robotSide : RobotSide.values)
         {
            ArrayList<GroundContactPoint> contactPointsForJoint = getGroundContactPointsOnJoint(joint);

            if(contactPointsForJoint != null)
            {
               String jointName = joint.getName();
               if (jointName.equals(sdfJointNameMap.getJointBeforeFootName(robotSide)))
               {
                  footGroundContactPoints.get(robotSide).addAll(contactPointsForJoint);
               }
               else if (jointName.equals(sdfJointNameMap.getJointBeforeHandName(robotSide)))
               {
                  handGroundContactPoints.get(robotSide).addAll(contactPointsForJoint);
               }
            }
         }
      }
   }

   public List<GroundContactPoint> getFootGroundContactPoints(RobotSide robotSide)
   {
      return footGroundContactPoints.get(robotSide);
   }

   public List<GroundContactPoint> getHandGroundContactPoints(RobotSide robotSide)
   {
      return handGroundContactPoints.get(robotSide);
   }

   public SideDependentList<String> getJointNamesBeforeFeet()
   {
      return jointsBeforeFeet;
   }

}
