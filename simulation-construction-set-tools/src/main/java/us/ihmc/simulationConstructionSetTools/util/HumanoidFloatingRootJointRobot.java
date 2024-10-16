package us.ihmc.simulationConstructionSetTools.util;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;

public class HumanoidFloatingRootJointRobot extends FloatingRootJointRobot
{
   private final SideDependentList<String> jointsBeforeFeet = new SideDependentList<String>();

   private final SideDependentList<ArrayList<GroundContactPoint>> footGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();
   private final SideDependentList<ArrayList<GroundContactPoint>> handGroundContactPoints = new SideDependentList<ArrayList<GroundContactPoint>>();

   public HumanoidFloatingRootJointRobot(RobotDescription robotDescription, HumanoidJointNameMap jointNameMap)
   {
      this(robotDescription, jointNameMap, true, true);
   }

   public HumanoidFloatingRootJointRobot(RobotDescription robotDescription,
                                         HumanoidJointNameMap jointNameMap,
                                         boolean enableDamping,
                                         boolean enableJointTorqueAndVelocityLimits)
   {
      super(robotDescription, enableDamping, enableJointTorqueAndVelocityLimits && (jointNameMap == null || jointNameMap.isTorqueVelocityLimitsEnabled()));
      initialWithJointNameMap(jointNameMap);
   }

   public HumanoidFloatingRootJointRobot(RobotDefinition robotDefinition, HumanoidJointNameMap jointNameMap)
   {
      this(robotDefinition, jointNameMap, true, true);
   }

   public HumanoidFloatingRootJointRobot(RobotDefinition robotDefinition,
                                         HumanoidJointNameMap jointNameMap,
                                         boolean enableDamping,
                                         boolean enableJointTorqueAndVelocityLimits)
   {
      super(robotDefinition, enableDamping, enableJointTorqueAndVelocityLimits && (jointNameMap == null || jointNameMap.isTorqueVelocityLimitsEnabled()));
      initialWithJointNameMap(jointNameMap);
   }

   private void initialWithJointNameMap(HumanoidJointNameMap jointNameMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
         handGroundContactPoints.put(robotSide, new ArrayList<GroundContactPoint>());
         if (jointNameMap != null)
         {
            jointsBeforeFeet.put(robotSide, jointNameMap.getJointBeforeFootName(robotSide));
         }
      }

      for (Joint joint : getOneDegreeOfFreedomJoints())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            ArrayList<GroundContactPoint> contactPointsForJoint = getGroundContactPointsOnJoint(joint);

            if (contactPointsForJoint != null)
            {
               String jointName = joint.getName();
               if (jointName.equals(jointNameMap.getJointBeforeFootName(robotSide)))
               {
                  footGroundContactPoints.get(robotSide).addAll(contactPointsForJoint);
               }
               else if (jointName.equals(jointNameMap.getJointBeforeHandName(robotSide)))
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

   public SideDependentList<String> getJointNamesBeforeFeet()
   {
      return jointsBeforeFeet;
   }

}
