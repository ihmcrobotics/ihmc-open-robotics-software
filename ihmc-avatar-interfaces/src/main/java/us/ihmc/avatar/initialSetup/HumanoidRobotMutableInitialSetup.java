package us.ihmc.avatar.initialSetup;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class HumanoidRobotMutableInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   public final Point3D rootJointPosition = new Point3D();
   public final Quaternion rootJointOrientation = new Quaternion();
   public final Map<String, Double> jointPositions = new HashMap<>();
   public final HumanoidJointNameMap jointMap;

   public HumanoidRobotMutableInitialSetup(HumanoidJointNameMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, HumanoidJointNameMap jointMap)
   {
      robot.getRootJoint().setPosition(rootJointPosition);
      robot.getRootJoint().setOrientation(rootJointOrientation);

      for (Entry<String, Double> entry : jointPositions.entrySet())
      {
         robot.getOneDegreeOfFreedomJoint(entry.getKey()).setQ(entry.getValue());
      }
      robot.update();
   }

   protected void setJoint(RobotSide robotSide, LegJointName legJointName, double q)
   {
      jointPositions.put(jointMap.getLegJointName(robotSide, legJointName), q);
   }

   protected void setJoint(RobotSide robotSide, ArmJointName armJointName, double q)
   {
      jointPositions.put(jointMap.getArmJointName(robotSide, armJointName), q);
   }

   protected void setJoint(SpineJointName spineJointName, double q)
   {
      jointPositions.put(jointMap.getSpineJointName(spineJointName), q);
   }

   protected void setJoint(NeckJointName neckJointName, double q)
   {
      jointPositions.put(jointMap.getNeckJointName(neckJointName), q);
   }

   @Override
   public void setInitialYaw(double yaw)
   {
   }

   @Override
   public double getInitialYaw()
   {
      return 0;
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
   }

   @Override
   public double getInitialGroundHeight()
   {
      return 0;
   }

   @Override
   public void setOffset(Vector3D additionalOffset)
   {
   }

   @Override
   public void getOffset(Vector3D offsetToPack)
   {
   }

   public double getJointPosition(String jointName)
   {
      return jointPositions.get(jointName);
   }

   public Map<String, Double> getJointPositions()
   {
      return jointPositions;
   }

   public Point3D getRootJointPosition()
   {
      return rootJointPosition;
   }

   public Quaternion getRootJointOrientation()
   {
      return rootJointOrientation;
   }
}