package us.ihmc.valkyrie;

import us.ihmc.avatar.initialSetup.HumanoidRobotMutableInitialSetup;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ValkyrieMutableInitialSetup extends HumanoidRobotMutableInitialSetup
{
   public ValkyrieMutableInitialSetup(HumanoidJointNameMap jointMap)
   {
      super(jointMap);
   }

   public void setLegJointQs(double hipYaw, double hipRoll, double hipPitch, double knee, double anklePitch, double ankleRoll)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         setLegJointQs(robotSide,
                       robotSide.negateIfRightSide(hipYaw),
                       robotSide.negateIfRightSide(hipRoll),
                       hipPitch,
                       knee,
                       anklePitch,
                       robotSide.negateIfRightSide(ankleRoll));
      }
   }

   public void setLegJointQs(RobotSide robotSide, double hipYaw, double hipRoll, double hipPitch, double knee, double anklePitch, double ankleRoll)
   {
      setJoint(robotSide, LegJointName.HIP_YAW, hipYaw);
      setJoint(robotSide, LegJointName.HIP_ROLL, hipRoll);
      setJoint(robotSide, LegJointName.HIP_PITCH, hipPitch);
      setJoint(robotSide, LegJointName.KNEE_PITCH, knee);
      setJoint(robotSide, LegJointName.ANKLE_PITCH, anklePitch);
      setJoint(robotSide, LegJointName.ANKLE_ROLL, ankleRoll);
   }

   public void setArmJointQs(double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowPitch)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         setArmJointQs(robotSide, shoulderPitch, robotSide.negateIfRightSide(shoulderRoll), shoulderYaw, robotSide.negateIfRightSide(elbowPitch));
      }
   }

   public void setArmJointQs(RobotSide robotSide, double shoulderPitch, double shoulderRoll, double shoulderYaw, double elbowPitch)
   {
      setJoint(robotSide, ArmJointName.SHOULDER_PITCH, shoulderPitch);
      setJoint(robotSide, ArmJointName.SHOULDER_ROLL, shoulderRoll);
      setJoint(robotSide, ArmJointName.SHOULDER_YAW, shoulderYaw);
      setJoint(robotSide, ArmJointName.ELBOW_PITCH, elbowPitch);
   }

   public void setSpineJointQs(double qYaw, double qPitch, double qRoll)
   {
      setJoint(SpineJointName.SPINE_YAW, qYaw);
      setJoint(SpineJointName.SPINE_PITCH, qPitch);
      setJoint(SpineJointName.SPINE_ROLL, qRoll);
   }

   public void setRootJointPose(double x, double y, double z, double yaw, double pitch, double roll)
   {
      rootJointPosition.set(x, y, z);
      rootJointOrientation.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setRootJointPose(double x, double y, double z, double qx, double qy, double qz, double qs)
   {
      rootJointPosition.set(x, y, z);
      rootJointOrientation.set(qx, qy, qz, qs);
   }
}