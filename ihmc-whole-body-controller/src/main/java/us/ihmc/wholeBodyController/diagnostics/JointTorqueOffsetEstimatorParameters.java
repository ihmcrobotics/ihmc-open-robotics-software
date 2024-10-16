package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.commons.robotics.partNames.ArmJointName;
import us.ihmc.commons.robotics.partNames.LegJointName;
import us.ihmc.commons.robotics.partNames.SpineJointName;

public class JointTorqueOffsetEstimatorParameters
{
   private ArmJointName[] armJointsToRun = {ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH};
   private LegJointName[] legJointsToRun = {LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.HIP_ROLL, LegJointName.KNEE_PITCH,
         LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL};
   private SpineJointName[] spineJointsToRun = {SpineJointName.SPINE_YAW, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL};

   public boolean hasArmJoints()
   {
      return armJointsToRun != null && armJointsToRun.length > 0;
   }

   public boolean hasLegJoints()
   {
      return legJointsToRun != null && legJointsToRun.length > 0;
   }

   public boolean hasSpineJoints()
   {
      return spineJointsToRun != null && spineJointsToRun.length > 0;
   }

   public void setArmJointsToRun(ArmJointName[] armJointsToRun)
   {
      this.armJointsToRun = armJointsToRun;
   }

   public void setLegJointsToRun(LegJointName[] legJointsToRun)
   {
      this.legJointsToRun = legJointsToRun;
   }

   public void setSpineJointsToRun(SpineJointName[] spineJointsToRun)
   {
      this.spineJointsToRun = spineJointsToRun;
   }

   public ArmJointName[] getArmJointsToRun()
   {
      return armJointsToRun;
   }

   public LegJointName[] getLegJointsToRun()
   {
      return legJointsToRun;
   }

   public SpineJointName[] getSpineJointsToRun()
   {
      return spineJointsToRun;
   }
}
