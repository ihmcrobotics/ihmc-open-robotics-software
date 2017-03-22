package us.ihmc.robotics.partNames;

import java.util.Set;

public interface JointNameMap extends RobotSpecificJointNames
{

   String getModelName();

   default double getModelScale()
   {
      return 1.0;
   }

   default double getMassScalePower()
   {
      return 3.0;
   }

   /**
    * @return list of joints that will not be inertia scaled for simulation stability
    */
   default String[] getHighInertiaForStableSimulationJoints()
   {
      return new String[0];
   }

   JointRole getJointRole(String jointName);

   NeckJointName getNeckJointName(String jointName);

   SpineJointName getSpineJointName(String jointName);

   String getPelvisName();

   String getUnsanitizedRootJointInSdf();

   String getChestName();

   String getHeadName();

   boolean isTorqueVelocityLimitsEnabled();

   Set<String> getLastSimulatedJoints();

   String[] getJointNamesBeforeFeet();

   Enum<?>[] getRobotSegments();

   Enum<?> getEndEffectorsRobotSegment(String joineNameBeforeEndEffector);

}