package us.ihmc.robotics.partNames;

import us.ihmc.robotics.robotSide.RobotSegment;

import java.util.Set;

public interface JointNameMap<E extends Enum<E> & RobotSegment<E>> extends RobotSpecificJointNames
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

   default double getDefaultKLimit()
   {
      return 100.0;
   }

   default double getDefaultBLimit()
   {
      return 20.0;
   }

   default double getJointKLimit(String jointName)
   {
      return getDefaultKLimit();
   }
   
   default double getJointBLimit(String jointName)
   {
      return getDefaultBLimit();
   }

   default double getDefaultVelocityLimitDamping()
   {
      return 500.0;
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

   String getRootBodyName();

   String getUnsanitizedRootJointInSdf();

   String getHeadName();

   default boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   Set<String> getLastSimulatedJoints();
   
   default Set<String> getEndEffectorJoints()
   {
      return getLastSimulatedJoints();
   }

   String[] getJointNamesBeforeFeet();

   E[] getRobotSegments();

   E getEndEffectorsRobotSegment(String jointNameBeforeEndEffector);
}