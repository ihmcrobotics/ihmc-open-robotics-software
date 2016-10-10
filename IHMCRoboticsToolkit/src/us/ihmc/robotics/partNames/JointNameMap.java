package us.ihmc.robotics.partNames;

import java.util.List;
import java.util.Set;

import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;

public interface JointNameMap extends RobotSpecificJointNames
{

   String getModelName();

   JointRole getJointRole(String jointName);

   NeckJointName getNeckJointName(String jointName);

   SpineJointName getSpineJointName(String jointName);

   String getPelvisName();

   String getUnsanitizedRootJointInSdf();

   String getChestName();

   String getHeadName();

   List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap();

   boolean isTorqueVelocityLimitsEnabled();

   Set<String> getLastSimulatedJoints();

   String[] getJointNamesBeforeFeet();

   Enum<?>[] getRobotSegments();

   Enum<?> getEndEffectorsRobotSegment(String joineNameBeforeEndEffector);

}