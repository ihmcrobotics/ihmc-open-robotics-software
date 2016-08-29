package us.ihmc.llaQuadruped;

import java.util.*;

import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.SDFQuadrupedJointNameMap;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

public class LLAQuadrupedJointNameMap implements SDFQuadrupedJointNameMap
{
   private final String modelName = "llaQuadruped";
   private final String rootJoint = "body";

   private final LegJointName[] legJointNames = {LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE};
   private final String[] jointNamesBeforeFeet = new String[4];

   private final HashMap<String, QuadrupedJointName> quadrupedJointNameMap = new HashMap<>();
   private final HashMap<QuadrupedJointName, String> sdfJointNameMap = new HashMap<>();
   private final HashMap<String, JointRole> jointRoles = new HashMap<>();

   private final List<ImmutablePair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<>();
   private final QuadrantDependentList<HashMap<LegJointName, String>> mapFromLegJointNameToJointId = new QuadrantDependentList<>();

   public LLAQuadrupedJointNameMap(LLAQuadrupedPhysicalProperties physicalProperties)
   {
      quadrupedJointNameMap.put("front_left_hip_roll", QuadrupedJointName.FRONT_LEFT_HIP_ROLL);
      quadrupedJointNameMap.put("front_left_hip_pitch", QuadrupedJointName.FRONT_LEFT_HIP_PITCH);
      quadrupedJointNameMap.put("front_left_knee_pitch", QuadrupedJointName.FRONT_LEFT_KNEE_PITCH);
      quadrupedJointNameMap.put("front_right_hip_roll", QuadrupedJointName.FRONT_RIGHT_HIP_ROLL);
      quadrupedJointNameMap.put("front_right_hip_pitch", QuadrupedJointName.FRONT_RIGHT_HIP_PITCH);
      quadrupedJointNameMap.put("front_right_knee_pitch", QuadrupedJointName.FRONT_RIGHT_KNEE_PITCH);
      quadrupedJointNameMap.put("hind_left_hip_roll", QuadrupedJointName.HIND_LEFT_HIP_ROLL);
      quadrupedJointNameMap.put("hind_left_hip_pitch", QuadrupedJointName.HIND_LEFT_HIP_PITCH);
      quadrupedJointNameMap.put("hind_left_knee_pitch", QuadrupedJointName.HIND_LEFT_KNEE_PITCH);
      quadrupedJointNameMap.put("hind_right_hip_roll", QuadrupedJointName.HIND_RIGHT_HIP_ROLL);
      quadrupedJointNameMap.put("hind_right_hip_pitch", QuadrupedJointName.HIND_RIGHT_HIP_PITCH);
      quadrupedJointNameMap.put("hind_right_knee_pitch", QuadrupedJointName.HIND_RIGHT_KNEE_PITCH);

      for (Map.Entry<String, QuadrupedJointName> entry : quadrupedJointNameMap.entrySet())
      {
         sdfJointNameMap.put(entry.getValue(), entry.getKey());
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         mapFromLegJointNameToJointId.set(robotQuadrant, new HashMap<LegJointName, String>());
      }

      for (LLAQuadrupedOrderedJointMap joint : LLAQuadrupedOrderedJointMap.values())
      {
         String jointName = joint.getName();
         jointRoles.put(jointName, JointRole.LEG);

         RobotQuadrant robotQuadrant = joint.getRobotQuadrant();
         if (robotQuadrant != null)
         {
            HashMap<LegJointName, String> jointIdToLegNameMap = mapFromLegJointNameToJointId.get(robotQuadrant);
            jointIdToLegNameMap.put(joint.getLegJointName(), joint.getName());
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(getJointBeforeFootName(robotQuadrant), physicalProperties.getOffsetFromJointBeforeFootToSole(robotQuadrant)));
         Vector3d shoulderContactOffset = new Vector3d(robotQuadrant.getEnd().negateIfHindEnd(0.08), robotQuadrant.getSide().negateIfRightSide(0.03), 0.02); // // FIXME: 6/2/16 
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(getLegJointName(robotQuadrant, LegJointName.HIP_ROLL), shoulderContactOffset));
      }

      jointNamesBeforeFeet[0] = getLegJointName(RobotQuadrant.FRONT_LEFT, LegJointName.KNEE);
      jointNamesBeforeFeet[1] = getLegJointName(RobotQuadrant.FRONT_RIGHT, LegJointName.KNEE);
      jointNamesBeforeFeet[2] = getLegJointName(RobotQuadrant.HIND_LEFT, LegJointName.KNEE);
      jointNamesBeforeFeet[3] = getLegJointName(RobotQuadrant.HIND_RIGHT, LegJointName.KNEE);
   }

   public Collection<QuadrupedJointName> getQuadrupedJointNames()
   {
      return quadrupedJointNameMap.values();
   }
   
   @Override
   public String getModelName()
   {
      return modelName;
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
   }

   @Override
   public NeckJointName getNeckJointName(String jointName)
   {
      return null;
   }

   @Override
   public SpineJointName getSpineJointName(String jointName)
   {
      return null;
   }

   @Override
   public String getPelvisName()
   {
      return rootJoint;
   }

   @Override
   public String getUnsanitizedRootJointInSdf()
   {
      return rootJoint;
   }

   @Override
   public String getChestName()
   {
      return null;
   }

   @Override
   public String getHeadName()
   {
      return null;
   }

   @Override
   public List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         lastSimulatedJoints.add(getJointBeforeFootName(robotQuadrant));
      }
      return lastSimulatedJoints;
   }

   @Override
   public String[] getJointNamesBeforeFeet()
   {
      return jointNamesBeforeFeet;
   }

   @Override
   public LegJointName[] getLegJointNames()
   {
      return legJointNames;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      return null;
   }

   @Override
   public SpineJointName[] getSpineJointNames()
   {
      return null;
   }

   @Override
   public NeckJointName[] getNeckJointNames()
   {
      return new NeckJointName[0];
   }

   @Override
   public String getLegJointName(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      HashMap<LegJointName, String> legJointMap = mapFromLegJointNameToJointId.get(robotQuadrant);
      return legJointMap.get(legJointName);
   }

   @Override
   public String getJointBeforeFootName(RobotQuadrant robotQuadrant)
   {
      HashMap<LegJointName, String> legJointMap = mapFromLegJointNameToJointId.get(robotQuadrant);
      return legJointMap.get(LegJointName.KNEE);
   }

   @Override
   public QuadrupedJointName getJointNameForSDFName(String name)
   {
      return quadrupedJointNameMap.get(name);
   }

   @Override
   public String getSDFNameForJointName(QuadrupedJointName name)
   {
      return sdfJointNameMap.get(name);
   }
   
   @Override
   public Enum<?>[] getRobotSegments()
   {
      return RobotQuadrant.values;
   }
}
