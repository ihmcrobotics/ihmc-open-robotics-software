package us.ihmc.atlas;

import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.back_bkz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.forcedSideDependentJointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.jointNames;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_elx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_ely;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_shy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wrx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_arm_wry;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_akx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_aky;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpx;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpy;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_hpz;
import static us.ihmc.atlas.ros.AtlasOrderedJointMap.l_leg_kny;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.atlas.parameters.AtlasContactPointParamaters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.ros.AtlasOrderedJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;

public class AtlasJointMap extends DRCRobotJointMap 
{
   SideDependentList<String> jointBeforeThighNames = new SideDependentList<String>("l_leg_hpy","r_leg_hpy");
   public static final String chestName = "utorso";
   public static final String pelvisName = "pelvis";
   public static final String headName = "head";
   
   private final LegJointName[] legJoints =
   {
      LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL
   };
   private final ArmJointName[] armJoints =
   {
      ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.ELBOW_PITCH, ArmJointName.ELBOW_ROLL, ArmJointName.WRIST_PITCH,
      ArmJointName.WRIST_ROLL
   };
   private final SpineJointName[] spineJoints = {SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL, SpineJointName.SPINE_YAW};
   private final NeckJointName[] neckJoints = {NeckJointName.LOWER_NECK_PITCH};
   private final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<String, JointRole>();
   private final LinkedHashMap<String, Pair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<String, Pair<RobotSide, LegJointName>>();
   private final LinkedHashMap<String, Pair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<String, Pair<RobotSide, ArmJointName>>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<String, SpineJointName>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<String, NeckJointName>();
   private final LinkedHashMap<String, Pair<RobotSide, LimbName>> limbNames = new LinkedHashMap<String, Pair<RobotSide, LimbName>>();
   private final SideDependentList<String> jointBeforeFeetNames = new SideDependentList<String>();
   private final AtlasContactPointParamaters contactPointParamaters;
   private final AtlasRobotVersion atlasVersion;
   
   
   public AtlasJointMap(AtlasRobotVersion atlasVersion)
   {

      this.atlasVersion = atlasVersion;
      for (RobotSide robotSide : RobotSide.values)
      {
         String[] forcedSideJointNames = forcedSideDependentJointNames.get(robotSide);
         legJointNames.put(forcedSideJointNames[l_leg_hpz], new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(forcedSideJointNames[l_leg_hpx], new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_ROLL));
         legJointNames.put(forcedSideJointNames[l_leg_hpy], new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_kny], new Pair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE));
         legJointNames.put(forcedSideJointNames[l_leg_aky], new Pair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_PITCH));
         legJointNames.put(forcedSideJointNames[l_leg_akx], new Pair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));

         armJointNames.put(forcedSideJointNames[l_arm_shy], new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_PITCH));
         armJointNames.put(forcedSideJointNames[l_arm_shx], new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_ROLL));
         armJointNames.put(forcedSideJointNames[l_arm_ely], new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_PITCH));
         armJointNames.put(forcedSideJointNames[l_arm_elx], new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_ROLL));
         armJointNames.put(forcedSideJointNames[l_arm_wry], new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_PITCH));
         armJointNames.put(forcedSideJointNames[l_arm_wrx], new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_ROLL));

         String prefix = getRobotSidePrefix(robotSide);

         limbNames.put(prefix + "hand", new Pair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         limbNames.put(prefix + "foot", new Pair<RobotSide, LimbName>(robotSide, LimbName.LEG));
         jointBeforeFeetNames.put(robotSide, forcedSideJointNames[l_leg_akx]);
       }
      spineJointNames.put(jointNames[back_bkz], SpineJointName.SPINE_YAW);
      spineJointNames.put(jointNames[back_bky], SpineJointName.SPINE_PITCH);
      spineJointNames.put(jointNames[back_bkx], SpineJointName.SPINE_ROLL);
      neckJointNames.put("neck_ry", NeckJointName.LOWER_NECK_PITCH);

      for (String legJoint : legJointNames.keySet())
      {
         jointRoles.put(legJoint, JointRole.LEG);
      }

      for (String armJoint : armJointNames.keySet())
      {
         jointRoles.put(armJoint, JointRole.ARM);
      }

      for (String spineJoint : spineJointNames.keySet())
      {
         jointRoles.put(spineJoint, JointRole.SPINE);
      }

      for (String neckJoint : neckJointNames.keySet())
      {
         jointRoles.put(neckJoint, JointRole.NECK);
      }

      contactPointParamaters = new AtlasContactPointParamaters(atlasVersion, this, false, false);
   }
   
   @Override
   public String getNameOfJointBeforeHand(RobotSide robotSide)
   {
      return forcedSideDependentJointNames.get(robotSide)[l_arm_wrx];
   }

   @Override
   public String getNameOfJointBeforeThigh(RobotSide robotSide)
   {
      return forcedSideDependentJointNames.get(robotSide)[l_leg_hpy];
   }

   @Override
   public String getNameOfJointBeforeChest()
   {
      return jointNames[back_bkx];
   }

   private String getRobotSidePrefix(RobotSide robotSide)
   {
      return (robotSide == RobotSide.LEFT) ? "l_" : "r_";
   }

   @Override
   public Pair<RobotSide, LegJointName> getLegJointName(String jointName)
   {
      return legJointNames.get(jointName);
   }

   @Override
   public Pair<RobotSide, ArmJointName> getArmJointName(String jointName)
   {
      return armJointNames.get(jointName);
   }

   @Override
   public Pair<RobotSide, LimbName> getLimbName(String limbName)
   {
      return limbNames.get(limbName);
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
   }

   @Override
   public NeckJointName getNeckJointName(String jointName)
   {
      return neckJointNames.get(jointName);
   }

   @Override
   public SpineJointName getSpineJointName(String jointName)
   {
      return spineJointNames.get(jointName);
   }

   @Override
   public String getPelvisName()
   {
      return pelvisName;
   }

   @Override
   public String getChestName()
   {
      return chestName;
   }

   @Override
   public String getHeadName()
   {
      return headName;
   }

   @Override
   public LegJointName[] getLegJointNames()
   {
      return legJoints;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      return armJoints;
   }

   public SpineJointName[] getSpineJointNames()
   {
      return spineJoints;
   }

   @Override
   public NeckJointName[] getNeckJointNames()
   {
      return neckJoints;
   }

   @Override
   public String getJointBeforeFootName(RobotSide robotSide)
   {
      return jointBeforeFeetNames.get(robotSide);
   }

//   public List<Pair<String, Vector3d>> getHandContactPoints(RobotSide robotSide)
//   {
//      return contactPointParamaters.getHandContactPoints(robotSide);
//   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return contactPointParamaters.getJointNameGroundContactPointMap();
   }

   @Override
   public String getModelName()
   {
      return atlasVersion.getModelName();
   }

   @Override
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return AtlasContactPointParamaters.ENABLE_JOINT_VELOCITY_TORQUE_LIMITS;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<>();
      lastSimulatedJoints.add("l_arm_wrx");
      lastSimulatedJoints.add("r_arm_wrx");
      return lastSimulatedJoints;
   }

   @Override
   public SideDependentList<String> getJointBeforeThighNames()
   {
      return jointBeforeThighNames;
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return AtlasOrderedJointMap.jointNames;
   }
   
   @Override
   public String getHighestNeckPitchJointName()
   {
	   return "neck_ry";
   }

   @Override
   public SideDependentList<Transform3D> getAnkleToSoleFrameTransform()
   {
      return new SideDependentList<Transform3D>(AtlasPhysicalProperties.ankle_to_sole_frame_tranform,
    		  AtlasPhysicalProperties.ankle_to_sole_frame_tranform);
   }
   
//   @Override
//   public SideDependentList<ArrayList<Point2d>> getFootGroundContactPointsInSoleFrameForController()
//   {
//	   return contactPointParamaters.getFootGroundContactPointsInSoleFrameForController();
//   }
}
