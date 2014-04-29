package us.ihmc.valkyrie.paramaters;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieJointMap extends DRCRobotJointMap 
{
   public static final String[] forceSensorNames = { "LeftAnkle", "RightAnkle", "LeftForearmSupinator", "RightForearmSupinator" };
   public static final SideDependentList<String> jointBeforeThighNames = new SideDependentList<String>("LeftHipExtensor","RightHipExtensor");
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("LeftAnkle", "RightAnkle");
   public static final String chestName = "v1Trunk";
   public static final String pelvisName = "v1Pelvis";
   public static final String pelvisPitchJointName = "WaistExtensor";
   public static final String pelvisYawJointName = "WaistRotator";
   public static final String lowerNeckPitchJointName = "LowerNeckExtensor";
   public static final String upperNeckPitchJointName = "UpperNeckExtensor";
   public static final String neckYawJointName = "NeckRotator";
   public static final String headName = "v1Head";
   public static final String lidarSensorName = "/v1/Ibeo_sensor";
   public static final String lidarJointName = "";
   public static final String leftCameraName = "/v1/LeftHazardCamera___default__";
   public static final String rightCameraName ="/v1/RightHazardCamera___default__";
   public static final String rightTrunkIMUSensor = "v1Trunk_RightIMU";
   public static final String leftTrunkIMUSensor = "v1Trunk_LeftIMU";
   public static final String leftPelvisIMUSensor = "v1Pelvis_LeftIMU";
   public static final String rightPelvisIMUSensor = "v1Pelvis_RightIMU";
   public static final String fakePelvisIMUSensor = "v1Pelvis_SimulatedIMU";

   // Use this until sim can handle multiple IMUs
   // public static final String[] imuSensorsToUse = {leftPelvisIMUSensor, rightPelvisIMUSensor};
   public static final String[] imuSensorsToUse = {leftPelvisIMUSensor};

   private final LegJointName[] legJoints = { LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE, LegJointName.ANKLE_PITCH,
         LegJointName.ANKLE_ROLL };
   private final ArmJointName[] armJoints = { ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.SHOULDER_YAW, ArmJointName.ELBOW_PITCH,
         ArmJointName.ELBOW_YAW, ArmJointName.WRIST_ROLL, ArmJointName.WRIST_PITCH };
   private final SpineJointName[] spineJoints = { SpineJointName.SPINE_YAW, SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL };
   private final NeckJointName[] neckJoints = { NeckJointName.LOWER_NECK_PITCH, NeckJointName.NECK_YAW, NeckJointName.UPPER_NECK_PITCH };

   private final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<String, JointRole>();
   private final LinkedHashMap<String, Pair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<String, Pair<RobotSide, LegJointName>>();
   private final LinkedHashMap<String, Pair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<String, Pair<RobotSide, ArmJointName>>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<String, SpineJointName>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<String, NeckJointName>();
   private final LinkedHashMap<String, Pair<RobotSide, LimbName>> limbNames = new LinkedHashMap<String, Pair<RobotSide, LimbName>>();
   private final SideDependentList<String> jointBeforeFeetNames = new SideDependentList<String>();
   
   private final ValkyrieContactPointParamaters contactPointParameters;

   public ValkyrieJointMap()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         legJointNames
               .put(robotSide.getCamelCaseNameForMiddleOfExpression() + "HipRotator", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "HipAdductor", new Pair<RobotSide, LegJointName>(robotSide,
               LegJointName.HIP_ROLL));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "HipExtensor", new Pair<RobotSide, LegJointName>(robotSide,
               LegJointName.HIP_PITCH));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "KneeExtensor", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "AnkleExtensor", new Pair<RobotSide, LegJointName>(robotSide,
               LegJointName.ANKLE_PITCH));
         legJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "Ankle", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));

         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ShoulderExtensor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.SHOULDER_PITCH));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ShoulderAdductor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.SHOULDER_ROLL));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ShoulderSupinator", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.SHOULDER_YAW));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ElbowExtensor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.ELBOW_PITCH));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "ForearmSupinator", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.ELBOW_YAW));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "WristExtensor", new Pair<RobotSide, ArmJointName>(robotSide,
               ArmJointName.WRIST_ROLL));
         armJointNames.put(robotSide.getCamelCaseNameForMiddleOfExpression() + "Wrist", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_PITCH));

         String prefix = getRobotSidePrefix(robotSide);

         limbNames.put(prefix + "Palm", new Pair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         limbNames.put(prefix + "UpperFoot", new Pair<RobotSide, LimbName>(robotSide, LimbName.LEG));

         jointBeforeFeetNames.put(robotSide, robotSide.getCamelCaseNameForMiddleOfExpression() + "Ankle");
      }

      spineJointNames.put("WaistRotator", SpineJointName.SPINE_YAW);
      spineJointNames.put("WaistExtensor", SpineJointName.SPINE_PITCH);
      spineJointNames.put("WaistLateralExtensor", SpineJointName.SPINE_ROLL);

      neckJointNames.put("LowerNeckExtensor", NeckJointName.LOWER_NECK_PITCH);
      neckJointNames.put("NeckRotator", NeckJointName.NECK_YAW);
      neckJointNames.put("UpperNeckExtensor", NeckJointName.UPPER_NECK_PITCH);

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
      contactPointParameters = new ValkyrieContactPointParamaters(this);
   }
   
   public String getHighestNeckPitchJointName()
   {
      return lowerNeckPitchJointName;
   }
   
   public double getPelvisToFoot()
   {
      return ValkyriePhysicalProperties.pelvisToFoot;
   }

   public String getNameOfJointBeforeHand(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForMiddleOfExpression() + "Wrist";
   }

   public String getNameOfJointBeforeThigh(RobotSide robotSide)
   {
      return robotSide.getCamelCaseNameForMiddleOfExpression() + "HipExtensor";
   }

   public String getNameOfJointBeforeChest()
   {
      return "WaistLateralExtensor";
   }

   private String getRobotSidePrefix(RobotSide robotSide)
   {
      return (robotSide == RobotSide.LEFT) ? "v1Left" : "v1Right";
   }

   public Pair<RobotSide, LegJointName> getLegJointName(String jointName)
   {
      return legJointNames.get(jointName);
   }

   public Pair<RobotSide, ArmJointName> getArmJointName(String jointName)
   {
      return armJointNames.get(jointName);
   }

   public Pair<RobotSide, LimbName> getLimbName(String limbName)
   {
      return limbNames.get(limbName);
   }

   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
   }

   public NeckJointName getNeckJointName(String jointName)
   {
      return neckJointNames.get(jointName);
   }

   public SpineJointName getSpineJointName(String jointName)
   {
      return spineJointNames.get(jointName);
   }

   public String getPelvisName()
   {
      return pelvisName;
   }

   public String getChestName()
   {
      return chestName;
   }

   public String getHeadName()
   {
      return headName;
   }

   public LegJointName[] getLegJointNames()
   {
      return legJoints;
   }

   public ArmJointName[] getArmJointNames()
   {
      return armJoints;
   }

   public SpineJointName[] getSpineJointNames()
   {
      return spineJoints;
   }

   public NeckJointName[] getNeckJointNames()
   {
      return neckJoints;
   }

   public String getJointBeforeFootName(RobotSide robotSide)
   {
      return jointBeforeFeetNames.get(robotSide);
   }

   public double getAnkleHeight()
   {
      return ValkyriePhysicalProperties.ankleHeight;
   }

   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return contactPointParameters.getJointNameGroundContactPointMap();
   }

   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   public String getLeftCameraName()
   {
      return leftCameraName;
   }

   public String getLidarSensorName()
   {
      return lidarSensorName;
   }

   public String getRightCameraName()
   {
      return rightCameraName;
   }

   public String[] getIMUSensorsToUse()
   {
      return imuSensorsToUse;
   }
   

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<>();
      lastSimulatedJoints.add("LeftWrist");
      lastSimulatedJoints.add("RightWrist");
      return lastSimulatedJoints;
   }

   @Override
   public String getLidarJointName()
   {
      return lidarJointName;
   }

   @Override
   public String getModelName()
   {
      return "V1";
   }

   @Override
   public String[] getForceSensorNames()
   {
      return forceSensorNames;
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }
   
   @Override
   public SideDependentList<String> getJointBeforeThighNames()
   {
      return jointBeforeThighNames;
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return ValkyrieOrderedJointMap.jointNames;
   }

   @Override
   public SideDependentList<Transform3D> getAnkleToSoleFrameTransform()
   {
      return ValkyriePhysicalProperties.ankleToSoleFrameTransforms;
   }
}
