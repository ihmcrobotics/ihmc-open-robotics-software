package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

public class DRCRobotJointMap implements SDFJointNameMap, RobotSpecificJointNames
{
   private final String chestName = "utorso";
   private final String pelvisName = "pelvis";
   private final String headName = "head";
   private final double ankleHeight = DRCConfigParameters.DRC_ROBOT_ANKLE_HEIGHT;
   
   private final LegJointName[] legJoints = { LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL };
   private final ArmJointName[] armJoints = { ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.ELBOW_PITCH, ArmJointName.ELBOW_ROLL, ArmJointName.WRIST_PITCH, ArmJointName.WRIST_ROLL };
   private final SpineJointName[] spineJoints = { SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL, SpineJointName.SPINE_YAW };
   private final NeckJointName[] neckJoints = { NeckJointName.LOWER_NECK_PITCH };
   
   private final HashMap<String, JointRole> jointRoles = new HashMap<String, JointRole>();
   
   private final HashMap<String, Pair<RobotSide, LegJointName>> legJointNames = new HashMap<String,Pair<RobotSide, LegJointName>>();
   private final HashMap<String, Pair<RobotSide, ArmJointName>> armJointNames = new HashMap<String,Pair<RobotSide, ArmJointName>>();
   private final HashMap<String, SpineJointName> spineJointNames = new HashMap<String,SpineJointName>();
   private final HashMap<String, NeckJointName> neckJointNames = new HashMap<String,NeckJointName>();
   
   private final HashMap<String, Pair<RobotSide, LimbName>> limbNames = new HashMap<String, Pair<RobotSide,LimbName>>();
   
   private final SideDependentList<String> jointBeforeFeetNames = new SideDependentList<String>();
   
   private final SideDependentList<ArrayList<Vector3d>> groundContactPointOffsetFromFeet = new SideDependentList<ArrayList<Vector3d>>();
   
   public DRCRobotJointMap()
   {
      for(RobotSide robotSide : RobotSide.values)
      {
         String prefix = robotSide==RobotSide.LEFT?"l_":"r_";
         
         legJointNames.put(prefix + "leg_uhz", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_YAW));
         legJointNames.put(prefix + "leg_mhx", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_ROLL));
         legJointNames.put(prefix + "leg_lhy", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.HIP_PITCH));
         legJointNames.put(prefix + "leg_kny", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.KNEE));
         legJointNames.put(prefix + "leg_uay", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_PITCH));
         legJointNames.put(prefix + "leg_lax", new Pair<RobotSide, LegJointName>(robotSide, LegJointName.ANKLE_ROLL));
         
         
         armJointNames.put(prefix + "arm_usy", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_PITCH));
         armJointNames.put(prefix + "arm_shx", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.SHOULDER_ROLL));
         armJointNames.put(prefix + "arm_ely", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_PITCH));
         armJointNames.put(prefix + "arm_elx", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.ELBOW_ROLL));
         armJointNames.put(prefix + "arm_uwy", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_PITCH));
         armJointNames.put(prefix + "arm_mwx", new Pair<RobotSide, ArmJointName>(robotSide, ArmJointName.WRIST_ROLL));
         
         limbNames.put(prefix + "hand", new Pair<RobotSide, LimbName>(robotSide, LimbName.ARM));
         limbNames.put(prefix + "foot", new Pair<RobotSide, LimbName>(robotSide, LimbName.LEG));
         
         jointBeforeFeetNames.put(robotSide, prefix + "leg_lax");
         
         
//         ArrayList<Vector3d> groundContactPointOffsetFromFoot = new ArrayList<Vector3d>();
//         
//         groundContactPointOffsetFromFoot.add(new Vector3d(-0.07, -0.06, -ankleHeight));
//         groundContactPointOffsetFromFoot.add(new Vector3d(-0.07, 0.06, -ankleHeight));
//         groundContactPointOffsetFromFoot.add(new Vector3d(0.18, -0.06, -ankleHeight));
//         groundContactPointOffsetFromFoot.add(new Vector3d(0.18, 0.06, -ankleHeight));
         
         groundContactPointOffsetFromFeet.put(robotSide, DRCConfigParameters.DRC_ROBOT_GROUND_CONTACT_POINT_OFFSET_FROM_FOOT);
         
      }
      
      spineJointNames.put("back_lbz", SpineJointName.SPINE_YAW);
      spineJointNames.put("back_mby", SpineJointName.SPINE_PITCH);
      spineJointNames.put("back_ubx", SpineJointName.SPINE_ROLL);
      
      neckJointNames.put("neck_ay", NeckJointName.LOWER_NECK_PITCH);
      
      
      for(String legJoint : legJointNames.keySet())
      {
         jointRoles.put(legJoint, JointRole.LEG);
      }
      
      for(String armJoint : armJointNames.keySet())
      {
         jointRoles.put(armJoint, JointRole.ARM);
      }
      
      for(String spineJoint : spineJointNames.keySet())
      {
         jointRoles.put(spineJoint, JointRole.SPINE);
      }
      
      for(String neckJoint : neckJointNames.keySet())
      {
         jointRoles.put(neckJoint, JointRole.NECK);
      }
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



   public List<Vector3d> getGroundContactPointOffset(RobotSide robotSide)
   {
      return groundContactPointOffsetFromFeet.get(robotSide);
   }



   public double getAnkleHeight()
   {
      return ankleHeight;
   }
}
