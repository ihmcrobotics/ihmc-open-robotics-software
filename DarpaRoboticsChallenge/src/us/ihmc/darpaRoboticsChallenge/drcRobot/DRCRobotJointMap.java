package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

public class DRCRobotJointMap implements SDFJointNameMap, RobotSpecificJointNames
{
   private final String chestName = "utorso";
   private final String pelvisName = "pelvis";
   private final String headName = "head";
   private final double ankleHeight = DRCRobotParameters.DRC_ROBOT_ANKLE_HEIGHT;

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

   private final HashMap<String, JointRole> jointRoles = new HashMap<String, JointRole>();

   private final HashMap<String, Pair<RobotSide, LegJointName>> legJointNames = new HashMap<String, Pair<RobotSide, LegJointName>>();
   private final HashMap<String, Pair<RobotSide, ArmJointName>> armJointNames = new HashMap<String, Pair<RobotSide, ArmJointName>>();
   private final HashMap<String, SpineJointName> spineJointNames = new HashMap<String, SpineJointName>();
   private final HashMap<String, NeckJointName> neckJointNames = new HashMap<String, NeckJointName>();

   private final HashMap<String, Pair<RobotSide, LimbName>> limbNames = new HashMap<String, Pair<RobotSide, LimbName>>();

   private final SideDependentList<String> jointBeforeFeetNames = new SideDependentList<String>();

   private final SideDependentList<List<Pair<String, Vector3d>>> footGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final SideDependentList<List<Pair<String, Vector3d>>> handGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final SideDependentList<List<Pair<String, Vector3d>>> thighGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();
   private final DRCRobotModel selectedModel;

   public DRCRobotJointMap(DRCRobotModel selectedModel)
   {
      this.selectedModel = selectedModel;

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = getRobotSidePrefix(robotSide);

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

         footGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());
         handGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());
         thighGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());

         for (Vector3d footv3d : DRCRobotParameters.DRC_ROBOT_GROUND_CONTACT_POINT_OFFSET_FROM_FOOT)
         {
            // add ankle joint contact points on each corner of the foot
            footGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(prefix + "leg_lax", footv3d));
         }

         if (selectedModel == DRCRobotModel.ATLAS_SANDIA_HANDS)
         {
            // add finger joint contact points offset to the middle of palm-facing side of the finger segment
            String longPrefix = (robotSide == RobotSide.LEFT) ? "left_" : "right_";
            for (double[] fJointContactOffsets : DRCRobotParameters.sandiaFingerContactPointOffsets)
            {
               if (((robotSide == RobotSide.LEFT) && (int) fJointContactOffsets[0] == 0)
                       || ((robotSide == RobotSide.RIGHT) && (int) fJointContactOffsets[0] == 1))
               {
                  handGroundContactPoints.get(robotSide).add(new Pair<String,
                          Vector3d>(longPrefix + "f" + (int) fJointContactOffsets[1] + "_j" + (int) fJointContactOffsets[2],
                                    new Vector3d(fJointContactOffsets[3], fJointContactOffsets[4], fJointContactOffsets[5])));
               }
            }

            // add wrist joint contact point on finger-facing side of palm
            double wcx = DRCRobotParameters.sandiaWristContactPointOffsets[0];
            double wcy = DRCRobotParameters.sandiaWristContactPointOffsets[1] * ((robotSide == RobotSide.RIGHT) ? -1 : 1);
            double wcz = DRCRobotParameters.sandiaWristContactPointOffsets[2];
            handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeHand(robotSide), new Vector3d(wcx, wcy, wcz)));
         }
         else if (selectedModel == DRCRobotModel.ATLAS_IROBOT_HANDS)
         {
            // add finger joint contact points offset to the middle of palm-facing side of the finger segment
            String longPrefix = (robotSide == RobotSide.LEFT) ? "left_" : "right_";
            for (double[] fJointContactOffsets : DRCRobotParameters.irobotFingerContactPointOffsets)
            {
               if (((robotSide == RobotSide.LEFT) && (int) fJointContactOffsets[0] == 0)
                       || ((robotSide == RobotSide.RIGHT) && (int) fJointContactOffsets[0] == 1))
               {
                  // finger[0]/joint_base
                  // finger[0]/flexible_joint_flex_from_9_to_distal
                  handGroundContactPoints.get(robotSide).add(new Pair<String,
                          Vector3d>(longPrefix + "finger[" + (int) fJointContactOffsets[1]
                                    + ((fJointContactOffsets[2] == 0.0)
                                       ? "]joint_base" : "]flexible_joint_flex_from_9_to_distal"), new Vector3d(fJointContactOffsets[3],
                                          fJointContactOffsets[4], fJointContactOffsets[5])));
               }
            }

            // add wrist joint contact point on finger-facing side of palm
            double wcx = DRCRobotParameters.irobotWristContactPointOffsets[0];
            double wcy = DRCRobotParameters.irobotWristContactPointOffsets[1] * ((robotSide == RobotSide.RIGHT) ? -1 : 1);
            double wcz = DRCRobotParameters.irobotWristContactPointOffsets[2];
            handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeHand(robotSide), new Vector3d(wcx, wcy, wcz)));
         }
         else if (selectedModel == DRCRobotModel.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS)
         {
            for (Point2d point : DRCRobotParameters.invisibleContactablePlaneHandContactPoints.get(robotSide))
            {
               Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
               DRCRobotParameters.invisibleContactablePlaneHandContactPointTransforms.get(robotSide).transform(point3d);
               handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeHand(robotSide), new Vector3d(point3d)));
            }
         }
         
         // add butt contact points on back of thighs
         for (Point2d point : DRCRobotParameters.thighContactPoints.get(robotSide))
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            DRCRobotParameters.thighContactPointTransforms.get(robotSide).transform(point3d);
            thighGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeThigh(robotSide), new Vector3d(point3d)));
         }
      }
      

      
      
      spineJointNames.put("back_lbz", SpineJointName.SPINE_YAW);
      spineJointNames.put("back_mby", SpineJointName.SPINE_PITCH);
      spineJointNames.put("back_ubx", SpineJointName.SPINE_ROLL);

      neckJointNames.put("neck_ay", NeckJointName.LOWER_NECK_PITCH);


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

      for (RobotSide robotSide : RobotSide.values())
      {
         jointNameGroundContactPointMap.addAll(footGroundContactPoints.get(robotSide));
         jointNameGroundContactPointMap.addAll(handGroundContactPoints.get(robotSide));
         jointNameGroundContactPointMap.addAll(thighGroundContactPoints.get(robotSide));
      }
   }

   private String getNameOfJointBeforeHand(RobotSide robotSide)
   {
      return getRobotSidePrefix(robotSide) + "arm_mwx";
   }

   public String getNameOfJointBeforeThigh(RobotSide robotSide)
   {
      return getRobotSidePrefix(robotSide) + "leg_lhy";
   }

   private String getRobotSidePrefix(RobotSide robotSide)
   {
      return (robotSide == RobotSide.LEFT) ? "l_" : "r_";
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

   public List<Pair<String, Vector3d>> getFootContactPoints(RobotSide robotSide)
   {
      return footGroundContactPoints.get(robotSide);
   }

   public List<Pair<String, Vector3d>> getThighContactPoints(RobotSide robotSide)
   {
      return thighGroundContactPoints.get(robotSide);
   }

   public List<Pair<String, Vector3d>> getHandContactPoints(RobotSide robotSide)
   {
      return handGroundContactPoints.get(robotSide);
   }

   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   public DRCRobotModel getSelectedModel()
   {
      return selectedModel;
   }
}
