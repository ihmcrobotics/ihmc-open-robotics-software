package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel.RobotType;
import us.ihmc.darpaRoboticsChallenge.IncorrectDrcRobotModelException;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.AtlasAndHandRobotParameters;
import us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap;
import static us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap.*;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

public class AtlasJointMap extends DRCRobotJointMap 
{
   public static final String[] forceSensorNames = { "l_leg_akx", "r_leg_akx", "l_arm_wrx", "r_arm_wrx" };
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("l_leg_akx", "r_leg_akx");
   SideDependentList<String> jointBeforeThighNames = new SideDependentList<String>("l_leg_hpy","r_leg_hpy");
   
   public static final double pelvisToFoot = 0.887;
   
   public static final String chestName = "utorso";
   public static final String pelvisName = "pelvis";
   public static final String headName = "head";
   public static final String lidarJointName = "hokuyo_joint";
   
   public static final String lidarSensorName = "head_hokuyo_sensor";
   public static final String leftCameraName = "stereo_camera_left";
   public static final String rightCameraName = "stereo_camera_right";
   
   public static final String bodyIMUSensor = "pelvis_imu_sensor";
   public static final String[] imuSensorsToUse = { bodyIMUSensor };
   
   private final double ankleHeight = AtlasAndHandRobotParameters.DRC_ROBOT_ANKLE_HEIGHT;

   private final LegJointName[] legJoints =
   {
      LegJointName.HIP_YAW, LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE, LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL
   };
   private final ArmJointName[] armJoints =
   {
      ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.ELBOW_PITCH, ArmJointName.ELBOW_ROLL, ArmJointName.WRIST_PITCH,
      ArmJointName.WRIST_ROLL
   };
   final SpineJointName[] spineJoints = {SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL, SpineJointName.SPINE_YAW};
   private final NeckJointName[] neckJoints = {NeckJointName.LOWER_NECK_PITCH};

   private final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<String, JointRole>();

   private final LinkedHashMap<String, Pair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<String, Pair<RobotSide, LegJointName>>();
   private final LinkedHashMap<String, Pair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<String, Pair<RobotSide, ArmJointName>>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<String, SpineJointName>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<String, NeckJointName>();

   private final LinkedHashMap<String, Pair<RobotSide, LimbName>> limbNames = new LinkedHashMap<String, Pair<RobotSide, LimbName>>();

   private final SideDependentList<String> jointBeforeFeetNames = new SideDependentList<String>();

   private final SideDependentList<List<Pair<String, Vector3d>>> footGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final SideDependentList<List<Pair<String, Vector3d>>> handGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final SideDependentList<List<Pair<String, Vector3d>>> thighGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final List<Pair<String, Vector3d>> pelvisContactPoints = new ArrayList<Pair<String, Vector3d>>();
   private final List<Pair<String, Vector3d>> pelvisBackContactPoints = new ArrayList<Pair<String, Vector3d>>();
   private final List<Pair<String, Vector3d>> chestBackContactPoints = new ArrayList<Pair<String, Vector3d>>();
   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();
   private final DRCRobotModel selectedModel;

   public AtlasJointMap(DRCRobotModel selectedModel, boolean addLoadsOfContactPoints)
   {
      this(selectedModel, addLoadsOfContactPoints, false);
   }
   
   private void checkModel(DRCRobotModel m)
   {
      if(m.getType() != RobotType.ATLAS)
      {
         throw new IncorrectDrcRobotModelException("Loaded robot model is not Atlas model dog!");
      }
   }
   
   public AtlasJointMap(DRCRobotModel selectedModel, boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsForFeetOnly)
   {
      checkModel(selectedModel);
      
      this.selectedModel = selectedModel;

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

         footGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());
         handGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());
         thighGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());

         ArrayList<Point2d> contactPointOffsetList;
         if (addLoadsOfContactPointsForFeetOnly)
            contactPointOffsetList = AtlasAndHandRobotParameters.DRC_ROBOT_GROUND_LOADS_OF_CONTACT_POINT_OFFSET_FROM_FOOT;
         else
            contactPointOffsetList = AtlasAndHandRobotParameters.DRC_ROBOT_GROUND_CONTACT_POINT_OFFSET_FROM_FOOT;
            
         for (Point2d footv3d : contactPointOffsetList)
         {
            // add ankle joint contact points on each corner of the foot
            footGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(forcedSideJointNames[l_leg_akx], new Vector3d(footv3d.getX(), footv3d.getY(), -AtlasAndHandRobotParameters.DRC_ROBOT_ANKLE_HEIGHT)));
         }

         if (selectedModel == DRCRobotModel.ATLAS_SANDIA_HANDS && addLoadsOfContactPoints)
         {
            // add finger joint contact points offset to the middle of palm-facing side of the finger segment
            String longPrefix = (robotSide == RobotSide.LEFT) ? "left_" : "right_";
            for (double[] fJointContactOffsets : AtlasAndHandRobotParameters.sandiaFingerContactPointOffsets)
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
            for(Vector3d offset : AtlasAndHandRobotParameters.sandiaWristContactPointOffsets.get(robotSide))
            {
               handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeHand(robotSide), offset));
            }
         }
         else if ((selectedModel.hasIRobotHands()) && addLoadsOfContactPoints)
         {
            // add finger joint contact points offset to the middle of palm-facing side of the finger segment
            String longPrefix = (robotSide == RobotSide.LEFT) ? "left_" : "right_";
            for (double[] fJointContactOffsets : AtlasAndHandRobotParameters.irobotFingerContactPointOffsets)
            {
               if (((robotSide == RobotSide.LEFT) && (int) fJointContactOffsets[0] == 0)
                       || ((robotSide == RobotSide.RIGHT) && (int) fJointContactOffsets[0] == 1))
               {
                  // finger[0]/joint_base
                  // finger[0]/flexible_joint_flex_from_9_to_distal
                  handGroundContactPoints.get(robotSide).add(new Pair<String,
                          Vector3d>(longPrefix + "finger_" + (int) fJointContactOffsets[1]
                                    + ((fJointContactOffsets[2] == 0.0)
                                       ? "_joint_base" : "_flexible_joint_flex_from_9_to_distal"), new Vector3d(fJointContactOffsets[3],
                                          fJointContactOffsets[4], fJointContactOffsets[5])));
               }
            }

            // add wrist joint contact point on finger-facing side of palm
            for(Vector3d offset : AtlasAndHandRobotParameters.irobotWristContactPointOffsets.get(robotSide))
            {
               handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeHand(robotSide), offset));
            }
         }
         else if (selectedModel == DRCRobotModel.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS)
         {
            for (Point2d point : AtlasAndHandRobotParameters.invisibleContactablePlaneHandContactPoints.get(robotSide))
            {
               Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
               AtlasAndHandRobotParameters.invisibleContactablePlaneHandContactPointTransforms.get(robotSide).transform(point3d);
               handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeHand(robotSide), new Vector3d(point3d)));
            }
         }
         
         // add butt contact points on back of thighs
         if(addLoadsOfContactPoints)
         {
            for (Point2d point : AtlasAndHandRobotParameters.thighContactPoints.get(robotSide))
            {
               Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
               AtlasAndHandRobotParameters.thighContactPointTransforms.get(robotSide).transform(point3d);
               thighGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(getNameOfJointBeforeThigh(robotSide), new Vector3d(point3d)));
            }
         }
      }
      

      if (addLoadsOfContactPoints)
      {
         for (Point2d point : AtlasAndHandRobotParameters.pelvisContactPoints)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            AtlasAndHandRobotParameters.pelvisContactPointTransform.transform(point3d);
            pelvisContactPoints.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
         }

         for (Point2d point : AtlasAndHandRobotParameters.pelvisBackContactPoints)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            AtlasAndHandRobotParameters.pelvisBackContactPointTransform.transform(point3d);
            pelvisBackContactPoints.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
         }
         
         for (Point2d point : AtlasAndHandRobotParameters.chestBackContactPoints)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            AtlasAndHandRobotParameters.chestBackContactPointTransform.transform(point3d);
            chestBackContactPoints.add(new Pair<String, Vector3d>(getNameOfJointBeforeChest(), new Vector3d(point3d)));
         }
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

      for (RobotSide robotSide : RobotSide.values)
      {
         jointNameGroundContactPointMap.addAll(footGroundContactPoints.get(robotSide));
         jointNameGroundContactPointMap.addAll(handGroundContactPoints.get(robotSide));
         jointNameGroundContactPointMap.addAll(thighGroundContactPoints.get(robotSide));
      }
      jointNameGroundContactPointMap.addAll(pelvisContactPoints);
      jointNameGroundContactPointMap.addAll(pelvisBackContactPoints);
      jointNameGroundContactPointMap.addAll(chestBackContactPoints);
   }
   
   @Override
   public double getPelvisToFoot()
   {
      return pelvisToFoot;
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

   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public DRCRobotModel getSelectedModel()
   {
      return selectedModel;
   }

   @Override
   public String getLidarJointName()
   {
      return lidarJointName;
   }

   @Override
   public String getModelName()
   {
      return selectedModel.getModelName();
   }

   @Override
   public boolean enableTorqueVelocityLimits()
   {
      return AtlasAndHandRobotParameters.ENABLE_JOINT_VELOCITY_TORQUE_LIMITS;
   }

   @Override
   public String getLeftCameraName()
   {
      return leftCameraName;
   }

   @Override
   public String getLidarSensorName()
   {
      return lidarSensorName;
   }

   @Override
   public String getRightCameraName()
   {
      return rightCameraName;
   }
   
   @Override
   public String[] getIMUSensorsToUse()
   {
      return imuSensorsToUse;
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
      return ROSAtlasJointMap.jointNames;
   }

   @Override
   public SideDependentList<Transform3D> getAnkleToSoleFrameTransform()
   {
      return new SideDependentList<>(AtlasAndHandRobotParameters.DRC_ROBOT_ANKLE_TO_SOLE_FRAME_TRANFORM,
            AtlasAndHandRobotParameters.DRC_ROBOT_ANKLE_TO_SOLE_FRAME_TRANFORM);
   }

   @Override
   public SideDependentList<ArrayList<Point2d>> getFootGroundContactPointsInSoleFrameForController()
   {
      return new SideDependentList<>(AtlasAndHandRobotParameters.DRC_ROBOT_GROUND_CONTACT_POINT_OFFSET_FROM_FOOT, AtlasAndHandRobotParameters.DRC_ROBOT_GROUND_CONTACT_POINT_OFFSET_FROM_FOOT);
   }
}
