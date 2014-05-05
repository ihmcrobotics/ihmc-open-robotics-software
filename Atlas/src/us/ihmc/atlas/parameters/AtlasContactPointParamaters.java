package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.HandContactParameters;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class AtlasContactPointParamaters extends DRCRobotContactPointParamaters
{
   
// Enable joint limits
   public static boolean ENABLE_JOINT_VELOCITY_TORQUE_LIMITS = false;

   static
   {
      if (!ENABLE_JOINT_VELOCITY_TORQUE_LIMITS)
      {
         System.err.println("Running with torque and velocity limits disabled, do not check in !!");
      }
   }
   
   private final Vector3d pelvisBoxOffset = new Vector3d(-0.100000, 0.000000, -0.050000);
   private final double pelvisBoxSizeX = 0.100000;
   private final double pelvisBoxSizeY = 0.150000;
   private final double pelvisBoxSizeZ = 0.200000;
   private final Transform3D pelvisContactPointTransform = new Transform3D();
   private final List<Point2d> pelvisContacts = new ArrayList<Point2d>();
   private final Transform3D pelvisBackContactPointTransform = new Transform3D();
   private final List<Point2d> pelvisBackContacts = new ArrayList<Point2d>();
   
   private final Vector3d chestBoxOffset = new Vector3d(0.044600, 0.000000, 0.186900);
   private final double chestBoxSizeX = 0.318800;
   private final double chestBoxSizeY = 0.240000;
   private final double chestBoxSizeZ = 0.316200;
   private final Transform3D chestBackContactPointTransform = new Transform3D();
   private final List<Point2d> chestBackContacts = new ArrayList<Point2d>();
   private final SideDependentList<Transform3D> thighContactPointTransforms = new SideDependentList<Transform3D>();
   private final SideDependentList<List<Point2d>> thighContactPoints = new SideDependentList<List<Point2d>>();
   
   
   private final SideDependentList<List<Pair<String, Vector3d>>> footGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final SideDependentList<List<Pair<String, Vector3d>>> handGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final SideDependentList<List<Pair<String, Vector3d>>> thighGroundContactPoints = new SideDependentList<List<Pair<String, Vector3d>>>();
   private final List<Pair<String, Vector3d>> pelvisContactPoints = new ArrayList<Pair<String, Vector3d>>();
   private final List<Pair<String, Vector3d>> pelvisBackContactPoints = new ArrayList<Pair<String, Vector3d>>();
   private final List<Pair<String, Vector3d>> chestBackContactPoints = new ArrayList<Pair<String, Vector3d>>();
   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();
   
   public static final ArrayList<Point2d> ground_contact_point_offset_from_foot = new ArrayList<Point2d>();

   static
   {
      ground_contact_point_offset_from_foot.add(new Point2d(-AtlasPhysicalProperties.foot_back, -(AtlasPhysicalProperties.foot_width / 2.0)));
      ground_contact_point_offset_from_foot.add(new Point2d(-AtlasPhysicalProperties.foot_back, AtlasPhysicalProperties.foot_width / 2.0));
      ground_contact_point_offset_from_foot.add(new Point2d(AtlasPhysicalProperties.foot_forward, -(AtlasPhysicalProperties.toe_width / 2.0)));
      ground_contact_point_offset_from_foot.add(new Point2d(AtlasPhysicalProperties.foot_forward, AtlasPhysicalProperties.toe_width / 2.0));
      //Added contact points between corners
      if (DRCConfigParameters.USE_SIX_CONTACT_POINTS_PER_FOOT)
      {
         ground_contact_point_offset_from_foot.add(new Point2d(AtlasPhysicalProperties.foot_start_toetaper_from_back-AtlasPhysicalProperties.foot_back, -(AtlasPhysicalProperties.foot_width / 2.0)));
         ground_contact_point_offset_from_foot.add(new Point2d(AtlasPhysicalProperties.foot_start_toetaper_from_back-AtlasPhysicalProperties.foot_back, AtlasPhysicalProperties.foot_width / 2.0));
      }
   }
   
   public static final ArrayList<Point2d> ground_loads_of_contact_point_offset_from_foot = new ArrayList<Point2d>();

   static
   {
      int nSubdivisionsX = 3;
      int nSubdivisionsY = 2;

      double lengthSubdivision = AtlasPhysicalProperties.foot_length / (nSubdivisionsX + 1.0);
      double widthSubdivision = AtlasPhysicalProperties.foot_width / (nSubdivisionsY + 1.0);

      double offsetX = -AtlasPhysicalProperties.foot_back;
      
      for (int i = 0; i <= nSubdivisionsX + 1; i++)
      {
         double offsetY = -AtlasPhysicalProperties.foot_width / 2.0;
         for (int j = 0; j <= nSubdivisionsY + 1; j++)
         {
            Point2d contactPointOffset = new Point2d(offsetX, offsetY);
            ground_loads_of_contact_point_offset_from_foot.add(contactPointOffset);
            offsetY += widthSubdivision;
         }
         offsetX += lengthSubdivision;
      }
   }
   
   
   
   public AtlasContactPointParamaters(AtlasRobotVersion selectedVersion, DRCRobotJointMap jointMap, boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsForFeetOnly)
   {
      
      Vector3d t0 = new Vector3d(0.0, 0.0, -pelvisBoxSizeZ / 2.0);
      t0.add(pelvisBoxOffset);
      pelvisContactPointTransform.setTranslation(t0);
      
      pelvisContacts.add(new Point2d(pelvisBoxSizeX / 2.0, pelvisBoxSizeY / 2.0));
      pelvisContacts.add(new Point2d(pelvisBoxSizeX / 2.0, -pelvisBoxSizeY / 2.0));
      pelvisContacts.add(new Point2d(-pelvisBoxSizeX / 2.0, pelvisBoxSizeY / 2.0));
      pelvisContacts.add(new Point2d(-pelvisBoxSizeX / 2.0, -pelvisBoxSizeY / 2.0));
      
      Matrix3d r0 = new Matrix3d();
      RotationFunctions.setYawPitchRoll(r0, 0.0, Math.PI / 2.0, 0.0);
      pelvisBackContactPointTransform.set(r0);
      
      Vector3d t1 = new Vector3d(-pelvisBoxSizeX / 2.0, 0.0, 0.0);
      t1.add(pelvisBoxOffset);
      pelvisBackContactPointTransform.setTranslation(t1);
      pelvisBackContacts.add(new Point2d(-pelvisBoxSizeZ / 2.0, pelvisBoxSizeY / 2.0));
      pelvisBackContacts.add(new Point2d(-pelvisBoxSizeZ / 2.0, -pelvisBoxSizeY / 2.0));
      pelvisBackContacts.add(new Point2d(pelvisBoxSizeZ / 2.0, pelvisBoxSizeY / 2.0));
      pelvisBackContacts.add(new Point2d(pelvisBoxSizeZ / 2.0, -pelvisBoxSizeY / 2.0));
      
      Matrix3d r1 = new Matrix3d();
      RotationFunctions.setYawPitchRoll(r1, 0.0, Math.PI / 2.0, 0.0);
      chestBackContactPointTransform.set(r1);
      
      Vector3d t2 = new Vector3d(-chestBoxSizeX / 2.0, 0.0, 0.0);
      t2.add(chestBoxOffset);
      chestBackContactPointTransform.setTranslation(t2);
      
      chestBackContacts.add(new Point2d(0.0, chestBoxSizeY / 2.0));
      chestBackContacts.add(new Point2d(0.0, -chestBoxSizeY / 2.0));
      chestBackContacts.add(new Point2d(chestBoxSizeZ / 2.0, chestBoxSizeY / 2.0));
      chestBackContacts.add(new Point2d(chestBoxSizeZ / 2.0, -chestBoxSizeY / 2.0));
      
      for (RobotSide robotSide : RobotSide.values)
      {
         Transform3D thighContactPointTransform = new Transform3D();
         double pitch = Math.PI / 2.0;
         thighContactPointTransform.setEuler(new Vector3d(0.0, pitch, 0.0));
         thighContactPointTransform.setTranslation(new Vector3d(-0.1179, robotSide.negateIfRightSide(0.02085), -0.08));
         thighContactPointTransforms.put(robotSide, thighContactPointTransform);
      }
      
      double[] xOffsets = new double[] {0.0, 0.1};// {0.0, 0.2};
      double[] yOffsets = new double[] {0.0, 0.0};
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2d> offsetsForSide = new ArrayList<Point2d>();
         
         for (int i = 0; i < 2; i++)
         {
            double xOffset = xOffsets[i];
            double yOffset = robotSide.negateIfRightSide(yOffsets[i]);
            
            offsetsForSide.add(new Point2d(xOffset, yOffset));
         }
         
         thighContactPoints.put(robotSide, offsetsForSide);
         
         footGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());
         handGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());
         thighGroundContactPoints.put(robotSide, new ArrayList<Pair<String, Vector3d>>());

         ArrayList<Point2d> contactPointOffsetList;
         if (addLoadsOfContactPointsForFeetOnly)
            contactPointOffsetList = ground_loads_of_contact_point_offset_from_foot;
         else
            contactPointOffsetList = ground_contact_point_offset_from_foot;
            
         for (Point2d footv3d : contactPointOffsetList)
         {
            // add ankle joint contact points on each corner of the foot
            footGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(footv3d.getX(), footv3d.getY(), -AtlasPhysicalProperties.ankleHeight)));
         }

         if (selectedVersion.getHandModel() == DRCHandType.SANDIA && addLoadsOfContactPoints)
         {
            // add finger joint contact points offset to the middle of palm-facing side of the finger segment
            String longPrefix = (robotSide == RobotSide.LEFT) ? "left_" : "right_";
            for (double[] fJointContactOffsets : HandContactParameters.sandiaFingerContactPointOffsets)
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
            for(Vector3d offset : HandContactParameters.sandiaWristContactPointOffsets.get(robotSide))
            {
               handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeHand(robotSide), offset));
            }
         }
         else if ((selectedVersion.hasIrobotHands()) && addLoadsOfContactPoints)
         {
            // add finger joint contact points offset to the middle of palm-facing side of the finger segment
            String longPrefix = (robotSide == RobotSide.LEFT) ? "left_" : "right_";
            for (double[] fJointContactOffsets : HandContactParameters.irobotFingerContactPointOffsets)
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
            for(Vector3d offset : HandContactParameters.irobotWristContactPointOffsets.get(robotSide))
            {
               handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeHand(robotSide), offset));
            }
         }
         else if (selectedVersion == AtlasRobotVersion.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS)
         {
            for (Point2d point : HandContactParameters.invisibleContactablePlaneHandContactPoints.get(robotSide))
            {
               Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
               HandContactParameters.invisibleContactablePlaneHandContactPointTransforms.get(robotSide).transform(point3d);
               handGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeHand(robotSide), new Vector3d(point3d)));
            }
         }
         
         // add butt contact points on back of thighs
         if(addLoadsOfContactPoints)
         {
            for (Point2d point : thighContactPoints.get(robotSide))
            {
               Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
               thighContactPointTransforms.get(robotSide).transform(point3d);
               thighGroundContactPoints.get(robotSide).add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeThigh(robotSide), new Vector3d(point3d)));
            }
         }
      }
      

      if (addLoadsOfContactPoints)
      {
         for (Point2d point : pelvisContacts)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            pelvisContactPointTransform.transform(point3d);
            pelvisContactPoints.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
         }

         for (Point2d point : pelvisBackContacts)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            pelvisBackContactPointTransform.transform(point3d);
            pelvisBackContactPoints.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
         }
         
         for (Point2d point : chestBackContacts)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            chestBackContactPointTransform.transform(point3d);
            chestBackContactPoints.add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeChest(), new Vector3d(point3d)));
         }
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
   public Transform3D getPelvisContactPointTransform()
   {
      return pelvisContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      return pelvisContacts;
   }

   @Override
   public Transform3D getPelvisBackContactPointTransform()
   {
      return pelvisBackContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      return pelvisBackContacts;
   }

   @Override
   public Transform3D getChestBackContactPointTransform()
   {
      return chestBackContactPointTransform;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      return chestBackContacts;
   }

   @Override
   public SideDependentList<Transform3D> getThighContactPointTransforms()
   {
      return thighContactPointTransforms;
   }

   @Override
   public SideDependentList<List<Point2d>>  getThighContactPoints()
   {
      return thighContactPoints;
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
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }
   
   @Override
   public SideDependentList<ArrayList<Point2d>> getFootGroundContactPointsInSoleFrameForController()
   {
      return new SideDependentList<>(ground_contact_point_offset_from_foot, ground_contact_point_offset_from_foot);
   }
}
