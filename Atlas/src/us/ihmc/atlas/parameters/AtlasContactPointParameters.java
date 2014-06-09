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
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.HandContactParameters;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class AtlasContactPointParameters extends DRCRobotContactPointParameters
{
   private final Vector3d pelvisBoxOffset = new Vector3d(-0.100000, 0.000000, -0.050000);
   private final double pelvisBoxSizeX = 0.100000;
   private final double pelvisBoxSizeY = 0.150000;
   private final double pelvisBoxSizeZ = 0.200000;
   private final Transform3D pelvisContactPointTransform = new Transform3D();
   private final List<Point2d> pelvisContactPoints = new ArrayList<Point2d>();
   private final Transform3D pelvisBackContactPointTransform = new Transform3D();
   private final List<Point2d> pelvisBackContactPoints = new ArrayList<Point2d>();

   private final Vector3d chestBoxOffset = new Vector3d(0.044600, 0.000000, 0.186900);
   private final double chestBoxSizeX = 0.318800;
   private final double chestBoxSizeY = 0.240000;
   private final double chestBoxSizeZ = 0.316200;
   private final Transform3D chestBackContactPointTransform = new Transform3D();
   private final List<Point2d> chestBackContactPoints = new ArrayList<Point2d>();
   private final SideDependentList<Transform3D> thighContactPointTransforms = new SideDependentList<Transform3D>();
   private final SideDependentList<List<Point2d>> thighContactPoints = new SideDependentList<List<Point2d>>();

   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();

   private static final ArrayList<Point2d> footGroundContactPoints = new ArrayList<Point2d>();
   static
   {
      footGroundContactPoints.add(new Point2d(-AtlasPhysicalProperties.footLength / 2.0, -(AtlasPhysicalProperties.footWidth / 2.0)));
      footGroundContactPoints.add(new Point2d(-AtlasPhysicalProperties.footLength / 2.0, AtlasPhysicalProperties.footWidth / 2.0));
      footGroundContactPoints.add(new Point2d( AtlasPhysicalProperties.footLength / 2.0, -(AtlasPhysicalProperties.toeWidth / 2.0)));
      footGroundContactPoints.add(new Point2d( AtlasPhysicalProperties.footLength / 2.0, AtlasPhysicalProperties.toeWidth / 2.0));
      //Added contact points between corners
      if (DRCConfigParameters.USE_SIX_CONTACT_POINTS_PER_FOOT)
      {
         footGroundContactPoints.add(new Point2d(AtlasPhysicalProperties.footStartToetaperFromBack, -(AtlasPhysicalProperties.footWidth / 2.0)));
         footGroundContactPoints.add(new Point2d(AtlasPhysicalProperties.footStartToetaperFromBack, AtlasPhysicalProperties.footWidth / 2.0));
      }
   }

   public static final ArrayList<Point2d> footLoadsOfGroundContactPoints = new ArrayList<Point2d>();
   static
   {
      int nSubdivisionsX = 3;
      int nSubdivisionsY = 2;

      double lengthSubdivision = AtlasPhysicalProperties.footLength / (nSubdivisionsX + 1.0);
      double widthSubdivision = AtlasPhysicalProperties.footWidth / (nSubdivisionsY + 1.0);

      double offsetX = -AtlasPhysicalProperties.footBack;

      for (int i = 0; i <= nSubdivisionsX + 1; i++)
      {
         double offsetY = -AtlasPhysicalProperties.footWidth / 2.0;
         for (int j = 0; j <= nSubdivisionsY + 1; j++)
         {
            Point2d contactPointOffset = new Point2d(offsetX, offsetY);
            footLoadsOfGroundContactPoints.add(contactPointOffset);
            offsetY += widthSubdivision;
         }
         offsetX += lengthSubdivision;
      }
   }

   public AtlasContactPointParameters(AtlasRobotVersion selectedVersion, DRCRobotJointMap jointMap, boolean addLoadsOfContactPoints,
         boolean addLoadsOfContactPointsForFeetOnly)
   {

      Vector3d t0 = new Vector3d(0.0, 0.0, -pelvisBoxSizeZ / 2.0);
      t0.add(pelvisBoxOffset);
      pelvisContactPointTransform.setTranslation(t0);

      pelvisContactPoints.add(new Point2d(pelvisBoxSizeX / 2.0, pelvisBoxSizeY / 2.0));
      pelvisContactPoints.add(new Point2d(pelvisBoxSizeX / 2.0, -pelvisBoxSizeY / 2.0));
      pelvisContactPoints.add(new Point2d(-pelvisBoxSizeX / 2.0, pelvisBoxSizeY / 2.0));
      pelvisContactPoints.add(new Point2d(-pelvisBoxSizeX / 2.0, -pelvisBoxSizeY / 2.0));

      Matrix3d r0 = new Matrix3d();
      RotationFunctions.setYawPitchRoll(r0, 0.0, Math.PI / 2.0, 0.0);
      pelvisBackContactPointTransform.set(r0);

      Vector3d t1 = new Vector3d(-pelvisBoxSizeX / 2.0, 0.0, 0.0);
      t1.add(pelvisBoxOffset);
      pelvisBackContactPointTransform.setTranslation(t1);
      pelvisBackContactPoints.add(new Point2d(-pelvisBoxSizeZ / 2.0, pelvisBoxSizeY / 2.0));
      pelvisBackContactPoints.add(new Point2d(-pelvisBoxSizeZ / 2.0, -pelvisBoxSizeY / 2.0));
      pelvisBackContactPoints.add(new Point2d(pelvisBoxSizeZ / 2.0, pelvisBoxSizeY / 2.0));
      pelvisBackContactPoints.add(new Point2d(pelvisBoxSizeZ / 2.0, -pelvisBoxSizeY / 2.0));

      Matrix3d r1 = new Matrix3d();
      RotationFunctions.setYawPitchRoll(r1, 0.0, Math.PI / 2.0, 0.0);
      chestBackContactPointTransform.set(r1);

      Vector3d t2 = new Vector3d(-chestBoxSizeX / 2.0, 0.0, 0.0);
      t2.add(chestBoxOffset);
      chestBackContactPointTransform.setTranslation(t2);

      chestBackContactPoints.add(new Point2d(0.0, chestBoxSizeY / 2.0));
      chestBackContactPoints.add(new Point2d(0.0, -chestBoxSizeY / 2.0));
      chestBackContactPoints.add(new Point2d(chestBoxSizeZ / 2.0, chestBoxSizeY / 2.0));
      chestBackContactPoints.add(new Point2d(chestBoxSizeZ / 2.0, -chestBoxSizeY / 2.0));

      for (RobotSide robotSide : RobotSide.values)
      {
         Transform3D thighContactPointTransform = new Transform3D();
         double pitch = Math.PI / 2.0;
         thighContactPointTransform.setEuler(new Vector3d(0.0, pitch, 0.0));
         thighContactPointTransform.setTranslation(new Vector3d(-0.1179, robotSide.negateIfRightSide(0.02085), -0.08));
         thighContactPointTransforms.put(robotSide, thighContactPointTransform);
      }

      double[] xOffsets = new double[] { 0.0, 0.1 };// {0.0, 0.2};
      double[] yOffsets = new double[] { 0.0, 0.0 };
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

         ArrayList<Point2d> footGCs;
         if (addLoadsOfContactPointsForFeetOnly)
            footGCs = footLoadsOfGroundContactPoints;
         else
            footGCs = footGroundContactPoints;

         for (Point2d footGC : footGCs)
         {
            // add ankle joint contact points on each corner of the foot
            Point3d gcOffset = new Point3d(footGC.getX(), footGC.getY(), 0.0);
            AtlasPhysicalProperties.ankleToSoleFrameTranform.transform(gcOffset);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(gcOffset)));
         }
        
         if (selectedVersion == AtlasRobotVersion.ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS)
         {
            for (Point2d point : HandContactParameters.invisibleContactablePlaneHandContactPoints.get(robotSide))
            {
               Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
               HandContactParameters.invisibleContactablePlaneHandContactPointTransforms.get(robotSide).transform(point3d);
               jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeHand(robotSide), new Vector3d(point3d)));
            }
         }

         // add butt contact points on back of thighs
         if (addLoadsOfContactPoints)
         {
            for (Point2d point : thighContactPoints.get(robotSide))
            {
               Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
               thighContactPointTransforms.get(robotSide).transform(point3d);
               jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeThigh(robotSide), new Vector3d(point3d)));
            }
         }
      }

      if (addLoadsOfContactPoints)
      {
         for (Point2d point : pelvisContactPoints)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            pelvisContactPointTransform.transform(point3d);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
         }

         for (Point2d point : pelvisBackContactPoints)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            pelvisBackContactPointTransform.transform(point3d);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
         }

         for (Point2d point : chestBackContactPoints)
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            chestBackContactPointTransform.transform(point3d);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeChest(), new Vector3d(point3d)));
         }
      }
   }

   @Override
   public Transform3D getPelvisContactPointTransform()
   {
      return pelvisContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      return pelvisContactPoints;
   }

   @Override
   public Transform3D getPelvisBackContactPointTransform()
   {
      return pelvisBackContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      return pelvisBackContactPoints;
   }

   @Override
   public Transform3D getChestBackContactPointTransform()
   {
      return chestBackContactPointTransform;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      return chestBackContactPoints;
   }

   @Override
   public SideDependentList<Transform3D> getThighContactPointTransforms()
   {
      return thighContactPointTransforms;
   }

   @Override
   public SideDependentList<List<Point2d>> getThighContactPoints()
   {
      return thighContactPoints;
   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public SideDependentList<ArrayList<Point2d>> getFootGroundContactPointsInSoleFrameForController()
   {
      return new SideDependentList<>(footGroundContactPoints, footGroundContactPoints);
   }
}
