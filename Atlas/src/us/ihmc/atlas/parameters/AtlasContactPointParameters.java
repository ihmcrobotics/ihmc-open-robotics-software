package us.ihmc.atlas.parameters;

import static us.ihmc.atlas.parameters.AtlasPhysicalProperties.footLength;
import static us.ihmc.atlas.parameters.AtlasPhysicalProperties.footWidth;
import static us.ihmc.atlas.parameters.AtlasPhysicalProperties.toeWidth;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.utilities.math.geometry.Transform3d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class AtlasContactPointParameters extends DRCRobotContactPointParameters
{
   private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();
   private final DRCRobotJointMap jointMap;

   private final Vector3d pelvisBoxOffset = new Vector3d(-0.100000, 0.000000, -0.150000);
   private final double pelvisBoxSizeX = 0.100000;
   private final double pelvisBoxSizeY = 0.150000;
   private final double pelvisBoxSizeZ = 0.200000;
   private final Transform3d pelvisContactPointTransform = new Transform3d();
   private final List<Point2d> pelvisContactPoints = new ArrayList<Point2d>();
   private final Transform3d pelvisBackContactPointTransform = new Transform3d();
   private final List<Point2d> pelvisBackContactPoints = new ArrayList<Point2d>();

   private final Vector3d chestBoxOffset = new Vector3d(0.044600, 0.000000, 0.186900);
   private final double chestBoxSizeX = 0.318800;
   private final double chestBoxSizeY = 0.240000;
   private final double chestBoxSizeZ = 0.316200;
   private final Transform3d chestBackContactPointTransform = new Transform3d();
   private final List<Point2d> chestBackContactPoints = new ArrayList<Point2d>();
   private final SideDependentList<Transform3d> thighContactPointTransforms = new SideDependentList<Transform3d>();
   private final SideDependentList<List<Point2d>> thighContactPoints = new SideDependentList<List<Point2d>>();

   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();

   private boolean handContactPointsHaveBeenCreated = false;
   private final SideDependentList<Transform3d> handContactPointTransforms = new SideDependentList<>();
   private final SideDependentList<List<Point2d>> handContactPoints = new SideDependentList<>();

   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();

   public AtlasContactPointParameters(DRCRobotJointMap jointMap)
   {
      this.jointMap = jointMap;
      createFootContactPoints();
   }

   public void createPelvisContactPoints()
   {
      Vector3d t0 = new Vector3d(0.0, 0.0, -pelvisBoxSizeZ / 2.0);
      t0.add(pelvisBoxOffset);
      pelvisContactPointTransform.setTranslation(t0);

      pelvisContactPoints.add(new Point2d(pelvisBoxSizeX / 2.0, pelvisBoxSizeY / 2.0));
      pelvisContactPoints.add(new Point2d(pelvisBoxSizeX / 2.0, -pelvisBoxSizeY / 2.0));
      pelvisContactPoints.add(new Point2d(-pelvisBoxSizeX / 2.0, pelvisBoxSizeY / 2.0));
      pelvisContactPoints.add(new Point2d(-pelvisBoxSizeX / 2.0, -pelvisBoxSizeY / 2.0));

      for (Point2d point : pelvisContactPoints)
      {
         Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
         pelvisContactPointTransform.transform(point3d);
         jointNameGroundContactPointMap.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
      }
      contactableBodiesFactory.addPelvisContactParameters(pelvisContactPoints, pelvisContactPointTransform);
   }

   public void createPelvisBackContactPoints()
   {
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

      for (Point2d point : pelvisBackContactPoints)
      {
         Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
         pelvisBackContactPointTransform.transform(point3d);
         jointNameGroundContactPointMap.add(new Pair<String, Vector3d>("pelvis", new Vector3d(point3d)));
      }
      contactableBodiesFactory.addPelvisBackContactParameters(pelvisBackContactPoints, pelvisBackContactPointTransform);
   }

   public void createChestBackContactPoints()
   {
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

      for (Point2d point : chestBackContactPoints)
      {
         Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
         chestBackContactPointTransform.transform(point3d);
         jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(jointMap.getNameOfJointBeforeChest(), new Vector3d(point3d)));
      }
      contactableBodiesFactory.addChestBackContactParameters(chestBackContactPoints, chestBackContactPointTransform);
   }

   public void createThighContactPoints()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         Transform3d thighContactPointTransform = new Transform3d();
         double pitch = Math.PI / 2.0;
         thighContactPointTransform.setEuler(new Vector3d(0.0, pitch, 0.0));
         thighContactPointTransform.setTranslation(new Vector3d(-0.1179, robotSide.negateIfRightSide(0.02085), -0.08));
         thighContactPointTransforms.put(robotSide, thighContactPointTransform);
      }

      double[] xOffsets = new double[] { 0.0, 0.1 };// {0.0, 0.2};
      double[] yOffsets = new double[] { 0.0, 0.0 };
      SideDependentList<String> nameOfJointBeforeThighs = jointMap.getNameOfJointBeforeThighs();
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2d> offsetsForSide = new ArrayList<Point2d>();

         for (int i = 0; i < 2; i++)
            offsetsForSide.add(new Point2d(xOffsets[i], robotSide.negateIfRightSide(yOffsets[i])));

         thighContactPoints.put(robotSide, offsetsForSide);

         // add butt contact points on back of thighs
         for (Point2d point : thighContactPoints.get(robotSide))
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            thighContactPointTransforms.get(robotSide).transform(point3d);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(nameOfJointBeforeThighs.get(robotSide), new Vector3d(point3d)));
         }
      }
      contactableBodiesFactory.addThighContactParameters(nameOfJointBeforeThighs, thighContactPoints, thighContactPointTransforms);
   }

   public void createFootContactPoints()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<Point2d>());
         footGroundContactPoints.get(robotSide).add(new Point2d(-footLength / 2.0, -footWidth / 2.0));
         footGroundContactPoints.get(robotSide).add(new Point2d(-footLength / 2.0, footWidth / 2.0));
         footGroundContactPoints.get(robotSide).add(new Point2d(footLength / 2.0, -toeWidth / 2.0));
         footGroundContactPoints.get(robotSide).add(new Point2d(footLength / 2.0, toeWidth / 2.0));

         for (Point2d footGC : footGroundContactPoints.get(robotSide))
         {
            // add ankle joint contact points on each corner of the foot
            Point3d gcOffset = new Point3d(footGC.getX(), footGC.getY(), 0.0);
            AtlasPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide).transform(gcOffset);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(gcOffset)));
         }
      }
      contactableBodiesFactory.addFootContactParameters(footGroundContactPoints);
   }

   public void createInvisibleHandContactPoints()
   {
      if (handContactPointsHaveBeenCreated)
         throw new RuntimeException("Contact points for the hands have already been created");
      else
         handContactPointsHaveBeenCreated = true;

      SideDependentList<String> nameOfJointBeforeHands = jointMap.getNameOfJointBeforeHands();
      for (RobotSide robotSide : RobotSide.values)
      {
         handContactPointTransforms.put(robotSide, new Transform3d());

         double y0 = 0.0;
         handContactPoints.put(robotSide, new ArrayList<Point2d>());
         handContactPoints.get(robotSide).add(new Point2d(-0.05, -0.05 + y0));
         handContactPoints.get(robotSide).add(new Point2d(-0.05, 0.05 + y0));
         handContactPoints.get(robotSide).add(new Point2d(0.05, 0.05 + y0));
         handContactPoints.get(robotSide).add(new Point2d(0.05, -0.05 + y0));

         for (Point2d point : handContactPoints.get(robotSide))
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            handContactPointTransforms.get(robotSide).transform(point3d);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(nameOfJointBeforeHands.get(robotSide), new Vector3d(point3d)));
         }
      }
      contactableBodiesFactory.addHandContactParameters(nameOfJointBeforeHands, handContactPoints, handContactPointTransforms);
   }

   public void createHandKnobContactPoints()
   {
      if (handContactPointsHaveBeenCreated)
         throw new RuntimeException("Contact points for the hands have already been created");
      else
         handContactPointsHaveBeenCreated = true;

      SideDependentList<String> nameOfJointBeforeHands = jointMap.getNameOfJointBeforeHands();
      for (RobotSide robotSide : RobotSide.values)
      {
         Transform3d handContactPointTransform = new Transform3d();
         handContactPointTransform.rotX(robotSide.negateIfRightSide(Math.PI / 2.0));
         handContactPointTransform.setTranslation(new Vector3d(0.0, robotSide.negateIfRightSide(0.13), robotSide.negateIfRightSide(0.01)));
         handContactPointTransforms.put(robotSide, handContactPointTransform);

         handContactPoints.put(robotSide, new ArrayList<Point2d>());
         handContactPoints.get(robotSide).add(new Point2d());

         for (Point2d point : handContactPoints.get(robotSide))
         {
            Point3d point3d = new Point3d(point.getX(), point.getY(), 0.0);
            handContactPointTransforms.get(robotSide).transform(point3d);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(nameOfJointBeforeHands.get(robotSide), new Vector3d(point3d)));
         }
      }
      contactableBodiesFactory.addHandContactParameters(nameOfJointBeforeHands, handContactPoints, handContactPointTransforms);
   }

   @Override
   public Transform3d getPelvisContactPointTransform()
   {
      return pelvisContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      return pelvisContactPoints;
   }

   @Override
   public Transform3d getPelvisBackContactPointTransform()
   {
      return pelvisBackContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      return pelvisBackContactPoints;
   }

   @Override
   public Transform3d getChestBackContactPointTransform()
   {
      return chestBackContactPointTransform;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      return chestBackContactPoints;
   }

   @Override
   public SideDependentList<Transform3d> getThighContactPointTransforms()
   {
      return thighContactPointTransforms;
   }

   @Override
   public SideDependentList<List<Point2d>> getThighContactPoints()
   {
      return thighContactPoints;
   }

   @Override
   public SideDependentList<Transform3d> getHandContactPointTransforms()
   {
      return handContactPointTransforms;
   }

   @Override
   public SideDependentList<List<Point2d>> getHandContactPoints()
   {
      return handContactPoints;
   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public SideDependentList<ArrayList<Point2d>> getFootContactPoints()
   {
      return footGroundContactPoints;
   }

   @Override
   public ContactableBodiesFactory getContactableBodiesFactory()
   {
      return contactableBodiesFactory;
   }
}
