package us.ihmc.atlas.parameters;

//~--- non-JDK imports --------------------------------------------------------

import static us.ihmc.atlas.parameters.AtlasPhysicalProperties.footLengthForControl;
import static us.ihmc.atlas.parameters.AtlasPhysicalProperties.footWidthForControl;
import static us.ihmc.atlas.parameters.AtlasPhysicalProperties.toeWidthForControl;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class AtlasContactPointParameters extends DRCRobotContactPointParameters
{
   private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();
   private final Vector3d pelvisBoxOffset = new Vector3d(-0.100000, 0.000000, -0.150000);
   private final double pelvisBoxSizeX = 0.100000;
   private final double pelvisBoxSizeY = 0.150000;
   private final double pelvisBoxSizeZ = 0.200000;
   private final RigidBodyTransform pelvisContactPointTransform = new RigidBodyTransform();
   private final List<Point2d> pelvisContactPoints = new ArrayList<Point2d>();
   private final RigidBodyTransform pelvisBackContactPointTransform = new RigidBodyTransform();
   private final List<Point2d> pelvisBackContactPoints = new ArrayList<Point2d>();
   private final Vector3d chestBoxOffset = new Vector3d(0.044600, 0.000000, 0.186900);
   private final double chestBoxSizeX = 0.318800;
   private final double chestBoxSizeY = 0.240000;
   private final double chestBoxSizeZ = 0.316200;
   private final RigidBodyTransform chestBackContactPointTransform = new RigidBodyTransform();
   private final List<Point2d> chestBackContactPoints = new ArrayList<Point2d>();
   private final SideDependentList<RigidBodyTransform> thighContactPointTransforms = new SideDependentList<RigidBodyTransform>();
   private final SideDependentList<List<Point2d>> thighContactPoints = new SideDependentList<List<Point2d>>();
   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();
   private boolean handContactPointsHaveBeenCreated = false;
   private final SideDependentList<RigidBodyTransform> handContactPointTransforms = new SideDependentList<>();
   private final SideDependentList<List<Point2d>> handContactPoints = new SideDependentList<>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();
   private final DRCRobotJointMap jointMap;
   private final AtlasRobotVersion atlasVersion;

   public AtlasContactPointParameters(DRCRobotJointMap jointMap, AtlasRobotVersion atlasVersion)
   {
      this.jointMap = jointMap;
      this.atlasVersion = atlasVersion;
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
      pelvisBackContactPointTransform.setRotationAndZeroTranslation(r0);

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
      chestBackContactPointTransform.setRotationAndZeroTranslation(r1);

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
         RigidBodyTransform thighContactPointTransform = new RigidBodyTransform();
         double pitch = Math.PI / 2.0;

         thighContactPointTransform.setEuler(0.0, pitch, 0.0);
         thighContactPointTransform.setTranslation(new Vector3d(-0.1179, robotSide.negateIfRightSide(0.02085), -0.08));
         thighContactPointTransforms.put(robotSide, thighContactPointTransform);
      }

      double[] xOffsets = new double[] { 0.0, 0.1 }; // {0.0, 0.2};
      double[] yOffsets = new double[] { 0.0, 0.0 };
      SideDependentList<String> nameOfJointBeforeThighs = jointMap.getNameOfJointBeforeThighs();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2d> offsetsForSide = new ArrayList<Point2d>();

         for (int i = 0; i < 2; i++)
         {
            offsetsForSide.add(new Point2d(xOffsets[i], robotSide.negateIfRightSide(yOffsets[i])));
         }

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
         
         //MomentumBasedControll ContactPoints
         footGroundContactPoints.put(robotSide, new ArrayList<Point2d>());
         footGroundContactPoints.get(robotSide).add(new Point2d(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         footGroundContactPoints.get(robotSide).add(new Point2d(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         footGroundContactPoints.get(robotSide).add(new Point2d(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         footGroundContactPoints.get(robotSide).add(new Point2d(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactableBodiesFactory.addFootContactParameters(footGroundContactPoints); 

         
         //SCS Sim contactPoints
         int numberOfSimContactPointsY = 2, numberOfSimContactPointsX = 2;
//         int numberOfSimContactPointsY = 3, numberOfSimContactPointsX = 4;
         for (int ix = 0; ix < numberOfSimContactPointsX; ix++)
         {
            for (int iy = 0; iy < numberOfSimContactPointsY; iy++)
            {
               Point3d gcOffset = new Point3d(
                     ix * footLengthForControl / (numberOfSimContactPointsX - 1) - footLengthForControl / 2,
                     iy * footWidthForControl / (numberOfSimContactPointsY - 1) - footWidthForControl / 2, 
                     0);

               AtlasPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide).transform(gcOffset);
               jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(gcOffset))); // to SCS
            }
         }

      }
   }

   public void createInvisibleHandContactPoints()
   {
      if (handContactPointsHaveBeenCreated)
      {
         throw new RuntimeException("Contact points for the hands have already been created");
      }
      else
      {
         handContactPointsHaveBeenCreated = true;
      }

      SideDependentList<String> nameOfJointBeforeHands = jointMap.getNameOfJointBeforeHands();

      for (RobotSide robotSide : RobotSide.values)
      {
         handContactPointTransforms.put(robotSide, new RigidBodyTransform());

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
      {
         throw new RuntimeException("Contact points for the hands have already been created");
      }
      else
      {
         handContactPointsHaveBeenCreated = true;
      }

      SideDependentList<String> nameOfJointBeforeHands = jointMap.getNameOfJointBeforeHands();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform handContactPointTransform = new RigidBodyTransform();

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

   public void createHandContactPoints()
   {
      switch (atlasVersion)
      {
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
            createHandKnobContactPoints();
            break;

         case ATLAS_DUAL_ROBOTIQ:
            for (RobotSide robotSide : RobotSide.values)
               createRobotiqHandContactPoints(robotSide);
            break;
            
         case ATLAS_ROBOTIQ_HOOK:
            createRobotiqHandContactPoints(RobotSide.LEFT);
            break;
            
         default:
            break;
      }
   }

   private void createRobotiqHandContactPoints(RobotSide robotSide)
   {
      String nameOfJointBeforeHand = jointMap.getNameOfJointBeforeHands().get(robotSide);

      String finger_1_joint_1 = RobotiqHandJointNameMinimal.FINGER_1_JOINT_1.getJointName(robotSide);
      String finger_1_joint_2 = RobotiqHandJointNameMinimal.FINGER_1_JOINT_2.getJointName(robotSide);
      String finger_1_joint_3 = RobotiqHandJointNameMinimal.FINGER_1_JOINT_3.getJointName(robotSide);

      String finger_2_joint_1 = RobotiqHandJointNameMinimal.FINGER_2_JOINT_1.getJointName(robotSide);
      String finger_2_joint_2 = RobotiqHandJointNameMinimal.FINGER_2_JOINT_2.getJointName(robotSide);
      String finger_2_joint_3 = RobotiqHandJointNameMinimal.FINGER_2_JOINT_3.getJointName(robotSide);

      String thumb_joint_1 = RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_1.getJointName(robotSide);
      String thumb_joint_2 = RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_2.getJointName(robotSide);
      String thumb_joint_3 = RobotiqHandJointNameMinimal.FINGER_MIDDLE_JOINT_3.getJointName(robotSide);

      Vector3d palmContactPoint1 = new Vector3d(0.035, robotSide.negateIfRightSide(0.218), robotSide.negateIfRightSide(0.01));
      Vector3d palmContactPoint2 = new Vector3d(-palmContactPoint1.x, palmContactPoint1.y, palmContactPoint1.z);

      Vector3d fingersJoint1ContactPoint = new Vector3d(0.0, robotSide.negateIfRightSide(0.032), -0.011);
      Vector3d fingersJoint2ContactPoint = new Vector3d(0.0, robotSide.negateIfRightSide(0.021), -0.0045);
      Vector3d fingersJoint3ContactPoint = new Vector3d(0.0, robotSide.negateIfRightSide(0.03), 0.006);

      Vector3d thumbJoint1ContactPoint = new Vector3d(0.0, robotSide.negateIfRightSide(0.033), 0.0105);
      Vector3d thumbJoint2ContactPoint = new Vector3d(0.0, robotSide.negateIfRightSide(0.021), 0.004);
      Vector3d thumbJoint3ContactPoint = new Vector3d(0.0, robotSide.negateIfRightSide(0.03), -0.006);

      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint1));
      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint2));

      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(finger_1_joint_1, fingersJoint1ContactPoint));
      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(finger_1_joint_2, fingersJoint2ContactPoint));
      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(finger_1_joint_3, fingersJoint3ContactPoint));

      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(finger_2_joint_1, fingersJoint1ContactPoint));
      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(finger_2_joint_2, fingersJoint2ContactPoint));
      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(finger_2_joint_3, fingersJoint3ContactPoint));

      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(thumb_joint_1, thumbJoint1ContactPoint));
      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(thumb_joint_2, thumbJoint2ContactPoint));
      jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(thumb_joint_3, thumbJoint3ContactPoint));
   }

   @Override
   public RigidBodyTransform getPelvisContactPointTransform()
   {
      return pelvisContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      return pelvisContactPoints;
   }

   @Override
   public RigidBodyTransform getPelvisBackContactPointTransform()
   {
      return pelvisBackContactPointTransform;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      return pelvisBackContactPoints;
   }

   @Override
   public RigidBodyTransform getChestBackContactPointTransform()
   {
      return chestBackContactPointTransform;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      return chestBackContactPoints;
   }

   @Override
   public SideDependentList<RigidBodyTransform> getThighContactPointTransforms()
   {
      return thighContactPointTransforms;
   }

   @Override
   public SideDependentList<List<Point2d>> getThighContactPoints()
   {
      return thighContactPoints;
   }

   @Override
   public SideDependentList<RigidBodyTransform> getHandContactPointTransforms()
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
      return jointNameGroundContactPointMap; // SCS
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
