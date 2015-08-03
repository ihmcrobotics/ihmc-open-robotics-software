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
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotContactPointParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

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
   private final List<ImmutablePair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<ImmutablePair<String, Vector3d>>();
   private boolean handContactPointsHaveBeenCreated = false;
   private final SideDependentList<RigidBodyTransform> handContactPointTransforms = new SideDependentList<>();
   private final SideDependentList<List<Point2d>> handContactPoints = new SideDependentList<>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();
   private final DRCRobotJointMap jointMap;
   private final AtlasRobotVersion atlasVersion;

   private boolean useSoftGroundContactParameters = false;

   public AtlasContactPointParameters(DRCRobotJointMap jointMap, AtlasRobotVersion atlasVersion, boolean createFootContactPoints)
   {
      this.jointMap = jointMap;
      this.atlasVersion = atlasVersion;
      if (createFootContactPoints)
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
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>("pelvis", new Vector3d(point3d)));
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
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>("pelvis", new Vector3d(point3d)));
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
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(jointMap.getNameOfJointBeforeChest(), new Vector3d(point3d)));
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
            jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeThighs.get(robotSide), new Vector3d(point3d)));
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
         int nContactPointsX = 2;
         int nContactPointsY = 2;

         double dx = 1.01 * footLengthForControl / (nContactPointsX - 1.0);
         double xOffset = 1.01 * footLengthForControl / 2.0;

         for (int ix = 1; ix <= nContactPointsX; ix++)
         {
            double alpha = (ix - 1.0) / (nContactPointsX - 1.0);
            double footWidthAtCurrentX = (1.0 - alpha) * 1.01 * footWidthForControl + alpha * 1.01 * toeWidthForControl;
            double dy = footWidthAtCurrentX / (nContactPointsY - 1.0);
            double yOffset = footWidthAtCurrentX / 2.0;

            for (int iy = 1; iy <= nContactPointsY; iy++)
            {
               double x = (ix - 1.0) * dx - xOffset;
               double y = (iy - 1.0) * dy - yOffset;
               double z = 0.0;
               
               Point3d gcOffset = new Point3d(x, y, z);

               AtlasPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide).transform(gcOffset);
               jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(gcOffset))); // to SCS
            }
         }
      }
   }

   public void createWobblyFootContactPoints(double footZWobbleForTests)
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
         int nContactPointsX = 2;
         int nContactPointsY = 2;

         double dx = 1.01 * footLengthForControl / (nContactPointsX - 1.0);
         double xOffset = 1.01 * footLengthForControl / 2.0;

         for (int ix = 1; ix <= nContactPointsX; ix++)
         {
            double alpha = (ix - 1.0) / (nContactPointsX - 1.0);
            double footWidthAtCurrentX = (1.0 - alpha) * 1.01 * footWidthForControl + alpha * 1.01 * toeWidthForControl;
            double dy = footWidthAtCurrentX / (nContactPointsY - 1.0);
            double yOffset = footWidthAtCurrentX / 2.0;

            for (int iy = 1; iy <= nContactPointsY; iy++)
            {
               double x = (ix - 1.0) * dx - xOffset;
               double y = (iy - 1.0) * dy - yOffset;
               
               double z = 0.0;
               if (((ix == 1) && (iy == 1)) || ((ix == 2) && (iy == 2)))
               {
                  z = -footZWobbleForTests;
               }
               
               Point3d gcOffset = new Point3d(x, y, z);

               AtlasPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide).transform(gcOffset);
               jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(gcOffset))); // to SCS
            }
         }
      }
   }

   public void addMoreFootContactPointsSimOnly()
   {
      int nContactPointsX = 8;
      int nContactPointsY = 3;

      double dx = 1.01 * footLengthForControl / (nContactPointsX - 1.0);
      double xOffset = 1.01 * footLengthForControl / 2.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         for (int ix = 1; ix <= nContactPointsX; ix++)
         {
            double alpha = (ix - 1.0) / (nContactPointsX - 1.0);
            double footWidthAtCurrentX = (1.0 - alpha) * 1.01 * footWidthForControl + alpha * 1.01 * toeWidthForControl;
            double dy = footWidthAtCurrentX / (nContactPointsY - 1.0);
            double yOffset = footWidthAtCurrentX / 2.0;

            for (int iy = 1; iy <= nContactPointsY; iy++)
            {
               if ((ix == 1 || ix == nContactPointsX) && (iy == 1 || iy == nContactPointsY)) // Avoid adding corners a second time
                  continue;
               double x = (ix - 1) * dx - xOffset;
               double y = (iy - 1) * dy - yOffset;
               Point3d gcOffset = new Point3d(x, y, 0);

               AtlasPhysicalProperties.soleToAnkleFrameTransforms.get(robotSide).transform(gcOffset);
               jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(jointMap.getJointBeforeFootName(robotSide), new Vector3d(gcOffset))); // to SCS
            }
         }
      }

      useSoftGroundContactParameters = true;
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
            jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHands.get(robotSide), new Vector3d(point3d)));
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
            jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHands.get(robotSide), new Vector3d(point3d)));
         }
      }

      contactableBodiesFactory.addHandContactParameters(nameOfJointBeforeHands, handContactPoints, handContactPointTransforms);
   }

   public void createHandContactPoints(boolean useHighResolutionPointGrid)
   {
      switch (atlasVersion)
      {
      case ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS:
         createHandKnobContactPoints();
         break;

      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
         if(DRCHandType.ROBOTIQ.isHandSimulated())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               createRobotiqHandContactPoints(robotSide, useHighResolutionPointGrid, false);
            }            
         }
         break;
      default:
         break;
      }
   }

   private void createRobotiqHandContactPoints(RobotSide robotSide, boolean useHighResolutionGrid, boolean areHandsFlipped)
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

      createRobotiqHandPalmContactPoints(robotSide, nameOfJointBeforeHand, useHighResolutionGrid, areHandsFlipped);

      double plusOrMinusY = robotSide.negateIfRightSide(1.0);
      double plusOrMinusZ = 1.0;
      
      if (areHandsFlipped)
         plusOrMinusZ = robotSide.negateIfRightSide(1.0); 
      
      Vector3d thumbJoint1ContactPoint = new Vector3d (0.0, plusOrMinusY * 0.033, plusOrMinusZ * 0.0105);
      Vector3d thumbJoint2ContactPoint1 = new Vector3d(0.0, plusOrMinusY * 0.005, plusOrMinusZ * -0.005);
      Vector3d thumbJoint2ContactPoint2 = new Vector3d(0.0, plusOrMinusY * 0.027, plusOrMinusZ * 0.007);
      Vector3d thumbJoint3ContactPoint = new Vector3d (0.0, plusOrMinusY * 0.025, plusOrMinusZ * -0.006);
      
      Vector3d fingersJoint1ContactPoint = new Vector3d(0.0, plusOrMinusY * 0.032, plusOrMinusZ * -0.011);
      Vector3d fingersJoint2ContactPoint1 = new Vector3d(0.0, thumbJoint2ContactPoint1.y, -thumbJoint2ContactPoint1.z);
      Vector3d fingersJoint2ContactPoint2 = new Vector3d(0.0, thumbJoint2ContactPoint2.y, -thumbJoint2ContactPoint2.z);
      Vector3d fingersJoint3ContactPoint = new Vector3d(0.0, thumbJoint3ContactPoint.y, -thumbJoint3ContactPoint.z);

      
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_1_joint_1, fingersJoint1ContactPoint));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_1_joint_2, fingersJoint2ContactPoint1));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_1_joint_2, fingersJoint2ContactPoint2));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_1_joint_3, fingersJoint3ContactPoint)); 

      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_2_joint_1, fingersJoint1ContactPoint));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_2_joint_2, fingersJoint2ContactPoint1));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_2_joint_2, fingersJoint2ContactPoint2));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(finger_2_joint_3, fingersJoint3ContactPoint));

      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(thumb_joint_1, thumbJoint1ContactPoint));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(thumb_joint_2, thumbJoint2ContactPoint1));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(thumb_joint_2, thumbJoint2ContactPoint2));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(thumb_joint_3, thumbJoint3ContactPoint));
   }
   
   private void createRobotiqHandPalmContactPoints(RobotSide robotSide, String nameOfJointBeforeHand, boolean useHighResolutionPointGrid, boolean areHandsFlipped)
   {
      double offsetFromWristToPalmPlane = 0.22; // 0.24
      Point3d palmCenter = new Point3d( -0.002, robotSide.negateIfRightSide(offsetFromWristToPalmPlane), 0.0);  // [-0.002, 0.22, 0.0] (-0.002, 0.24, 0.015)  
      double palmWidth = 0.07;
      double palmHeight = 0.075;  

      if (areHandsFlipped)
         palmHeight = robotSide.negateIfLeftSide(palmHeight);

      
      // Row of five contact points along center of palm
      Vector3d palmContactPoint1 = new Vector3d(palmCenter.x - palmWidth / 2.0, palmCenter.y, palmCenter.z);
      Vector3d palmContactPoint1b = new Vector3d(palmCenter.x - palmWidth / 4.0, palmCenter.y, palmCenter.z);
      Vector3d palmContactPointCenter = new Vector3d(palmCenter.x, palmCenter.y, palmCenter.z);
      Vector3d palmContactPoint2b = new Vector3d(palmCenter.x + palmWidth / 4.0, palmCenter.y, palmCenter.z);
      Vector3d palmContactPoint3 = new Vector3d(palmCenter.x + palmWidth / 2.0, palmCenter.y, palmCenter.z);
      
      // Pad that sits between the two outer fingers
      Vector3d palmContactPoint4 = new Vector3d(palmCenter.x, palmCenter.y, palmCenter.z - palmHeight / 2.0); 
      Vector3d palmContactPoint4b = new Vector3d(palmCenter.x, palmCenter.y, palmCenter.z - palmHeight / 4.0); 

      // Two pads that sandwich the center thumb
      Vector3d palmContactPoint5 = new Vector3d(palmContactPoint3.x, palmCenter.y, palmCenter.z + palmHeight / 2.0);
      Vector3d palmContactPoint5b = new Vector3d(palmContactPoint3.x, palmCenter.y, palmCenter.z + palmHeight / 4.0);
      Vector3d palmContactPoint6 = new Vector3d(palmContactPoint1.x, palmCenter.y, palmContactPoint5.z);
      Vector3d palmContactPoint6b = new Vector3d(palmContactPoint1.x, palmCenter.y, palmContactPoint5b.z);
 
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPointCenter));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint1));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint3));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint4));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint5));
      jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint6));
      
      if (useHighResolutionPointGrid)
      {
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint1b));
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint2b));
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint4b));
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint5b));
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3d>(nameOfJointBeforeHand, palmContactPoint6b));
      }
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
   public List<ImmutablePair<String, Vector3d>> getJointNameGroundContactPointMap()
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

   @Override
   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      if (useSoftGroundContactParameters)
      {
         linearGroundContactModel.setZStiffness(4000.0);
         linearGroundContactModel.setZDamping(750.0);
         linearGroundContactModel.setXYStiffness(50000.0);
         linearGroundContactModel.setXYDamping(1000.0);
      }
      else
      {
         linearGroundContactModel.setZStiffness(2000.0 / AtlasPhysicalProperties.scale);
         linearGroundContactModel.setZDamping(1500.0 / AtlasPhysicalProperties.scale);
         linearGroundContactModel.setXYStiffness(50000.0 / AtlasPhysicalProperties.scale);
         linearGroundContactModel.setXYDamping(2000.0 / AtlasPhysicalProperties.scale);
      }
   }
}
