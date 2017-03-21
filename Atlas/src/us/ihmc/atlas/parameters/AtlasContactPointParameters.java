package us.ihmc.atlas.parameters;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotiq.model.RobotiqHandModel.RobotiqHandJointNameMinimal;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class AtlasContactPointParameters extends RobotContactPointParameters
{
   private final int numberOfContactableBodies;

   private boolean handContactPointsHaveBeenCreated = false;
   private final AtlasJointMap jointMap;
   private final AtlasRobotVersion atlasVersion;

   private boolean useSoftGroundContactParameters = false;

   public AtlasContactPointParameters(AtlasJointMap jointMap, AtlasRobotVersion atlasVersion, boolean createFootContactPoints,
         FootContactPoints footContactPoints, boolean createAdditionalContactPoints)
   {
      super(jointMap, jointMap.getPhysicalProperties().getToeWidthForControl(), jointMap.getPhysicalProperties().getFootWidthForControl(), jointMap.getPhysicalProperties().getFootLengthForControl(), jointMap.getPhysicalProperties().getSoleToAnkleFrameTransforms());

      this.jointMap = jointMap;
      this.atlasVersion = atlasVersion;
      if (createFootContactPoints)
      {
         if (footContactPoints == null)
            createDefaultFootContactPoints();
         else
            createFootContactPoints(footContactPoints);
      }

      int totalContacts = 2;
      if (createAdditionalContactPoints)
         totalContacts += createAdditionalHandContactPoints();

      numberOfContactableBodies = totalContacts;
   }

   public int createAdditionalHandContactPoints()
   {
      switch (atlasVersion)
      {
      case ATLAS_UNPLUGGED_V5_NO_HANDS:
         createHandKnobContactPoints();
         return 2;

      case ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ:
         if (DRCHandType.ROBOTIQ.isHandSimulated())
         {
            for (RobotSide robotSide : RobotSide.values)
               createRobotiqHandContactPoints(robotSide, false);
         }
         return 2;

      default:
         return 0;
      }
   }

   private void createHandKnobContactPoints()
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
         String bodyName = jointMap.getHandName(robotSide);
         Point3D pointLocationInParentJoint = new Point3D(0.0, robotSide.negateIfRightSide(0.13), 0.0);

         RigidBodyTransform transformToContactFrame = new RigidBodyTransform(new Quaternion(), pointLocationInParentJoint);
         if (robotSide == RobotSide.LEFT)
            transformToContactFrame.appendRollRotation(Math.PI);

         addSimulationContactPoint(nameOfJointBeforeHands.get(robotSide), pointLocationInParentJoint);
         contactableBodiesFactory.addAdditionalContactPoint(bodyName, bodyName + "Contact", transformToContactFrame);
      }
   }

   private void createRobotiqHandContactPoints(RobotSide robotSide, boolean areHandsFlipped)
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

      createRobotiqHandPalmContactPoints(robotSide, nameOfJointBeforeHand, areHandsFlipped);

      double plusOrMinusY = robotSide.negateIfRightSide(1.0);
      double plusOrMinusZ = 1.0;

      if (areHandsFlipped)
         plusOrMinusZ = robotSide.negateIfRightSide(1.0);

      Vector3D thumbJoint1ContactPoint = new Vector3D(0.0, plusOrMinusY * 0.033, plusOrMinusZ * 0.0105);
      Vector3D thumbJoint2ContactPoint1 = new Vector3D(0.0, plusOrMinusY * 0.005, plusOrMinusZ * -0.005);
      Vector3D thumbJoint2ContactPoint2 = new Vector3D(0.0, plusOrMinusY * 0.027, plusOrMinusZ * 0.007);
      Vector3D thumbJoint3ContactPoint = new Vector3D(0.0, plusOrMinusY * 0.025, plusOrMinusZ * -0.006);

      Vector3D fingersJoint1ContactPoint = new Vector3D(0.0, plusOrMinusY * 0.032, plusOrMinusZ * -0.011);
      Vector3D fingersJoint2ContactPoint1 = new Vector3D(0.0, thumbJoint2ContactPoint1.getY(), -thumbJoint2ContactPoint1.getZ());
      Vector3D fingersJoint2ContactPoint2 = new Vector3D(0.0, thumbJoint2ContactPoint2.getY(), -thumbJoint2ContactPoint2.getZ());
      Vector3D fingersJoint3ContactPoint = new Vector3D(0.0, thumbJoint3ContactPoint.getY(), -thumbJoint3ContactPoint.getZ());

      addSimulationContactPoint(finger_1_joint_1, fingersJoint1ContactPoint);
      addSimulationContactPoint(finger_1_joint_2, fingersJoint2ContactPoint1);
      addSimulationContactPoint(finger_1_joint_2, fingersJoint2ContactPoint2);
      addSimulationContactPoint(finger_1_joint_3, fingersJoint3ContactPoint);
      addSimulationContactPoint(finger_2_joint_1, fingersJoint1ContactPoint);
      addSimulationContactPoint(finger_2_joint_2, fingersJoint2ContactPoint1);
      addSimulationContactPoint(finger_2_joint_2, fingersJoint2ContactPoint2);
      addSimulationContactPoint(finger_2_joint_3, fingersJoint3ContactPoint);
      addSimulationContactPoint(thumb_joint_1, thumbJoint1ContactPoint);
      addSimulationContactPoint(thumb_joint_2, thumbJoint2ContactPoint1);
      addSimulationContactPoint(thumb_joint_2, thumbJoint2ContactPoint2);
      addSimulationContactPoint(thumb_joint_3, thumbJoint3ContactPoint);
   }

   private void createRobotiqHandPalmContactPoints(RobotSide robotSide, String nameOfJointBeforeHand, boolean areHandsFlipped)
   {
      double offsetFromWristToPalmPlane = 0.22; // 0.24
      Point3D palmCenter = new Point3D(-0.002, robotSide.negateIfRightSide(offsetFromWristToPalmPlane), 0.0); // [-0.002, 0.22, 0.0] (-0.002, 0.24, 0.015)
      double palmWidth = 0.07;
      double palmHeight = 0.075;

      if (areHandsFlipped)
         palmHeight = robotSide.negateIfLeftSide(palmHeight);

      // Row of five contact points along center of palm
      Vector3D palmContactPoint1 = new Vector3D(palmCenter.getX() - palmWidth / 2.0, palmCenter.getY(), palmCenter.getZ());
      Vector3D palmContactPoint1b = new Vector3D(palmCenter.getX() - palmWidth / 4.0, palmCenter.getY(), palmCenter.getZ());
      Vector3D palmContactPointCenter = new Vector3D(palmCenter.getX(), palmCenter.getY(), palmCenter.getZ());
      Vector3D palmContactPoint2b = new Vector3D(palmCenter.getX() + palmWidth / 4.0, palmCenter.getY(), palmCenter.getZ());
      Vector3D palmContactPoint3 = new Vector3D(palmCenter.getX() + palmWidth / 2.0, palmCenter.getY(), palmCenter.getZ());

      // Pad that sits between the two outer fingers
      Vector3D palmContactPoint4 = new Vector3D(palmCenter.getX(), palmCenter.getY(), palmCenter.getZ() - palmHeight / 2.0);
      Vector3D palmContactPoint4b = new Vector3D(palmCenter.getX(), palmCenter.getY(), palmCenter.getZ() - palmHeight / 4.0);

      // Two pads that sandwich the center thumb
      Vector3D palmContactPoint5 = new Vector3D(palmContactPoint3.getX(), palmCenter.getY(), palmCenter.getZ() + palmHeight / 2.0);
      Vector3D palmContactPoint5b = new Vector3D(palmContactPoint3.getX(), palmCenter.getY(), palmCenter.getZ() + palmHeight / 4.0);
      Vector3D palmContactPoint6 = new Vector3D(palmContactPoint1.getX(), palmCenter.getY(), palmContactPoint5.getZ());
      Vector3D palmContactPoint6b = new Vector3D(palmContactPoint1.getX(), palmCenter.getY(), palmContactPoint5b.getZ());

      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPointCenter);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint1);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint3);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint4);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint5);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint6);

      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint1b);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint2b);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint4b);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint5b);
      addSimulationContactPoint(nameOfJointBeforeHand, palmContactPoint6b);
   }

   @Override
   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      double scale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      if (useSoftGroundContactParameters)
      {
         linearGroundContactModel.setZStiffness(scale * 4000.0);
         linearGroundContactModel.setZDamping(scale * 750.0);
         linearGroundContactModel.setXYStiffness(scale * 50000.0);
         linearGroundContactModel.setXYDamping(scale * 1000.0);
      }
      else
      {
         linearGroundContactModel.setZStiffness(scale * 2000.0);
         linearGroundContactModel.setZDamping(scale * 1500.0);
         linearGroundContactModel.setXYStiffness(scale * 50000.0);
         linearGroundContactModel.setXYDamping(scale * 2000.0);
      }
   }

   public int getNumberOfContactableBodies()
   {
      return numberOfContactableBodies;
   }
}
