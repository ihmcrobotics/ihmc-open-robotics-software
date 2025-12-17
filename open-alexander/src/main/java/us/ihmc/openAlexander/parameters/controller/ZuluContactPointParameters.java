package us.ihmc.openAlexander.parameters.controller;

import us.ihmc.openAlexander.*;
import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class ZuluContactPointParameters extends RobotContactPointParameters<RobotSide>
{
   private final boolean createHandContactPoints;

   public ZuluContactPointParameters(HumanoidJointNameMap jointMap, AlexanderPhysicalProperties physicalProperties,
                                     FootContactPoints<RobotSide> footContactPoints, boolean createHandContactPoints)
   {
      super(jointMap,
            physicalProperties.getToeWidthForControl(),
            physicalProperties.getFootWidthForControl(),
            physicalProperties.getFootLengthForControl(),
            physicalProperties.getSoleToAnkleFrameTransforms());

      createFootContactPoints(footContactPoints);

      this.createHandContactPoints = createHandContactPoints;

      if (createHandContactPoints)
      {
         createHandContactPointsForNubs(physicalProperties);
      }
   }


   public ZuluContactPointParameters(HumanoidJointNameMap jointMap, AlexanderPhysicalProperties physicalProperties, boolean createHandContactPoints)
   {
      super(jointMap,
            physicalProperties.getToeWidthForControl(),
            physicalProperties.getFootWidthForControl(),
            physicalProperties.getFootLengthForControl(),
            physicalProperties.getSoleToAnkleFrameTransforms());

      createDefaultFootContactPoints();

      this.createHandContactPoints = createHandContactPoints;

      if (createHandContactPoints)
      {
         createHandContactPointsForNubs(physicalProperties);
      }
   }

   private void addControllerContactPoint(String bodyName, String contactName, RigidBodyTransform transformToContactFrame)
   {
      additionalContactRigidBodyNames.add(bodyName);
      additionalContactNames.add(contactName);
      additionalContactTransforms.add(transformToContactFrame);
   }

   private int createHandContactPointsForNubs(AlexanderPhysicalProperties physicalProperties)
   {
      int numberOfHandContactPoints = 0;

      for (RobotSide robotSide : RobotSide.values)
      {
         /////////////// NUB MODE ////////////////////////////

         if (((ZuluJointMap) jointMap).getArmConfiguration(robotSide) == ZuluArmConfiguration.NUB)
         {
            String handName = ((HumanoidJointNameMap) jointMap).getHandName(robotSide);
            String elbowJointName = ((HumanoidJointNameMap) jointMap).getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
            Vector3D contactPointPositionInParentJointFrame = new Vector3D(ZuluNubHandModel.getElbowToControlFrame());

            addSimulationContactPoint(elbowJointName, contactPointPositionInParentJointFrame);

            RigidBodyTransform transformToContactFrame = new RigidBodyTransform(new Quaternion(), contactPointPositionInParentJointFrame);
            addControllerContactPoint(handName, handName + "Contact", transformToContactFrame);
            numberOfHandContactPoints++;
         }
         else if (((ZuluJointMap) jointMap).getArmConfiguration(robotSide) == ZuluArmConfiguration.FOREARM)
         {
            String handName = ((HumanoidJointNameMap) jointMap).getHandName(robotSide);
            String wristJointName = ((HumanoidJointNameMap) jointMap).getArmJointName(robotSide, ArmJointName.WRIST_YAW);
            Vector3D contactPointPositionInParentJointFrame = new Vector3D(physicalProperties.getHandControlFrameToWristTransform(robotSide).getTranslation());

            addSimulationContactPoint(wristJointName, contactPointPositionInParentJointFrame);

            RigidBodyTransform transformToContactFrame = new RigidBodyTransform(new Quaternion(), contactPointPositionInParentJointFrame);
            addControllerContactPoint(handName, handName + "Contact", transformToContactFrame);
            numberOfHandContactPoints++;
         }
      }

      return numberOfHandContactPoints;
   }

   public static FramePoint3DReadOnly computeNubPoseInBodyFrame(RobotSide robotSide)
   {
      DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      fullRobotModel.updateFrames();

      OneDoFJointBasics elbowJoint = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH);
      FramePoint3D knubPoint = new FramePoint3D(elbowJoint.getFrameAfterJoint(), ZuluNubHandModel.getElbowToControlFrame());
      knubPoint.changeFrame(elbowJoint.getSuccessor().getBodyFixedFrame());
      return knubPoint;
   }

   public int getNumberOfContactableBodies()
   {
      if (createHandContactPoints)
      { // 2 hands, 2 feet
         return 4;
      }
      else
      { // 2 feet
         return 2;
      }
   }

   public static void main(String[] args)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         System.out.println(robotSide + " " + computeNubPoseInBodyFrame(robotSide));
      }
   }
}
