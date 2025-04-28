package us.ihmc.alexander.parameters.controller;

import us.ihmc.alexander.*;
import us.ihmc.alexander.parameters.model.AlexanderPhysicalProperties;
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

public class AlexanderContactPointParameters extends RobotContactPointParameters<RobotSide>
{
   private final boolean createHandContactPoints;

   public AlexanderContactPointParameters(HumanoidJointNameMap jointMap, AlexanderPhysicalProperties physicalProperties,
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


   public AlexanderContactPointParameters(HumanoidJointNameMap jointMap, AlexanderPhysicalProperties physicalProperties, boolean createHandContactPoints)
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

   public void createGroundContactModelParameters(double simDT)
   {
      double zStiffness;
      double zDamping;
      double xyStiffness;
      double xyDamping;

      // The gains were computed for simDT = 0.0001sec. This assumes that the gains should be inversely proportional to the simulation DT.
      double simDTRef = 0.0001;
      double modelScale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      if (useSoftGroundContactParameters)
      {
         double scale = modelScale * Math.pow(simDTRef / simDT, 0.25);
         zStiffness = (4000.0 * scale);
         zDamping = (750.0 * scale);
         xyStiffness = (50000.0 * scale);
         xyDamping = (1000.0 * scale);
      }
      else
      {
         double scale = modelScale * Math.pow(simDTRef / simDT, 0.6);

         zStiffness = (1500.0 * scale);
         zDamping = (1000.0 * scale);
         xyStiffness = (30000.0 * scale);
         xyDamping = (1500.0 * scale);
      }

      setGroundContactModelParameters(new GroundContactModelParameters(zStiffness, zDamping, xyStiffness, xyDamping));
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

         if (((AlexanderJointMap) jointMap).getArmConfiguration(robotSide) == AlexanderArmConfiguration.NUB)
         {
            String handName = ((HumanoidJointNameMap) jointMap).getHandName(robotSide);
            String elbowJointName = ((HumanoidJointNameMap) jointMap).getArmJointName(robotSide, ArmJointName.ELBOW_PITCH);
            Vector3D contactPointPositionInParentJointFrame = new Vector3D(AlexanderNubHandModel.getElbowToControlFrame());

            addSimulationContactPoint(elbowJointName, contactPointPositionInParentJointFrame);

            RigidBodyTransform transformToContactFrame = new RigidBodyTransform(new Quaternion(), contactPointPositionInParentJointFrame);
            addControllerContactPoint(handName, handName + "Contact", transformToContactFrame);
            numberOfHandContactPoints++;
         }
         else if (((AlexanderJointMap) jointMap).getArmConfiguration(robotSide) == AlexanderArmConfiguration.FOREARM)
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
      DRCRobotModel robotModel = new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      fullRobotModel.updateFrames();

      OneDoFJointBasics elbowJoint = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH);
      FramePoint3D knubPoint = new FramePoint3D(elbowJoint.getFrameAfterJoint(), AlexanderNubHandModel.getElbowToControlFrame());
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
