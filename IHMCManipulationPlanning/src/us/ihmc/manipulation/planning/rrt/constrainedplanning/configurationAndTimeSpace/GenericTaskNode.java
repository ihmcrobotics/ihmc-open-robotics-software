package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint3D;
import us.ihmc.robotics.geometry.FramePose;

public class GenericTaskNode extends CTTaskNode
{
   public static double handCoordinateOffsetX = -0.2;
   public static int nodeDimension = 11;

   public GenericTaskNode()
   {
      super(11);
   }
   
   public GenericTaskNode(CTTaskNode node)
   {
      super(node);
   }

   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll, double eeX, double eeY, double eeZ,
                          double eeRoll, double eePitch, double eeYaw)
   {
      super(11);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
      setNodeData(4, chestRoll);
      setNodeData(5, eeX);
      setNodeData(6, eeY);
      setNodeData(7, eeZ);
      setNodeData(8, eeRoll);
      setNodeData(9, eePitch);
      setNodeData(10, eeYaw);
   }

   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll, ConfigurationSpace eeConfigurationSpace)
   {
      super(11);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
      setNodeData(4, chestRoll);
      setNodeData(5, eeConfigurationSpace.getTranslationX());
      setNodeData(6, eeConfigurationSpace.getTranslationY());
      setNodeData(7, eeConfigurationSpace.getTranslationZ());
      setNodeData(8, eeConfigurationSpace.getRotationRoll());
      setNodeData(9, eeConfigurationSpace.getRotationPitch());
      setNodeData(10, eeConfigurationSpace.getRotationYaw());
   }

   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll)
   {
      super(11);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
      setNodeData(4, chestRoll);
      ConfigurationSpace eeConfigurationSpace = new ConfigurationSpace();
      setNodeData(5, eeConfigurationSpace.getTranslationX());
      setNodeData(6, eeConfigurationSpace.getTranslationY());
      setNodeData(7, eeConfigurationSpace.getTranslationZ());
      setNodeData(8, eeConfigurationSpace.getRotationRoll());
      setNodeData(9, eeConfigurationSpace.getRotationPitch());
      setNodeData(10, eeConfigurationSpace.getRotationYaw());
   }

   private ConfigurationSpace getEndEffectorConfigurationSpace()
   {
      ConfigurationSpace configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(getNodeData(5), getNodeData(6), getNodeData(7));
      configurationSpace.setRotation(getNodeData(8), getNodeData(9), getNodeData(10));
      return configurationSpace;
   }

   public Pose3D getEndEffectorPose()
   {
      /*
       * to world frame.
       */
      return constrainedEndEffectorTrajectory.getEndEffectorPose(getNodeData(0), getEndEffectorConfigurationSpace());
   }

   @Override
   public boolean isValidNode()
   {
      /*
       * using @code WheneverWholeBodyKinematicsSolver. set initial
       * configuration
       */

      if (getParentNode() != null)
      {
         nodeTester.updateRobotConfigurationDataJointsOnly(getParentNode().getOneDoFJoints());
         for (int i = 0; i < getParentNode().getOneDoFJoints().length; i++)
         {
            double jointPosition = getParentNode().getOneDoFJoints()[i].getQ();
         }
      }
      else
      {
         PrintTools.warn("parentNode is required.");
         nodeTester.updateRobotConfigurationDataJointsOnly(FullRobotModelUtils.getAllJointsExcludingHands(initialRobotModel));
      }

      nodeTester.initialize();

      nodeTester.holdCurrentTrajectoryMessages();
      /*
       * set whole body tasks.
       */
      Pose3D desiredPose = getEndEffectorPose();
      FramePoint3D desiredPointToWorld = new FramePoint3D(worldFrame, desiredPose.getPosition());
      FrameOrientation desiredOrientationToWorld = new FrameOrientation(worldFrame, desiredPose.getOrientation());

      FramePose desiredPoseToWorld = new FramePose(desiredPointToWorld, desiredOrientationToWorld);

      desiredPoseToWorld.changeFrame(midZUpFrame);

      Pose3D desiredPoseToMidZUp = new Pose3D(new Point3D(desiredPoseToWorld.getPosition()), new Quaternion(desiredPoseToWorld.getOrientation()));
      desiredPoseToMidZUp.appendTranslation(handCoordinateOffsetX, 0.0, 0.0);

      nodeTester.setDesiredHandPose(constrainedEndEffectorTrajectory.getRobotSide(), desiredPoseToMidZUp);
      nodeTester.setHandSelectionMatrixFree(constrainedEndEffectorTrajectory.getAnotherRobotSide());

      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(getNodeData(2));

      PrintTools.info("c yaw " + getNodeData(2) * 180 / Math.PI);
      desiredChestOrientation.appendPitchRotation(getNodeData(3));
      desiredChestOrientation.appendRollRotation(getNodeData(4));
      nodeTester.setDesiredChestOrientation(desiredChestOrientation);

      nodeTester.setDesiredPelvisHeight(getNodeData(1));

      nodeTester.putTrajectoryMessages();

      setIsValidNode(nodeTester.isSolved());

      setConfigurationJoints(nodeTester.getFullRobotModelCopy());

      return isValid;
   }

   @Override
   public CTTaskNode createNode()
   {
      return new GenericTaskNode();
   }

}
