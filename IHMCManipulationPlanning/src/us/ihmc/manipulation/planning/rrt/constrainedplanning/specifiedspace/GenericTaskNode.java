package us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.manipulation.planning.trajectory.ConfigurationSpace;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;

public class GenericTaskNode extends TaskNode
{
   public GenericTaskNode()
   {
      super(11);
   }
   
   public GenericTaskNode(double time, double pelvisHeight, double chestYaw, double chestPitch, double chestRoll, double eeX, double eeY, double eeZ, double eeRoll, double eePitch, double eeYaw)
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

   @Override
   public boolean isValidNode()
   {
      /*
       * using @code WheneverWholeBodyKinematicsSolver.
       * set initial configuration
       */
      
      if(getParentNode() != null)
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
      }

      nodeTester.initialize();

      nodeTester.holdCurrentTrajectoryMessages();
      /*
       * set whole body tasks.
       */            
      Pose3D desiredPose = endEffectorTrajectory.getEndEffectorPose(getNodeData(0));
      FramePoint desiredPointToWorld = new FramePoint(worldFrame, desiredPose.getPosition());
      FrameOrientation desiredOrientationToWorld = new FrameOrientation(worldFrame, desiredPose.getOrientation());
            
      FramePose desiredPoseToWorld = new FramePose(desiredPointToWorld, desiredOrientationToWorld);      
      
      desiredPoseToWorld.changeFrame(midZUpFrame);
      
      Pose3D desiredPoseToMidZUp = new Pose3D(new Point3D(desiredPoseToWorld.getPosition()), new Quaternion(desiredPoseToWorld.getOrientation()));
      nodeTester.setDesiredHandPose(endEffectorTrajectory.getRobotSide(), desiredPoseToMidZUp);
      nodeTester.setHandSelectionMatrixFree(endEffectorTrajectory.getAnotherRobotSide());
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(getNodeData(2));
      desiredChestOrientation.appendPitchRotation(getNodeData(3));
      nodeTester.setDesiredChestOrientation(desiredChestOrientation);
            
      nodeTester.setDesiredPelvisHeight(getNodeData(1));
      
      nodeTester.putTrajectoryMessages();
      
      setIsValidNode(nodeTester.isSolved());
      
      setConfigurationJoints(nodeTester.getFullRobotModelCopy());
     
      return isValid;
   }

   @Override
   public TaskNode createNode()
   {
      return new GenericTaskNode();
   }

}
