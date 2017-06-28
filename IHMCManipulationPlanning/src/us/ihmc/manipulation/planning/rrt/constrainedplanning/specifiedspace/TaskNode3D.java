package us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.manipulation.planning.trajectory.EndEffectorTrajectory;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;

public class TaskNode3D extends TaskNode
{   
   public static WheneverWholeBodyKinematicsSolver nodeTester;
   public static EndEffectorTrajectory endEffectorTrajectory;
   
   
   public TaskNode3D()
   {
      super(4);      
   }
   
   public TaskNode3D(double time, double pelvisHeight, double chestYaw, double chestPitch)
   {
      super(4);
      setNodeData(0, time);
      setNodeData(1, pelvisHeight);
      setNodeData(2, chestYaw);
      setNodeData(3, chestPitch);
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
      }

      nodeTester.initialize();
      nodeTester.holdCurrentTrajectoryMessages();
      
      /*
       * set whole body tasks.
       */            
      nodeTester.setDesiredHandPose(RobotSide.RIGHT, endEffectorTrajectory.getEndEffectorPose(getNodeData(0)));
      nodeTester.setHandSelectionMatrixFree(RobotSide.LEFT);
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendYawRotation(getNodeData(2));
      desiredChestOrientation.appendPitchRotation(getNodeData(3));
      nodeTester.setDesiredChestOrientation(desiredChestOrientation);
            
      nodeTester.setDesiredPelvisHeight(getNodeData(1));
      
      nodeTester.putTrajectoryMessages();
      
      setIsValidNode(nodeTester.isSolved());
            
      setConfigurationJoints(nodeTester.getFullRobotModelCopy().getOneDoFJoints());
      
      return isValid;
   }

   @Override
   public TaskNode createNode()
   {
      return new TaskNode3D();
   }

   

   
}
