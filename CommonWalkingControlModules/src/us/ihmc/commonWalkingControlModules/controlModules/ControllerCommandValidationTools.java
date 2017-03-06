package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage.ArmControlMode;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.io.printing.PrintTools;

public class ControllerCommandValidationTools
{
   public static boolean checkArmTrajectoryCommand(OneDoFJoint[] joints, ArmTrajectoryCommand command)
   {
      return checkOneDoFJointTrajectoryCommandList(joints, command.getTrajectoryPointLists());
   }

   public static boolean checkNeckTrajectoryCommand(OneDoFJoint[] joints, NeckTrajectoryCommand command)
   {
      return checkOneDoFJointTrajectoryCommandList(joints, command.getTrajectoryPointLists());
   }

   public static boolean checkSpineTrajectoryCommand(OneDoFJoint[] joints, SpineTrajectoryCommand command)
   {
      return checkOneDoFJointTrajectoryCommandList(joints, command.getTrajectoryPointLists());
   }

   public static boolean checkArmDesiredAccelerationsCommand(OneDoFJoint[] joints, ArmDesiredAccelerationsCommand command)
   {
      return command.getArmControlMode() != ArmControlMode.USER_CONTROL_MODE || command.getNumberOfJoints() == joints.length;
   }

   public static boolean checkOneDoFJointTrajectoryCommandList(OneDoFJoint[] joints, RecyclingArrayList<OneDoFJointTrajectoryCommand> trajectoryPointLists)
   {
      if (trajectoryPointLists.size() != joints.length)
      {
         PrintTools.warn("Incorrect joint length. Expected "+joints.length+" got "+trajectoryPointLists.size());
         return false;
      }

      for (int jointIndex = 0; jointIndex < trajectoryPointLists.size(); jointIndex++)
      {
         if (!ControllerCommandValidationTools.checkJointspaceTrajectoryPointList(joints[jointIndex], trajectoryPointLists.get(jointIndex)))
         {
            
            PrintTools.warn("Invalid joint trajectory ( "+jointIndex+" - "+joints[jointIndex].getName()+")");
            return false;
         }
      }

      return true;
   }

   public static boolean checkJointspaceTrajectoryPointList(OneDoFJoint joint, SimpleTrajectoryPoint1DList trajectoryPointList)
   {
      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
      {
         double waypointPosition = trajectoryPointList.getTrajectoryPoint(i).getPosition();
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();
         if (!MathTools.intervalContains(waypointPosition, jointLimitLower, jointLimitUpper))
         {
            PrintTools.warn("Joint out of bounds: "+joint.getName()+" (" +jointLimitLower+", "+jointLimitUpper+ ") = "+waypointPosition+" (t="+i+")");
            return false;
         }
      }
      return true;
   }
}
