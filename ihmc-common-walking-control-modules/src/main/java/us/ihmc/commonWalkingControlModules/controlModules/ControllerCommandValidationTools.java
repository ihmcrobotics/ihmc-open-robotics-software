package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.NeckTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.OneDoFJointTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;

public class ControllerCommandValidationTools
{
   public static boolean checkArmTrajectoryCommand(OneDoFJointBasics[] joints, ArmTrajectoryCommand command)
   {
      return checkOneDoFJointTrajectoryCommandList(joints, command.getJointspaceTrajectory().getTrajectoryPointLists());
   }

   public static boolean checkNeckTrajectoryCommand(OneDoFJointBasics[] joints, NeckTrajectoryCommand command)
   {
      return checkOneDoFJointTrajectoryCommandList(joints, command.getJointspaceTrajectory().getTrajectoryPointLists());
   }

   public static boolean checkSpineTrajectoryCommand(OneDoFJointBasics[] joints, SpineTrajectoryCommand command)
   {
      return checkOneDoFJointTrajectoryCommandList(joints, command.getJointspaceTrajectory().getTrajectoryPointLists());
   }

   public static boolean checkArmDesiredAccelerationsCommand(OneDoFJointBasics[] joints, ArmDesiredAccelerationsCommand command)
   {
      return command.getDesiredAccelerations().getNumberOfJoints() == joints.length;
   }

   public static boolean checkOneDoFJointTrajectoryCommandList(OneDoFJointBasics[] joints, RecyclingArrayList<OneDoFJointTrajectoryCommand> trajectoryPointLists)
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

   public static boolean checkJointspaceTrajectoryPointList(OneDoFJointBasics joint, SimpleTrajectoryPoint1DList trajectoryPointList)
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
