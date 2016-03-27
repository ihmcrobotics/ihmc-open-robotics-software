package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmDesiredAccelerationsCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage.ArmControlMode;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1DList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.io.printing.PrintTools;

public class ControllerCommandValidationTools
{
   public static boolean checkJointspaceTrajectoryPointLists(OneDoFJoint[] joints, RecyclingArrayList<SimpleTrajectoryPoint1DList> trajectoryPointLists)
   {
      if (trajectoryPointLists.size() != joints.length)
         return false;

      for (int jointIndex = 0; jointIndex < trajectoryPointLists.size(); jointIndex++)
      {
         if (!ControllerCommandValidationTools.checkJointspaceTrajectoryPointList(joints[jointIndex], trajectoryPointLists.get(jointIndex)))
            return false;
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
         if (!MathTools.isInsideBoundsInclusive(waypointPosition, jointLimitLower, jointLimitUpper))
            return false;
      }
      return true;
   }

   public static boolean checkArmDesiredAccelerationsCommand(RobotSide robotSide, OneDoFJoint[] joints, ArmDesiredAccelerationsCommand command)
   {
      if (command.getRobotSide() != robotSide)
      {
         PrintTools.warn("Received a " + command.getClass().getSimpleName() + " for the wrong side.");
         return false;
      }

      if (command.getArmControlMode() == ArmControlMode.USER_CONTROL_MODE && command.getNumberOfJoints() != joints.length)
         return false;

      return true;
   }

}
