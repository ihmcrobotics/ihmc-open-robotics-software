package us.ihmc.darpaRoboticsChallenge.testTools;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.HandstepHelper;
import us.ihmc.robotics.robotSide.RobotSide;

public class ScriptedHandstepGenerator
{
   private final HandstepHelper handstepHelper;

   public ScriptedHandstepGenerator(FullHumanoidRobotModel fullRobotModel)
   {
      handstepHelper = new HandstepHelper(fullRobotModel);
   }

   public Handstep createHandstep(RobotSide robotSide, Tuple3d position, Vector3d surfaceNormal, double rotationAngleAboutNormal, double swingTrajectoryTime)
   {
      Handstep desiredHandstep = handstepHelper.getDesiredHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);

      return desiredHandstep;
   }
}
