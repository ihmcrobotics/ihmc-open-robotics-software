package us.ihmc.avatar.testTools;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.HandstepHelper;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class ScriptedHandstepGenerator
{
   private final HandstepHelper handstepHelper;

   public ScriptedHandstepGenerator(FullHumanoidRobotModel fullRobotModel)
   {
      handstepHelper = new HandstepHelper(fullRobotModel);
   }

   public Handstep createHandstep(RobotSide robotSide, Tuple3DBasics position, Vector3D surfaceNormal, double rotationAngleAboutNormal, double swingTrajectoryTime)
   {
      Handstep desiredHandstep = handstepHelper.getDesiredHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);

      return desiredHandstep;
   }
}
