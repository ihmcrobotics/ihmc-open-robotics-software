package us.ihmc.humanoidBehaviors.taskExecutor;

import java.util.ArrayList;

import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LimbName;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;

public class FootstepTask implements Task
{
   private final FootstepListBehavior footstepListBehavior;
   private ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

   public FootstepTask(FullRobotModel fullRobotModel, RobotSide robotSide, FootstepListBehavior footstepListBehavior, FramePose footPose)
   {
      ReferenceFrame soleFrame;
      RigidBody endEffector;
      
      soleFrame = fullRobotModel.getSoleFrame(robotSide);
      endEffector = fullRobotModel.getEndEffector(robotSide, LimbName.LEG);
      
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", ReferenceFrame.getWorldFrame());
      poseReferenceFrame.setPoseAndUpdate(footPose);
      footsteps.add(new Footstep(endEffector, robotSide, soleFrame, poseReferenceFrame));
      this.footstepListBehavior = footstepListBehavior;
   }

   @Override
   public void doTransitionIntoAction()
   {
      footstepListBehavior.initialize();
      footstepListBehavior.set(footsteps);
   }

   @Override
   public void doAction()
   {
      footstepListBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      footstepListBehavior.finalize();
      footsteps.clear();
   }

   @Override
   public boolean isDone()
   {
      return footstepListBehavior.isDone();
   }

}
