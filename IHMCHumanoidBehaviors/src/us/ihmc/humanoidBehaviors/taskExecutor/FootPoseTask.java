package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.FootPosePacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootPoseBehavior;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class FootPoseTask implements Task
{
   private static final boolean DEBUG = false;
   private final FootPosePacket footPosePacket;
   private final FootPoseBehavior footPoseBehavior;

   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final double sleepTime;

   public FootPoseTask(RobotSide robotSide, Point3d position, Quat4d orientation, DoubleYoVariable yoTime, FootPoseBehavior footPoseBehavior,
         double trajectoryTime)
   {
      this(robotSide, position, orientation, yoTime, footPoseBehavior, trajectoryTime, 0.0);
   }

   public FootPoseTask(RobotSide robotSide, Point3d position, Quat4d orientation, DoubleYoVariable yoTime, FootPoseBehavior footPoseBehavior,
         double trajectoryTime, double sleepTime)
   {
      this.footPoseBehavior = footPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;

      footPosePacket = new FootPosePacket(robotSide, position, orientation, trajectoryTime);
   }

   public FootPoseTask(RobotSide robotSide, FramePose pose, DoubleYoVariable yoTime, FootPoseBehavior footPoseBehavior, double trajectoryTime, double sleepTime)
   {
      this.footPoseBehavior = footPoseBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;

      pose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      Point3d position = new Point3d();
      Quat4d orientation = new Quat4d();
      pose.getPose(position, orientation);
      footPosePacket = new FootPosePacket(robotSide, position, orientation, trajectoryTime);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("Started arm task");
      footPoseBehavior.initialize();
      footPoseBehavior.setInput(footPosePacket);
   }

   @Override
   public void doAction()
   {
      footPoseBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && footPoseBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished " + footPosePacket.robotSide.getCamelCaseNameForMiddleOfExpression() + " foot pose");
      footPoseBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return footPoseBehavior.isDone() && sleepTimeAchieved;
   }
}
