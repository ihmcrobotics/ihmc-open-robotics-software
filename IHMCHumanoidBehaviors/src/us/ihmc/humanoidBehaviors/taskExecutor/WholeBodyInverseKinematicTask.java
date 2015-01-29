package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class WholeBodyInverseKinematicTask implements Task
{
   private static final boolean DEBUG = false;

   private static final double trajectoryDuration = 2.0;
   
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final WholeBodyInverseKinematicBehavior wholeBodyIKBehavior;

   private final RobotSide robotSide;
   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final FramePose desiredHandPose;
   private final double trajectoryTime;

   public WholeBodyInverseKinematicTask(RobotSide robotSide, DoubleYoVariable yoTime, WholeBodyInverseKinematicBehavior wholeBodyIKBehavior,
         FramePose desiredHandPose, double trajectoryTime)
   {
      this.yoTime = yoTime;
      this.wholeBodyIKBehavior = wholeBodyIKBehavior;
      this.robotSide = robotSide;
      this.trajectoryTime = trajectoryTime;
      desiredHandPose.checkReferenceFrameMatch(worldFrame);
      this.desiredHandPose = desiredHandPose;

   }

   @Override
   public void doTransitionIntoAction()
   {
      wholeBodyIKBehavior.initialize();
      wholeBodyIKBehavior.setInputs(robotSide, desiredHandPose, trajectoryDuration);
   }

   @Override
   public void doAction()
   {
      wholeBodyIKBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && wholeBodyIKBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      wholeBodyIKBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime;
      return wholeBodyIKBehavior.isDone() && sleepTimeAchieved;
   }

}
