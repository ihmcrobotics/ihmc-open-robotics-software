package us.ihmc.darpaRoboticsChallenge.posePlayback;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.YoPolynomial;

public class PosePlaybackSmoothPoseInterpolator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable poseStartTime = new DoubleYoVariable("poseStartTime", registry);
   private final DoubleYoVariable poseMorphPercentage = new DoubleYoVariable("poseMorphPercentage", registry);
   private final DoubleYoVariable poseMorphDuration = new DoubleYoVariable("poseMorphDuration", registry);
   private final IntegerYoVariable poseSequenceIndex = new IntegerYoVariable("poseSequenceIndex", registry);
   
   private boolean lastPoseIncrementedSequence = false;

   private final int numberOfCoefficients = 4;    // Cubic

   private final YoPolynomial yoPolynomial = new YoPolynomial("posePolynomial", numberOfCoefficients, registry);

   private PosePlaybackRobotPoseSequence sequence;


   public PosePlaybackSmoothPoseInterpolator(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);

      poseMorphDuration.set(1.0);
   }

   public void startSequencePlayback(PosePlaybackRobotPoseSequence sequence, double startTime)
   {
      this.sequence = sequence;

      computeMorphDuration();
      setupPolynomialSpline(startTime, poseMorphDuration.getDoubleValue());

      poseStartTime.set(startTime);
      poseSequenceIndex.set(0);
   }

   private void computeMorphDuration()
   {
      poseMorphDuration.set(1.0);
   }

   private void setupPolynomialSpline(double time, double duration)
   {
      yoPolynomial.setCubic(time, time + duration, 0.0, 0.0, 1.0, 0.0);
   }

   public PosePlaybackRobotPose getPose(double time)
   {
      double timeIntoPose = time - poseStartTime.getDoubleValue();

      int index = poseSequenceIndex.getIntegerValue();
      if (isDone())
      {
         return sequence.getFinalPose();
      }

      PosePlaybackRobotPose poseOne = sequence.getPose(index);
      PosePlaybackRobotPose poseTwo = sequence.getPose(poseSequenceIndex.getIntegerValue() + 1);

      poseMorphPercentage.set(timeIntoPose / poseMorphDuration.getDoubleValue());
      PosePlaybackRobotPose morphedPose = PosePlaybackRobotPose.morph(poseOne, poseTwo, poseMorphPercentage.getDoubleValue());

      if (poseMorphPercentage.getDoubleValue() >= 1.0)
      {
         poseSequenceIndex.increment();
         poseStartTime.set(time);
         lastPoseIncrementedSequence = true;
      }
      else
      {
         lastPoseIncrementedSequence = false;
      }
      return morphedPose;
   }
   
   public double getTransitionTimeDelay()
   {
      return sequence.getPose(poseSequenceIndex.getIntegerValue()).getPlayBackDelayBeforePose();
   }
   
   public boolean didLastPoseIncrementSequence()
   {
      return lastPoseIncrementedSequence;
   }

   public boolean isDone()
   {
      return (poseSequenceIndex.getIntegerValue() >= sequence.getNumberOfPoses() - 1);
   }

}
