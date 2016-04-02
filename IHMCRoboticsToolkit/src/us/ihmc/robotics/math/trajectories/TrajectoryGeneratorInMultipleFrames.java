package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.robotics.math.frames.YoMultipleFramesHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class TrajectoryGeneratorInMultipleFrames
{
   private final boolean allowMultipleFrames;

   private final ReferenceFrame initialTrajectoryFrame;
   private final ArrayList<YoMultipleFramesHolder> multipleFramesHolders;
   private final ArrayList<TrajectoryGeneratorInMultipleFrames> trajectoryGeneratorsInMultipleFrames;

   public TrajectoryGeneratorInMultipleFrames(boolean allowMultipleFrames, ReferenceFrame initialTrajectoryFrame)
   {
      this.allowMultipleFrames = allowMultipleFrames;
      this.initialTrajectoryFrame = initialTrajectoryFrame;

      multipleFramesHolders = new ArrayList<>();
      trajectoryGeneratorsInMultipleFrames = new ArrayList<>();
   }

   protected void registerMultipleFramesHolders(YoMultipleFramesHolder... multipleFramesHolders)
   {
      for (YoMultipleFramesHolder multipleFramesHolder : multipleFramesHolders)
         this.multipleFramesHolders.add(multipleFramesHolder);
   }

   protected void registerTrajectoryGeneratorsInMultipleFrames(TrajectoryGeneratorInMultipleFrames... trajectoryGeneratorsInMultipleFrames)
   {
      for (TrajectoryGeneratorInMultipleFrames trajectoryGeneratorInMultipleFrames : trajectoryGeneratorsInMultipleFrames)
         this.trajectoryGeneratorsInMultipleFrames.add(trajectoryGeneratorInMultipleFrames);
   }

   public void registerAndSwitchFrame(ReferenceFrame desiredFrame)
   {
      registerNewTrajectoryFrame(desiredFrame);
      switchTrajectoryFrame(desiredFrame);
   }

   public void registerNewTrajectoryFrame(ReferenceFrame newReferenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).registerReferenceFrame(newReferenceFrame);
      for (int i = 0; i < trajectoryGeneratorsInMultipleFrames.size(); i++)
         trajectoryGeneratorsInMultipleFrames.get(i).registerNewTrajectoryFrame(newReferenceFrame);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
      {
         YoMultipleFramesHolder multipleFramesHolder = multipleFramesHolders.get(i);
         if (multipleFramesHolder.containsNaN())
            multipleFramesHolder.setToNaN(referenceFrame);
         else
            multipleFramesHolder.changeFrame(referenceFrame);
      }
      for (int i = 0; i < trajectoryGeneratorsInMultipleFrames.size(); i++)
         trajectoryGeneratorsInMultipleFrames.get(i).changeFrame(referenceFrame, false);
   }

   protected void changeFrame(ReferenceFrame referenceFrame, boolean checkIfAllowed)
   {
      if (checkIfAllowed)
         checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).changeFrame(referenceFrame);
      for (int i = 0; i < trajectoryGeneratorsInMultipleFrames.size(); i++)
         trajectoryGeneratorsInMultipleFrames.get(i).changeFrame(referenceFrame, false);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      checkIfMultipleFramesAllowed();

      for (int i = 0; i < multipleFramesHolders.size(); i++)
         multipleFramesHolders.get(i).switchCurrentReferenceFrame(referenceFrame);
      for (int i = 0; i < trajectoryGeneratorsInMultipleFrames.size(); i++)
         trajectoryGeneratorsInMultipleFrames.get(i).switchTrajectoryFrame(referenceFrame);
   }

   private void checkIfMultipleFramesAllowed()
   {
      if (!allowMultipleFrames)
         throw new RuntimeException("Must set allowMultipleFrames to true in the constructor if you ever want to register a new frame.");
   }

   public ReferenceFrame getCurrentTrajectoryFrame()
   {
      if (!allowMultipleFrames)
         return initialTrajectoryFrame;
      else
         return multipleFramesHolders.get(0).getReferenceFrame();
   }
}
