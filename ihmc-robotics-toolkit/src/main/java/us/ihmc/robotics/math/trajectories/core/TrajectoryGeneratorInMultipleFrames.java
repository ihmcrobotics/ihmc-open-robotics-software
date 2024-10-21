package us.ihmc.robotics.math.trajectories.core;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameChangeable;
import us.ihmc.euclid.transform.interfaces.Transform;

public abstract class TrajectoryGeneratorInMultipleFrames implements FrameChangeable
{
   private final List<FrameChangeable> frameChangeables = new ArrayList<>();

   protected void registerFrameChangeables(FrameChangeable... frameChangeables)
   {
      for (FrameChangeable frameChangeable : frameChangeables)
      {
         this.frameChangeables.add(frameChangeable);
      }
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return frameChangeables.get(0).getReferenceFrame();
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      for (int i = 0; i < frameChangeables.size(); i++)
      {
         frameChangeables.get(i).setReferenceFrame(referenceFrame);
      }
   }

   @Override
   public void applyTransform(Transform transform)
   {
      for (int i = 0; i < frameChangeables.size(); i++)
      {
         frameChangeables.get(i).applyTransform(transform);
      }
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      for (int i = 0; i < frameChangeables.size(); i++)
      {
         frameChangeables.get(i).applyInverseTransform(transform);
      }
   }
}
