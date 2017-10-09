package us.ihmc.robotics.math.frames;

import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public interface YoMultipleFramesHolder extends ReferenceFrameHolder
{
   public void registerReferenceFrame(ReferenceFrame newReferenceFrame);

   public void changeFrame(ReferenceFrame desiredReferenceFrame);

   public ReferenceFrame switchCurrentReferenceFrame(ReferenceFrame newCurrentReferenceFrame);

   public boolean isReferenceFrameRegistered(ReferenceFrame referenceFrame);

   public int getNumberOfReferenceFramesRegistered();

   public void getRegisteredReferenceFrames(List<ReferenceFrame> referenceFramesToPack);

   public boolean containsNaN();

   public void setToNaN(ReferenceFrame desiredReferenceFrame);

   @Override
   public ReferenceFrame getReferenceFrame();
}
