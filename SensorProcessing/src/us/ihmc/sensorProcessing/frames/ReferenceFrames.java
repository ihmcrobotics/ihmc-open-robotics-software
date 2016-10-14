package us.ihmc.sensorProcessing.frames;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface ReferenceFrames
{
   public void updateFrames();
   
   public ReferenceFrame getCenterOfMassFrame();
}