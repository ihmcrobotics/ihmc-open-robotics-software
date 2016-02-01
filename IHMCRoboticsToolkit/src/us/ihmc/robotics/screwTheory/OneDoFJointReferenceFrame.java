package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class OneDoFJointReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 1L;

   public OneDoFJointReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);
   }

   public abstract void setAndUpdate(double jointPosition);
}