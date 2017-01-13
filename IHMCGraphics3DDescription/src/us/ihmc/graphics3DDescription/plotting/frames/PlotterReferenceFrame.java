package us.ihmc.graphics3DDescription.plotting.frames;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@SuppressWarnings("serial")
public abstract class PlotterReferenceFrame extends ReferenceFrame
{
   private final PlotterFrameSpace frameSpace;
   protected final PlotterSpaceConverter spaceConverter;

   public PlotterReferenceFrame(String frameName, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame, PlotterFrameSpace frameSpace,
                                PlotterSpaceConverter spaceConverter)
   {
      super(frameName, isBodyCenteredFrame, isWorldFrame, isZupFrame);
      this.frameSpace = frameSpace;
      this.spaceConverter = spaceConverter;
   }

   public PlotterReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame,
                                PlotterFrameSpace frameSpace, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, isBodyCenteredFrame, isWorldFrame, isZupFrame);
      this.frameSpace = frameSpace;
      this.spaceConverter = spaceConverter;
   }

   public PlotterReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isBodyCenteredFrame,
                                boolean isWorldFrame, boolean isZupFrame, PlotterFrameSpace frameSpace, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, transformToParent, isBodyCenteredFrame, isWorldFrame, isZupFrame);
      this.frameSpace = frameSpace;
      this.spaceConverter = spaceConverter;
   }

   public PlotterReferenceFrame(String frameName, ReferenceFrame parentFrame, PlotterFrameSpace frameSpace, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame);
      this.frameSpace = frameSpace;
      this.spaceConverter = spaceConverter;
   }

   protected PlotterFrameSpace getFrameSpace()
   {
      return frameSpace;
   }

   protected PlotterSpaceConverter getSpaceConverter()
   {
      return spaceConverter;
   }
}
