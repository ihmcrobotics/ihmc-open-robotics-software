package us.ihmc.graphicsDescription.plotting.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

public abstract class PlotterReferenceFrame extends ReferenceFrame
{
   private final PlotterFrameSpace frameSpace;
   protected final PlotterSpaceConverter spaceConverter;

   public PlotterReferenceFrame(String frameName, PlotterFrameSpace frameSpace, PlotterSpaceConverter spaceConverter)
   {
      super(frameName);
      this.frameSpace = frameSpace;
      this.spaceConverter = spaceConverter;
   }

   public PlotterReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isWorldFrame, boolean isZupFrame, PlotterFrameSpace frameSpace,
                                PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, isWorldFrame, isZupFrame);
      this.frameSpace = frameSpace;
      this.spaceConverter = spaceConverter;
   }

   public PlotterReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isWorldFrame,
                                boolean isZupFrame, PlotterFrameSpace frameSpace, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, transformToParent, isWorldFrame, isZupFrame);
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
