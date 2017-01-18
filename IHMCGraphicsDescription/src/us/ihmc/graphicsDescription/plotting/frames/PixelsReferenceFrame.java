package us.ihmc.graphicsDescription.plotting.frames;

import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@SuppressWarnings("serial")
public abstract class PixelsReferenceFrame extends PlotterReferenceFrame
{
   public PixelsReferenceFrame(String frameName, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, isBodyCenteredFrame, isWorldFrame, isZupFrame, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public PixelsReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame,
                               PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, isBodyCenteredFrame, isWorldFrame, isZupFrame, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public PixelsReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isBodyCenteredFrame,
                               boolean isWorldFrame, boolean isZupFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, transformToParent, isBodyCenteredFrame, isWorldFrame, isZupFrame, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public PixelsReferenceFrame(String frameName, ReferenceFrame parentFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public Vector2d getConversionToPixels()
   {
      return getSpaceConverter().getConversionToSpace(PlotterFrameSpace.PIXELS);
   }
}
