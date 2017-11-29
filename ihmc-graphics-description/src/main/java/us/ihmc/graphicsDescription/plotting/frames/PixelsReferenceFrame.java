package us.ihmc.graphicsDescription.plotting.frames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;

public abstract class PixelsReferenceFrame extends PlotterReferenceFrame
{
   public PixelsReferenceFrame(String frameName, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public PixelsReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isWorldFrame, boolean isZupFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, isWorldFrame, isZupFrame, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public PixelsReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isWorldFrame,
                               boolean isZupFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, transformToParent, isWorldFrame, isZupFrame, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public PixelsReferenceFrame(String frameName, ReferenceFrame parentFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, PlotterFrameSpace.PIXELS, spaceConverter);
   }

   public Vector2D getConversionToPixels()
   {
      return getSpaceConverter().getConversionToSpace(PlotterFrameSpace.PIXELS);
   }
}
