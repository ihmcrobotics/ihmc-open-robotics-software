package us.ihmc.graphicsDescription.plotting.frames;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@SuppressWarnings("serial")
public abstract class MetersReferenceFrame extends PlotterReferenceFrame
{   
   public MetersReferenceFrame(String frameName, ReferenceFrame parentFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, PlotterFrameSpace.METERS, spaceConverter);
   }

   public MetersReferenceFrame(String frameName, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, isBodyCenteredFrame, isWorldFrame, isZupFrame, PlotterFrameSpace.METERS, spaceConverter);
   }

   public MetersReferenceFrame(String frameName, ReferenceFrame parentFrame, boolean isBodyCenteredFrame, boolean isWorldFrame, boolean isZupFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, isBodyCenteredFrame, isWorldFrame, isZupFrame, PlotterFrameSpace.METERS, spaceConverter);
   }

   public MetersReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isBodyCenteredFrame,
                               boolean isWorldFrame, boolean isZupFrame, PlotterSpaceConverter spaceConverter)
   {
      super(frameName, parentFrame, transformToParent, isBodyCenteredFrame, isWorldFrame, isZupFrame, PlotterFrameSpace.METERS, spaceConverter);
   }
   
   public Vector2D getConversionToMeters()
   {
      return getSpaceConverter().getConversionToSpace(PlotterFrameSpace.METERS);
   }
}
