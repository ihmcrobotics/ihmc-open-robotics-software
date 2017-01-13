package us.ihmc.graphics3DDescription.plotting.frames;

import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
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
   
   public Vector2d getConversionToMeters()
   {
      return getSpaceConverter().getConversionToSpace(PlotterFrameSpace.METERS);
   }
}
