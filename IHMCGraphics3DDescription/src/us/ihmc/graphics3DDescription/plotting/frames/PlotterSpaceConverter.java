package us.ihmc.graphics3DDescription.plotting.frames;

import javax.vecmath.Vector2d;

public interface PlotterSpaceConverter
{
   public Vector2d getConversionToSpace(PlotterFrameSpace plotterFrameType);
}
