package us.ihmc.graphicsDescription.plotting.frames;

import javax.vecmath.Vector2d;

public interface PlotterSpaceConverter
{
   public Vector2d getConversionToSpace(PlotterFrameSpace plotterFrameType);
}
