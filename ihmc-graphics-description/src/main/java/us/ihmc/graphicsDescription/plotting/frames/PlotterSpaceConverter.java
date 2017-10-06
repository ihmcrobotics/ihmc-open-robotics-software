package us.ihmc.graphicsDescription.plotting.frames;

import us.ihmc.euclid.tuple2D.Vector2D;

public interface PlotterSpaceConverter
{
   public Vector2D getConversionToSpace(PlotterFrameSpace plotterFrameType);
}
