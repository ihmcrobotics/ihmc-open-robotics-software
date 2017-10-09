package us.ihmc.graphicsDescription.plotting;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.graphicsDescription.plotting.frames.MetersReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PixelsReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PlotterReferenceFrame;

@SuppressWarnings("serial")
public class PlotterVector2d extends FrameVector2D
{
   public PlotterVector2d(PlotterVector2d frameTuple2d)
   {
      super(frameTuple2d);
   }

   public PlotterVector2d(PlotterReferenceFrame referenceFrame, double x, double y)
   {
      super(referenceFrame, x, y);
   }

   public PlotterVector2d(PlotterReferenceFrame referenceFrame, double[] vector)
   {
      super(referenceFrame, vector);
   }

   public PlotterVector2d(PlotterReferenceFrame referenceFrame, Tuple2DBasics tuple)
   {
      super(referenceFrame, tuple);
   }

   public PlotterVector2d(PlotterReferenceFrame referenceFrame)
   {
      super(referenceFrame);
   }
   
   public void changeFrame(MetersReferenceFrame metersReferenceFrame)
   {
      if (getReferenceFrame() instanceof PixelsReferenceFrame)
      {
         changeFrame(ReferenceFrame.getWorldFrame());
         scale(metersReferenceFrame.getConversionToMeters().getX(), metersReferenceFrame.getConversionToMeters().getY());
      }
      
      super.changeFrame(metersReferenceFrame);
   }

   public void changeFrame(PixelsReferenceFrame pixelsReferenceFrame)
   {
      if (getReferenceFrame() instanceof MetersReferenceFrame)
      {
         changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
         scale(pixelsReferenceFrame.getConversionToPixels().getX(), pixelsReferenceFrame.getConversionToPixels().getY());
      }
      
      super.changeFrameAndProjectToXYPlane(pixelsReferenceFrame);
   }
}
