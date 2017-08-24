package us.ihmc.graphicsDescription.plotting;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.graphicsDescription.plotting.frames.MetersReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PixelsReferenceFrame;
import us.ihmc.graphicsDescription.plotting.frames.PlotterReferenceFrame;

@SuppressWarnings("serial")
public class PlotterPoint2d extends FramePoint2D
{
   public PlotterPoint2d(PlotterPoint2d frameTuple2d)
   {
      super(frameTuple2d);
   }

   public PlotterPoint2d(PlotterReferenceFrame referenceFrame, double x, double y)
   {
      super(referenceFrame, x, y);
   }

   public PlotterPoint2d(PlotterReferenceFrame referenceFrame, double[] position)
   {
      super(referenceFrame, position);
   }

   public PlotterPoint2d(PlotterReferenceFrame referenceFrame, Tuple2DBasics position)
   {
      super(referenceFrame, position);
   }

   public PlotterPoint2d(PlotterReferenceFrame referenceFrame)
   {
      super(referenceFrame);
   }
   
   public void changeFrame(MetersReferenceFrame metersReferenceFrame)
   {
      if (getReferenceFrame() instanceof PixelsReferenceFrame)
      {
         changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
         scale(metersReferenceFrame.getConversionToMeters().getX(), metersReferenceFrame.getConversionToMeters().getY());
      }
      
      super.changeFrameAndProjectToXYPlane(metersReferenceFrame);
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
