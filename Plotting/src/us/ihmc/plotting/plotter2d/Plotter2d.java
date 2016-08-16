package us.ihmc.plotting.plotter2d;

import javax.swing.JPanel;

import us.ihmc.plotting.plotter2d.frames.MetersReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PixelsReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PlotterFrameSpace;
import us.ihmc.plotting.plotter2d.frames.PlotterSpaceConverter;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@SuppressWarnings("serial")
public class Plotter2d
{
   private final JPanel panel;
   private double metersToPixels = 50.0;
   
   private final PlotterSpaceConverter spaceConverter;
   private final PixelsReferenceFrame pixelsFrame;
   private final PixelsReferenceFrame screenFrame;
   private final MetersReferenceFrame metersFrame;
   
   public Plotter2d()
   {
      panel = new JPanel();
      
      spaceConverter = new PlotterSpaceConverter()
      {
         @Override
         public double getConversionToSpace(PlotterFrameSpace plotterFrameType)
         {
            return plotterFrameType == PlotterFrameSpace.METERS ? 1.0 / metersToPixels : metersToPixels;
         }
      };
      pixelsFrame = new PixelsReferenceFrame("pixelsFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      screenFrame = new PixelsReferenceFrame("screenFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslation(50.0, 100.0, 0.0);
            transformToParent.applyRotationZ(Math.PI);
         }
      };
      metersFrame = new MetersReferenceFrame("metersFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
   }
}
