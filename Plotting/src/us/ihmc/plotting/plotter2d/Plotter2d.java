package us.ihmc.plotting.plotter2d;

import javax.swing.JPanel;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.plotter2d.frames.MetersReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PixelsReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PlotterFrameSpace;
import us.ihmc.plotting.plotter2d.frames.PlotterSpaceConverter;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

@SuppressWarnings("serial")
public class Plotter2d extends JPanel
{
   private Vector2d metersToPixels = new Vector2d(50.0, 50.0);
   
   private final PlotterSpaceConverter spaceConverter;
   private final PixelsReferenceFrame pixelsFrame;
   private final PixelsReferenceFrame screenFrame;
   private final MetersReferenceFrame metersFrame;
   
   private final PlotterPoint2d screenPosition;
   
   public Plotter2d()
   {
      spaceConverter = new PlotterSpaceConverter()
      {
         private Vector2d scaleVector = new Vector2d();
         
         @Override
         public Vector2d getConversionToSpace(PlotterFrameSpace plotterFrameType)
         {
            if (plotterFrameType == PlotterFrameSpace.METERS)
            {
               scaleVector.set(1.0 / metersToPixels.getX(), 1.0 / metersToPixels.getY());
            }
            else
            {
               scaleVector.set(metersToPixels.getX(), metersToPixels.getY());
            }
            return scaleVector;
         }
      };
      pixelsFrame = new PixelsReferenceFrame("pixelsFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      screenFrame = new PixelsReferenceFrame("screenFrame", pixelsFrame, spaceConverter)
      {
         
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslation(screenPosition.getX(), screenPosition.getY(), 0.0);
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
      
      screenPosition = new PlotterPoint2d(pixelsFrame, 200.0, 200.0);
      
      updateFrames();
   }
   
   private void updateFrames()
   {
      pixelsFrame.update();
      screenFrame.update();
      metersFrame.update();
   }
}
