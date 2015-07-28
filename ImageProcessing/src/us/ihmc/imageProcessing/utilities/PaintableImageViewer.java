package us.ihmc.imageProcessing.utilities;

import java.awt.Graphics;
import java.util.ArrayList;

/**
 * User: Matt
 * Date: 3/5/13
 */
public class PaintableImageViewer extends ImageViewer
{
   private ArrayList<PostProcessor> postProcessors = new ArrayList<PostProcessor>();

   public void addPostProcessor(PostProcessor postProcessor)
   {
      postProcessors.add(postProcessor);
   }

   public void paint(Graphics graphics)
   {
      super.paint(graphics);
      for (PostProcessor postProcessor : postProcessors)
      {
         postProcessor.paint(graphics);
      }
   }
}
