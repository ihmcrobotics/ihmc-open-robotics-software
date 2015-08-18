package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JPanel;

import boofcv.struct.image.ImageUInt8;

/**
 * Displays a classified image from {@link DrcColorTerrainClassifier}.
 *
 * @author Peter Abeles
 */
public class TerrainClassifyPanel extends JPanel
{
   protected ImageUInt8 map = new ImageUInt8(1, 1);

   protected double cellToPixel = 1.0;

   protected boolean adjustView;

   Color colors[];

   public TerrainClassifyPanel(boolean adjustView)
   {
      this.adjustView = adjustView;

      colors = new Color[4];
      colors[0] = Color.BLACK;
      colors[1] = Color.GRAY;
      colors[2] = Color.GREEN;
      colors[3] = Color.RED;
   }

   public TerrainClassifyPanel(boolean adjustView, ImageUInt8 map)
   {
      this(adjustView);
      this.map = map;
      autoPreferredSize();
   }

   public void autoPreferredSize()
   {
      if (adjustView)
         setPreferredSize(new Dimension((int) (map.height * cellToPixel), (int) (map.width * cellToPixel)));
      else
         setPreferredSize(new Dimension((int) (map.width * cellToPixel), (int) (map.height * cellToPixel)));
   }

   public synchronized void setMap(ImageUInt8 map, boolean copy)
   {
      if (copy)
      {
         this.map.reshape(map.getWidth(), map.getHeight());
         this.map.setTo(map);
      }
      else
      {
         this.map = map;
      }
   }

   @Override
   public synchronized void paint(Graphics g)
   {
      int h = getHeight();
      int w = getWidth();

      for (int y = 0; y < map.height; y++)
      {
         int y0 = (int) (y * cellToPixel);
         int y1 = (int) ((y + 1) * cellToPixel);

         for (int x = 0; x < map.width; x++)
         {
            int x0 = (int) (x * cellToPixel);
            int x1 = (int) ((x + 1) * cellToPixel);

            int value = map.unsafe_get(x, y);

            g.setColor(colors[value]);

            if (adjustView)
            {
               x0 = h - x0 - 1;
               x1 = h - x1 - 1;
               g.fillRect(w - y0 - 1, x1, y1 - y0, x0 - x1);
            }
            else
            {
               g.fillRect(x0, y0, x1 - x0, y1 - y0);
            }
         }
      }
   }
}
