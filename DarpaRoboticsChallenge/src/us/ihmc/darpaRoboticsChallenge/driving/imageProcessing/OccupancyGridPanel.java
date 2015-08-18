package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import javax.swing.JPanel;

/**
 * @author Peter Abeles
 */
public class OccupancyGridPanel extends JPanel
{
   protected OccupancyGrid map = new OccupancyGrid(1, 1, 1);

   protected double cellToPixel = 1.0;

   protected boolean adjustView;

   Color gray[];

   public OccupancyGridPanel(boolean adjustView)
   {
      this.adjustView = adjustView;

      gray = new Color[256];

      for (int i = 0; i < 256; i++)
      {
         gray[i] = new Color(i, i, i);
      }
      gray[255] = Color.RED;
   }

   public void autoPreferredSize()
   {
      if (adjustView)
         setPreferredSize(new Dimension((int) (map.height * cellToPixel), (int) (map.width * cellToPixel)));
      else
         setPreferredSize(new Dimension((int) (map.width * cellToPixel), (int) (map.height * cellToPixel)));
   }

   public synchronized void setMap(OccupancyGrid map, boolean copy)
   {
      if (copy)
      {
         this.map.resize(map.getWidth(),map.getHeight());
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

            int value = (int) (255.0 * map.unsafe_get(x, y));

            g.setColor(gray[value]);

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
