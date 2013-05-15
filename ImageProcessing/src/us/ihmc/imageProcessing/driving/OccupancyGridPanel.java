package us.ihmc.imageProcessing.driving;

import javax.swing.*;
import java.awt.*;

/**
 * @author Peter Abeles
 */
public class OccupancyGridPanel extends JPanel
{

   OccupancyGrid map = new OccupancyGrid(1,1,1);

   double cellToPixel = 1.0;

   boolean flipY;

   Color gray[];

   public OccupancyGridPanel(boolean flipY)
   {
      this.flipY = flipY;

      gray = new Color[256];

      for( int i = 0; i < 256; i++ ) {
         gray[i] = new Color(i,i,i);
      }
   }

   public void autoPreferredSize() {
      setPreferredSize(new Dimension((int)(map.width*cellToPixel),(int)(map.height*cellToPixel)));
   }

   public synchronized void setMap( OccupancyGrid map , boolean copy ) {
      if( copy ) {
         this.map.setTo(map);
      } else {
         this.map = map;
      }
   }

   @Override
   public synchronized void paint(Graphics g)
   {
      int h = getHeight();

      for( int y = 0; y < map.height; y++ ) {
         int y0 = (int)(y*cellToPixel);
         int y1 = (int)((y+1)*cellToPixel);

         if( flipY ) {
            y1 = h - y1 - 1;
            y0 = h - y0 - 1;
            int tmp = y1;
            y1 = y0;
            y0 = tmp;
         }

         for( int x = 0; x < map.width; x++ ) {
            int x0 = (int)(x*cellToPixel);
            int x1 = (int)((x+1)*cellToPixel);

            int value = (int)(255.0*map.unsafe_get(x,y));

            g.setColor(gray[value]);

            g.fillRect(x0,y0,x1-x0,y1-y0);
         }
      }
   }
}
