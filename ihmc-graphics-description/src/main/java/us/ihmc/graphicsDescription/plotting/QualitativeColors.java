package us.ihmc.graphicsDescription.plotting;

import java.awt.Color;

/**
 * Find new sets at http://colorbrewer2.org
 */
public class QualitativeColors
{
   private Color[] colors;

   public QualitativeColors()
   {
      colors = new Color[12];
      colors[0] = new Color(166, 206, 227);
      colors[1] = new Color(31, 120, 180);
      colors[2] = new Color(178, 223, 138);
      colors[3] = new Color(51, 160, 44);
      colors[4] = new Color(251, 154, 153);
      colors[5] = new Color(227, 26, 28);
      colors[6] = new Color(253, 191, 111);
      colors[7] = new Color(255, 127, 0);
      colors[8] = new Color(202, 178, 214);
      colors[9] = new Color(106, 61, 154);
      colors[10] = new Color(255, 255, 153);
      colors[11] = new Color(177, 89, 40);
   }

   public Color getColor(int index)
   {
      return colors[index % colors.length];
   }
}
