package us.ihmc.plotting;

import javax.swing.JPanel;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;

import java.util.ArrayList;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2008</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class PlotterLegendPanel extends JPanel implements ArtifactsChangedListener
{
   private ArrayList<Artifact> artifacts = new ArrayList<Artifact>();
   private double scale;

   public PlotterLegendPanel(double scale)
   {
      this.scale = scale;

      super.setBackground(new Color(180, 220, 240));

      setPreferredSize(new Dimension(400, 800));
   }

   public void setScale(double scale)
   {
      this.scale = scale;
   }

   public void setArtifacts(ArrayList<Artifact> artifacts)
   {
      this.artifacts.clear();
      this.artifacts.addAll(artifacts);
      repaint();
   }

   public void paintComponent(Graphics g)
   {
      super.paintComponent(g);

      int deltaY = 30;

      int artifactX = 30;
      int labelX = 120;
      int y = 50;

      for (Artifact artifact : artifacts)
      {
         artifact.drawLegend(g, artifactX, y, scale);

         Color color = g.getColor();
         int red = color.getRed();
         int green = color.getGreen();
         int blue = color.getBlue();

//       Color newColor = new Color((red + 128) % 256, (green + 128) % 256, (blue + 128) % 256);
//       Color newColor = color.darker();
//       g.setColor(newColor);
         g.setColor(Color.black);

//       g.fillRect(labelX - 10, y-deltaY/2, 250, deltaY-5);

//       g.setColor(color);
         g.drawString(artifact.getID(), labelX, y);
         y = y + deltaY;
      }
   }

   public void artifactsChanged(ArrayList newArtifacts)
   {
      setArtifacts(newArtifacts);
      repaint();
   }
}
