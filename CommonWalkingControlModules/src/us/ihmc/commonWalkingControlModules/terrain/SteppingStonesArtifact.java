package us.ihmc.commonWalkingControlModules.terrain;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.simulationconstructionset.util.ground.steppingStones.SteppingStones;


/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SteppingStonesArtifact extends Artifact
{
   /**
    *
    */
   private static final long serialVersionUID = 2347935046167621302L;

   private final double[][][] polygonPoints, shrunkenPolygonPoints;

   private final PlotterGraphics plotterGraphics = new PlotterGraphics();
   private final Color color, shrunkenColor;

   private static final int pixels = 2;
   private static final BasicStroke stroke = new BasicStroke(pixels);

   public SteppingStonesArtifact(String name, SteppingStones steppingStones, Color color, Color shrunkenColor)
   {
      super(name);
      this.color = color;
      this.shrunkenColor = shrunkenColor;
      this.setLevel(0);

      polygonPoints = steppingStones.getConvexPolygonVertices();
      shrunkenPolygonPoints = steppingStones.getShrunkenConvexPolygonVertices();
   }


   public void drawLegend(Graphics graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      graphics.setColor(color);
      graphics.drawString("Stepping Stones", Xcenter, Ycenter);
   }

   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      graphics.setColor(color);
      if (stroke != null)
         ((Graphics2D) graphics).setStroke(stroke);

      plotterGraphics.setCenter(Xcenter, Ycenter);
      plotterGraphics.setScale(scaleFactor);

      for (int i = 0; i < polygonPoints.length; i++)
      {
         plotterGraphics.drawPolygon(graphics, polygonPoints[i]);
      }

      graphics.setColor(shrunkenColor);

      for (int i = 0; i < shrunkenPolygonPoints.length; i++)
      {
         plotterGraphics.drawPolygon(graphics, shrunkenPolygonPoints[i]);
      }

   }
   
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }
   
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
