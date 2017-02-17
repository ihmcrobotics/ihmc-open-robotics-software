package us.ihmc.exampleSimulations.harmonograph;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JPanel;

import us.ihmc.euclid.tuple3D.Point3D;

public class HarmonographPaperJPanel extends JPanel
{
   private static final long serialVersionUID = 6882880982176035832L;
   private final ArrayList<Point3D> points = new ArrayList<Point3D>();
   private final int maxPoints = 64000;
   private final int[] xPoints = new int[maxPoints];
   private final int[] yPoints = new int[maxPoints];
   
   public HarmonographPaperJPanel()
   {
      
   }
   
   public synchronized void clearPoints()
   {
      points.clear();
   }
   
   public synchronized void addPoint(Point3D point)
   {
      if (points.size() >= maxPoints) return;
      if (!points.isEmpty())
      {
         Point3D previousPoint = points.get(points.size()-1);
         if ((previousPoint.distance(point)) < 0.05 * HarmonographRobot.INCHES) return;
      }
      
      this.points.add(new Point3D(point));
//      System.out.println("addPoint: " + point);
//      if (points.size() % 1000 == 0) System.out.println("points.size() = " + points.size());
      
      int[] pointIntArray = convertPoint3dToIntArray(point);
      xPoints[points.size()-1] = pointIntArray[0];
      yPoints[points.size()-1] = pointIntArray[1];
      
      this.repaint();
   }
   
   
   
   private int[] convertPoint3dToIntArray(Point3D point)
   {
      double width = 600.0;
      double scale =  12.0 * HarmonographRobot.INCHES;
      
      int x = (int) (width * (0.5 + (point.getX() / scale)));
      int y = (int) (width * (0.5 + (point.getY() / scale)));
      
      return new int[]{x, y};
   }

   protected void paintComponent(Graphics g) 
   {
      super.paintComponent(g);
      g.setColor(Color.red);
      
      synchronized(this)
      {
         g.drawPolyline(xPoints, yPoints, points.size());
      }

   }
}