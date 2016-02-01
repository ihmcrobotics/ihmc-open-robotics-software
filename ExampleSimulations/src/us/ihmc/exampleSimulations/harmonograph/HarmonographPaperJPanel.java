package us.ihmc.exampleSimulations.harmonograph;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.swing.JPanel;
import javax.vecmath.Point3d;

public class HarmonographPaperJPanel extends JPanel
{
   private static final long serialVersionUID = 6882880982176035832L;
   private final ArrayList<Point3d> points = new ArrayList<Point3d>();
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
   
   public synchronized void addPoint(Point3d point)
   {
      if (points.size() >= maxPoints) return;
      if (!points.isEmpty())
      {
         Point3d previousPoint = points.get(points.size()-1);
         if ((previousPoint.distance(point)) < 0.05 * HarmonographRobot.INCHES) return;
      }
      
      this.points.add(new Point3d(point));
//      System.out.println("addPoint: " + point);
//      if (points.size() % 1000 == 0) System.out.println("points.size() = " + points.size());
      
      int[] pointIntArray = convertPoint3dToIntArray(point);
      xPoints[points.size()-1] = pointIntArray[0];
      yPoints[points.size()-1] = pointIntArray[1];
      
      this.repaint();
   }
   
   
   
   private int[] convertPoint3dToIntArray(Point3d point)
   {
      double width = 600.0;
      double scale =  12.0 * HarmonographRobot.INCHES;
      
      int x = (int) (width * (0.5 + (point.x / scale)));
      int y = (int) (width * (0.5 + (point.y / scale)));
      
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