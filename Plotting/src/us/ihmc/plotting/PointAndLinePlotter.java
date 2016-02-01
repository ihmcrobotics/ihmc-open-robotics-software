package us.ihmc.plotting;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Point;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.vecmath.Point2d;

/**
 * <p>ComSpline is used to test the natural cubic spline generator</p>
 *
 * @author Learning Locomotion Team
 * @version 2.0
 */
public class PointAndLinePlotter extends JPanel
{
   /**
    * 
    */
   private static final long serialVersionUID = 3112685871773037198L;
   @SuppressWarnings("unused")
   private int DEFAULT_OVAL_SIZE = 4;
   private ArrayList<Point2d[]> listOfPoints = new ArrayList<Point2d[]>();    // points to be interpolated
   private ArrayList<Color> listOfColors = new ArrayList<Color>();
   private ArrayList<Integer> listOfOvalSizes = new ArrayList<Integer>();
   private int width = 1000;
   private int height = 500;
   private double xScale;
   private double yScale;
   private double xMin, xMax;
   private double yMin, yMax;
   private boolean squareAxes = false;
   private boolean initialized = false;

   public PointAndLinePlotter()
   {
   }

   public PointAndLinePlotter(Point2d[] points, int ovalSize)
   {
      init(points, ovalSize, Color.BLACK);
   }

   public PointAndLinePlotter(ArrayList<Point2d> points, int ovalSize)
   {
      Point2d[] arrayPoints = new Point2d[points.size()];

      points.toArray(arrayPoints);

      init(arrayPoints, ovalSize, Color.BLACK);
   }

   public PointAndLinePlotter(double[] points, int ovalSize)
   {
      Point2d[] arrayOfPoints = new Point2d[points.length];

      double numberOfPoints = points.length;

      for (int i = 0; i < points.length; i++)
      {
         arrayOfPoints[i] = new Point2d();
         arrayOfPoints[i].y = points[i];
         arrayOfPoints[i].x = (double) (i + 1.0) / numberOfPoints;
      }

      init(arrayOfPoints, ovalSize, Color.BLACK);
   }

   public void setAxisSquare(boolean squareAxes)
   {
      this.squareAxes = squareAxes;
   }


   private void init(Point2d[] points, int ovalSize, Color color)
   {
      Point2d[] pointList = getArrayCopy(points);
      listOfPoints.add(pointList);
      this.setPreferredSize(new Dimension((int) (width), (int) (height)));

      JPanel panel = new JPanel();
      GridBagConstraints gbc = new GridBagConstraints();
      panel.setLayout(new GridBagLayout());

      gbc.gridx = 0;
      gbc.gridy = 0;
      gbc.weightx = 1;
      gbc.weighty = 1;
      gbc.fill = GridBagConstraints.BOTH;
      panel.add(this, gbc);

      JFrame f = new JFrame("Spline");
      f.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            System.exit(0);
         }
      });

      f.getContentPane().add(panel);

      f.pack();
      f.setVisible(true);

      // set the scale
      xMin = Double.POSITIVE_INFINITY;
      xMax = Double.NEGATIVE_INFINITY;

      yMin = Double.POSITIVE_INFINITY;
      yMax = Double.NEGATIVE_INFINITY;

      listOfColors.add(color);
      listOfOvalSizes.add(ovalSize);

      initialized = true;
   }

   private Point2d[] getArrayCopy(Point2d[] array)
   {
      Point2d[] ret = new Point2d[array.length];

      for (int i = 0; i < ret.length; i++)
      {
         ret[i] = array[i];
      }

      return ret;
   }

   private void calcuateScaleAndWindowSize()
   {
      for (Point2d[] array : listOfPoints)
      {
         for (int i = 0; i < array.length; i++)
         {
            xMin = Math.min(xMin, array[i].x);
            xMax = Math.max(xMax, array[i].x);
            yMin = Math.min(yMin, array[i].y);
            yMax = Math.max(yMax, array[i].y);
         }
      }

      xScale = 0.8 * width / (xMax - xMin + 1e-6);
      yScale = 0.5 * height / (yMax - yMin + 1e-6);

      if (squareAxes)
      {
         double scale = Math.min(xScale, yScale);
         xScale = scale;
         yScale = scale;
      }
   }


   public void update(Graphics g)
   {
      paint(g);
   }


   public void addPoints(Point2d[] newPoints, Color color, int ovalSize)
   {
      Point2d[] newPointsCopy = getArrayCopy(newPoints);

      if (!initialized)
         init(newPoints, ovalSize, color);
      else
      {
         listOfPoints.add(newPointsCopy);
         listOfColors.add(color);
         listOfOvalSizes.add(ovalSize);

         repaint();
      }
   }

   public void addPointsWithoutRepaint(Point2d[] newPoints, Color color, int ovalSize)
   {
      Point2d[] newPointsCopy = getArrayCopy(newPoints);

      if (!initialized)
         init(newPoints, ovalSize, color);
      else
      {
         listOfPoints.add(newPointsCopy);
         listOfColors.add(color);
         listOfOvalSizes.add(ovalSize);
      }
   }


   public void addPoint(Point2d newPoint, Color color, int ovalSize)
   {
      ArrayList<Point2d> newPoints = new ArrayList<Point2d>();
      newPoints.add(newPoint);

      addPoints(newPoints, color, ovalSize);
   }

   public void addPoints(ArrayList<Point2d> newPoints, Color color, int ovalSize)
   {
      Point2d[] newPointsCopy = new Point2d[newPoints.size()];
      newPoints.toArray(newPointsCopy);

      addPoints(newPointsCopy, color, ovalSize);
   }

   public void addPointsWithoutRepaint(ArrayList<Point2d> newPoints, Color color, int ovalSize)
   {
      Point2d[] newPointsCopy = new Point2d[newPoints.size()];
      newPoints.toArray(newPointsCopy);

      addPointsWithoutRepaint(newPointsCopy, color, ovalSize);
   }


// public Color getLegColor(LegName legName)
// {
//    switch (legName)
//    {
//       case FRONT_LEFT:
//       {
//          return Color.YELLOW;
//       }
//       case FRONT_RIGHT:
//       {
//          return Color.ORANGE;
//       }
//       case HIND_RIGHT:
//       {
//          return Color.BLUE;
//       }
//       case HIND_LEFT:
//       {
//          return Color.BLACK;
//       }
//    }
//
//    return Color.MAGENTA;
// }
//




   public void paint(Graphics g)
   {
      calcuateScaleAndWindowSize();


      // Clear screen and set colors
      setBackground(Color.white);
      g.setColor(Color.white);
      g.fillRect(0, 0, width, height);


      for (int j = 0; j < listOfPoints.size(); j++)
      {
         g.setColor(listOfColors.get(j));

         Point2d[] array = listOfPoints.get(j);

         int ovalSize = listOfOvalSizes.get(j);

         for (int i = 0; i < array.length; i++)
         {
            Point plotXY = getPlotXY(array[i]);

            g.fillOval(plotXY.x - ovalSize / 2, plotXY.y - ovalSize / 2, ovalSize, ovalSize);
         }
      }


      // Plot points

   }

   private Point getPlotXY(Point2d point)
   {
      Point ret = new Point();
      ret.x = (int) ((point.x - xMin) * xScale);
      ret.y = (int) ((point.y - yMin) * yScale);

      ret.y = (int) (0.75 * height) - ret.y;

      return ret;
   }
}
