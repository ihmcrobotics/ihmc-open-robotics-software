package us.ihmc.plotting.shapes;

import java.awt.Color;
import java.awt.Graphics;
import java.io.PrintWriter;
import java.io.Serializable;
import java.util.Arrays;
import java.util.Vector;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Coordinate;

public class PointArtifact extends Artifact implements Serializable
{
   /**
    *
    */
   private static final long serialVersionUID = -1676323503716482842L;
   private final Vector<Point2d> _sonarHistory = new Vector<Point2d>();
   private int _historyLength = 1;
   private Color historyColor = Color.blue;
   int _medianFilterSize = 20;
   int _meanFilterSize = 999;
   private int size = 10;

   @SuppressWarnings("unused")
   private long _startTime;

   public PointArtifact(String id)
   {
      this(id, 1);
   }

   public PointArtifact(String id, Point2d point)
   {
      this(id, 1);

      setPoint(point);
   }


   public PointArtifact(String id, int history)
   {
      super(id);
      setType("point");
      setLevel(2);
      _startTime = System.currentTimeMillis();
      _historyLength = history;
      color = Color.red;
   }

   public void setPoint(Point2d point)
   {
      synchronized (_sonarHistory)
      {
         _sonarHistory.addElement(point);

         if (_sonarHistory.size() > _historyLength)
         {
            _sonarHistory.removeElementAt(0);
         }
      }
   }

   public void setSize(int size)
   {
      this.size = size;
   }

   public void setCoordinate(Coordinate coordinate)
   {
      _sonarHistory.addElement(new Point2d(coordinate.getX(), coordinate.getY()));

      if (_sonarHistory.size() > _historyLength)
      {
         _sonarHistory.removeElementAt(0);
      }
   }

   public Coordinate getCoordinate()
   {
      if (_sonarHistory.size() == 0)
         return null;

      return new Coordinate(_sonarHistory.get(0).x, _sonarHistory.get(0).y, Coordinate.METER);
   }

   public Point2d getPoint2d()
   {
      if (_sonarHistory.size() == 0)
         return null;

      return _sonarHistory.get(0);
   }


   public void setHistoryLength(int length)
   {
      _historyLength = length;
   }

   public void setHistoryColor(Color color)
   {
      historyColor = color;
   }

   public static double getMedian(Vector<?> buffer)
   {
      int n = buffer.size();
      double[] unsorted = new double[n];
      double[] sorted = new double[n];

      // System.out.println("median for:");
      for (int i = 0; i < n; i++)
      {
         // System.out.println("\t" + ((Double)buffer.elementAt(i)).doubleValue());
         unsorted[i] = ((Double) buffer.elementAt(i)).doubleValue();
      }

      System.arraycopy(unsorted, 0, sorted, 0, n);
      Arrays.sort(sorted);

      // System.out.println("\t=" + sorted[n/2]);
      return sorted[n / 2];
   }

   public double getMean(Vector<?> buffer)
   {
      int n = buffer.size();
      double mean = 0;

      // System.out.println("mean for:");
      for (int i = 0; i < n; i++)
      {
         // System.out.println("\t" + ((Double)buffer.elementAt(i)).doubleValue());
         mean += ((Double) buffer.elementAt(i)).doubleValue();
      }

      mean = mean / n;

      // System.out.println("\t=" + mean);
      return mean;
   }

   public double getStdDev(Vector<?> buffer, double mean)
   {
      int n = buffer.size();
      double sd = 0;

      // System.out.println("stddev for:");
      for (int i = 0; i < n; i++)
      {
         // System.out.println("\t" + ((Double)buffer.elementAt(i)).doubleValue());
         sd += Math.pow((((Double) buffer.elementAt(i)).doubleValue() - mean), 2);
      }

      sd = Math.sqrt(sd);

      // System.out.println("\t=" + sd);
      return sd;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      Vector<Double> xMedianFliter = new Vector<Double>();
      Vector<Double> yMedianFliter = new Vector<Double>();
      Vector<Double> xMeanFilter = new Vector<Double>();
      Vector<Double> yMeanFilter = new Vector<Double>();

      Point2d coordinate = null;
      synchronized (_sonarHistory)
      {
         for (int i = 0; i < _sonarHistory.size(); i++)
         {
            // paint points
            coordinate = _sonarHistory.elementAt(i);

            if (coordinate != null)
            {
               int x = Xcenter + ((int) Math.round(coordinate.x * scaleFactor) - (size / 2));
               int y = Ycenter - ((int) Math.round(coordinate.y * scaleFactor)) - (size / 2);
               if (i == (_sonarHistory.size() - 1))
               {
                  g.setColor(color);
                  g.fillOval(x, y, size, size);
               } else
               {
                  g.setColor(historyColor);
                  g.fillOval(x, y, (int) (size * 0.7), (int) (size * 0.7));
               }


               // save for median and mean
               if (i >= (_sonarHistory.size() - _medianFilterSize))
               {
                  xMedianFliter.addElement(new Double(coordinate.x));
                  yMedianFliter.addElement(new Double(coordinate.y));
               }

               if (i >= (_sonarHistory.size() - _meanFilterSize))
               {
                  xMeanFilter.addElement(new Double(coordinate.x));
                  yMeanFilter.addElement(new Double(coordinate.y));
               }
            }

         }
      }

      // paint median

      /*
       *                double xMedian = getMedian(xMeanFilter);
       *                double yMedian = getMedian(yMeanFilter);
       *                int x = Xcenter + (new Double(xMedian* scaleFactor).intValue());
       *                int y = Ycenter - (new Double(yMedian* scaleFactor).intValue());
       *                g.setColor(Color.black);
       *                g.drawOval(x -10, y-10, 20, 20);
       */

      // paint mean

      /*
       *                double xMean = getMean(xMeanFilter);
       *                double yMean = getMean(yMeanFilter);
       *                double xStdDev = getStdDev(xMeanFilter, xMean);
       *                double yStdDev = getStdDev(yMeanFilter, yMean);
       *                x = Xcenter + (new Double(xMean* scaleFactor).intValue());
       *                y = Ycenter - (new Double(yMean* scaleFactor).intValue());
       *                g.setColor(Color.blue);
       *                int xSD = new Double(xStdDev*scaleFactor).intValue();
       *                int ySD = new Double(yStdDev*scaleFactor).intValue();
       *                g.drawOval((x - (xSD/2)), (y - (ySD/2)), xSD, ySD);
       */

      // System.out.println("gps " + coordinate.getX() + " " + coordinate.getY() +
      // " " + xMeanFilter.size() + " " + (System.currentTimeMillis() - _startTime)/1000 +
      // " " + xMean + " " + yMean + " " + xStdDev + " " + yStdDev);
   }


   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
   }


   public void save(PrintWriter printWriter)
   {
      for (int i = 0; i < _sonarHistory.size(); i++)
      {
         Point2d coordinate = _sonarHistory.get(i);
         printWriter.println(coordinate.x + " " + coordinate.y);
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
