package us.ihmc.utilities.ros.types;

import java.awt.Color;
import java.util.ArrayList;

import org.apache.commons.lang3.ArrayUtils;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.MutableColor;

public class GrowablePointCloud
{
   ArrayList<Point3D> points = new ArrayList<>();
   ArrayList<Float> intensities = new ArrayList<>();
   ArrayList<Color> colors = new ArrayList<>();

   public void clear()
   {
      points.clear();
      intensities.clear();
      colors.clear();
   }

   public void addPoint(Point3D p, Color color)
   {
      addPoint(p, (color.getRed()+color.getGreen()+color.getBlue())/3.0f, color);
   }
   public void addPoint(Point3D p, float intensity)
   {
      addPoint(p, intensity, new Color(255*intensity/6000, 255*intensity/6000,255*intensity/6000));
      
   }
   public synchronized void addPoint(Point3D p, float intensity, Color color)
   {
      points.add(p);
      intensities.add(intensity);
      colors.add(color);
   }

   public Point3D[] getPoints()
   {
      return (Point3D[]) points.toArray(new Point3D[0]);
   }

   public float[] getIntensities()
   {
      return ArrayUtils.toPrimitive(intensities.toArray(new Float[0]));
   }

   public MutableColor[] getColors()
   {
      return (MutableColor[]) colors.toArray(new MutableColor[0]);
   }
   
   public int size()
   {
      return points.size();
   }
   
   public Point3D getMeanPoint()
   {
      Point3D pointMean = new Point3D();
      for(int i=0;i<points.size();i++)
      {
         pointMean.add(points.get(i));
      }
      pointMean.scale(1.0/points.size());
      return pointMean;
   }
}