package us.ihmc.utilities.ros.types;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Color3f;
import javax.vecmath.Point3d;

import org.apache.commons.lang3.ArrayUtils;

public class GrowablePointCloud
{
   ArrayList<Point3d> points = new ArrayList<>();
   ArrayList<Float> intensities = new ArrayList<>();
   ArrayList<Color> colors = new ArrayList<>();

   public void clear()
   {
      points.clear();
      intensities.clear();
      colors.clear();
   }

   public void addPoint(Point3d p, Color color)
   {
      addPoint(p, (color.getRed()+color.getGreen()+color.getBlue())/3.0f, color);
   }
   public void addPoint(Point3d p, float intensity)
   {
      addPoint(p, intensity, new Color(255*intensity/6000, 255*intensity/6000,255*intensity/6000));
      
   }
   public synchronized void addPoint(Point3d p, float intensity, Color color)
   {
      points.add(p);
      intensities.add(intensity);
      colors.add(color);
   }

   public Point3d[] getPoints()
   {
      return (Point3d[]) points.toArray(new Point3d[0]);
   }

   public float[] getIntensities()
   {
      return ArrayUtils.toPrimitive(intensities.toArray(new Float[0]));
   }

   public Color3f[] getColors()
   {
      return (Color3f[]) colors.toArray(new Color3f[0]);
   }
   
   public int size()
   {
      return points.size();
   }
   
   public Point3d getMeanPoint()
   {
      Point3d pointMean = new Point3d();
      for(int i=0;i<points.size();i++)
      {
         pointMean.add(points.get(i));
      }
      pointMean.scale(1.0/points.size());
      return pointMean;
   }
}