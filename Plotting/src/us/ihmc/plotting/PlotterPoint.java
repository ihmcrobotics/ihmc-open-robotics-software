package us.ihmc.plotting;

import javax.vecmath.Point2d;
import javax.vecmath.Point2i;

public class PlotterPoint
{
   private final Point2d pointInMeters = new Point2d();
   private final Point2i pointInPixels = new Point2i();
   
   private final PlotterTransform transform;
   
   public PlotterPoint(PlotterTransform transform)
   {
      this.transform = transform;
   }
   
   public void setInPixels(int x, int y)
   {
      pointInPixels.set(x, y);
      updatePointInMeters();
   }
   
   public void setInMeters(double x, double y)
   {
      pointInMeters.set(x, y);
      updatePointInPixels();
   }
   
   private void updatePointInMeters()
   {
      double xInMeters = transform.transformPixelsToMetersX(pointInPixels.getX());
      double yInMeters = transform.transformPixelsToMetersY(pointInPixels.getY());
      pointInMeters.set(xInMeters, yInMeters);
   }
   
   private void updatePointInPixels()
   {
      int xInPixels = transform.transformMetersToPixelsX(pointInMeters.getX());
      int yInPixels = transform.transformMetersToPixelsY(pointInMeters.getY());
      pointInPixels.set(xInPixels, yInPixels);
   }
   
   public void scale(double scalar)
   {
      pointInMeters.scale(scalar);
      updatePointInPixels();
   }
   
   public void add(PlotterPoint pointToAdd)
   {
      pointInMeters.add(pointToAdd.pointInMeters);
      updatePointInPixels();
   }
   
   public void sub(PlotterPoint pointToSub)
   {
      pointInMeters.sub(pointToSub.pointInMeters);
      updatePointInPixels();
   }
   
   public int getInPixelsX()
   {
      return pointInPixels.getX();
   }
   
   public int getInPixelsY()
   {
      return pointInPixels.getY();
   }
   
   public double getInMetersX()
   {
      return pointInMeters.getX();
   }
   
   public double getInMetersY()
   {
      return pointInMeters.getY();
   }
}
