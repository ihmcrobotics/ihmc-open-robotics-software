package us.ihmc.gdx.simulation.environment.object.objects;


import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;

public class GDXBuildingObject
{
   private final ArrayList<Point3D> corners = new ArrayList<>();
   private float height = 5.0f;

   public GDXBuildingObject()
   {

   }

   public Point3D getLastCorner()
   {
      if(corners.size() >= 1)
         return corners.get(corners.size() - 1);
      else
         return null;
   }



   public ArrayList<Point3D> getCorners()
   {
      return corners;
   }

   public void addCorner(Point3D corner)
   {
      corners.add(corner);
   }

   public void setHeight(float height)
   {
      this.height = height;
   }

   public float getHeight()
   {
      return height;
   }
}
