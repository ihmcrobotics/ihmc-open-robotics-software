package us.ihmc.perception.elements;

import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;

public class Landmark
{
   int id = -1;
   private ArrayList<Integer> keyframesIDs;
   private Point3D point;

   public Landmark(int id)
   {
      this.id = id;
      keyframesIDs = new ArrayList<>();
   }

   public int getId()
   {
      return id;
   }

   public Point3D getPoint()
   {
      return point;
   }

   public void setPoint(Point3D point)
   {
      this.point = point;
   }

   public int getKeyframeCount()
   {
      return keyframesIDs.size();
   }

   public void addKeyframe(int id)
   {
      keyframesIDs.add(id);
   }
}
