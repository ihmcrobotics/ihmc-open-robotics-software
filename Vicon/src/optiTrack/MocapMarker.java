package optiTrack;

import javax.vecmath.Vector3d;

public class MocapMarker
{
   private int id;
   private Vector3d position;
   private float markerSize;

   public MocapMarker(int id, Vector3d position, float markerSize)
   {
      this.id = id;
      this.position = position;
      this.markerSize = markerSize;
   }

   public int getId()
   {
      return id;
   }

   public Vector3d getPosition()
   {
      return position;
   }
   
   public float getMarkerSize()
   {
      return markerSize;
   }
}
