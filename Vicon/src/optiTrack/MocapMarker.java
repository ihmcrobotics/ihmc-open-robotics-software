package optiTrack;

import us.ihmc.euclid.tuple3D.Vector3D;

public class MocapMarker
{
   private int id;
   private Vector3D position;
   private float markerSize;

   public MocapMarker(int id, Vector3D position, float markerSize)
   {
      this.id = id;
      this.position = position;
      this.markerSize = markerSize;
   }

   public int getId()
   {
      return id;
   }

   public Vector3D getPosition()
   {
      return position;
   }
   
   public float getMarkerSize()
   {
      return markerSize;
   }
}
