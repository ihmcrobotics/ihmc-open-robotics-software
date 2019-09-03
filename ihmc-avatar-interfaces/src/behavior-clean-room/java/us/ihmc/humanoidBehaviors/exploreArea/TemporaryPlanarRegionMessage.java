package us.ihmc.humanoidBehaviors.exploreArea;

import java.util.ArrayList;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegion;

public class TemporaryPlanarRegionMessage
{
//   public ArrayList<TemporaryConvexPolygon2DMessage> polygons;
   public RigidBodyTransform transformToWorld;
   public int index;
   public int numberOfPolygons;
   
   public TemporaryPlanarRegionMessage()
   {

   }

   public static TemporaryPlanarRegionMessage convertToTemporaryPlanarRegionMessage(PlanarRegion planarRegion, int index)
   {
      TemporaryPlanarRegionMessage message = new TemporaryPlanarRegionMessage();

//      message.polygons = TemporaryConvexPolygon2DMessage.convertToTemporaryConvexPolygon2DMessageList(planarRegion.getConvexPolygons(), index);

      message.transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(message.transformToWorld);

      message.index = index;
      message.numberOfPolygons = planarRegion.getNumberOfConvexPolygons();

      return message;
   }

//   public static PlanarRegion convertToPlanarRegion(TemporaryPlanarRegionMessage message)
//   {
//      PlanarRegion planarRegion = new PlanarRegion(message.transformToWorld); //, TemporaryConvexPolygon2DMessage.convertToConvexPolygon2Ds(message.polygons));
//      return planarRegion;
//   }

}
