package us.ihmc.humanoidBehaviors.exploreArea;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class TemporaryConvexPolygon2DMessage
{
   public ArrayList<Point2D> points;
   public int index;

   public TemporaryConvexPolygon2DMessage()
   {

   }

   public static ArrayList<TemporaryConvexPolygon2DMessage> convertToTemporaryConvexPolygon2DMessageList(List<ConvexPolygon2D> convexPolygons, int index)
   {
      ArrayList<TemporaryConvexPolygon2DMessage> polygonMessages = new ArrayList<TemporaryConvexPolygon2DMessage>();

      for (ConvexPolygon2D polygon : convexPolygons)
      {
         TemporaryConvexPolygon2DMessage message = convertToTemporaryConvexPolygon2DMessage(polygon, index);

         polygonMessages.add(message);
      }

      return polygonMessages;
   }

   public static TemporaryConvexPolygon2DMessage convertToTemporaryConvexPolygon2DMessage(ConvexPolygon2D polygon, int index)
   {
      TemporaryConvexPolygon2DMessage message = new TemporaryConvexPolygon2DMessage();
      message.points = new ArrayList<Point2D>();

      List<? extends Point2DReadOnly> polygonVerticesView = polygon.getPolygonVerticesView();

      for (Point2DReadOnly point : polygonVerticesView)
      {
         message.points.add(new Point2D(point));
      }
      
      message.index = index;
 
      return message;
   }

   public static ConvexPolygon2D convertToConvexPolygon2D(TemporaryConvexPolygon2DMessage message)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(message.points));
      return polygon;
   }

   public static ArrayList<ConvexPolygon2D> convertToConvexPolygon2Ds(ArrayList<TemporaryConvexPolygon2DMessage> polygonMessages)
   {
      ArrayList<ConvexPolygon2D> polygons = new ArrayList<ConvexPolygon2D>();

      for (TemporaryConvexPolygon2DMessage message : polygonMessages)
      {
         polygons.add(convertToConvexPolygon2D(message));
      }

      return polygons;
   }

}
