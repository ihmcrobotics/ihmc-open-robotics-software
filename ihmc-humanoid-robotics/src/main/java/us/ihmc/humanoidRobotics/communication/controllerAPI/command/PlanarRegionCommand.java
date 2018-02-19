package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.Polygon2DMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.PreallocatedList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class PlanarRegionCommand implements Command<PlanarRegionCommand, PlanarRegionMessage>
{
   public static final int NO_REGION_ID = -1;

   private int regionId = NO_REGION_ID;
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   private final RecyclingArrayList<Point2D> concaveHullsVertices = new RecyclingArrayList<Point2D>(20, Point2D.class);
   private final RecyclingArrayList<ConvexPolygon2D> convexPolygons = new RecyclingArrayList<ConvexPolygon2D>(10, ConvexPolygon2D.class);

   private final Vector3D regionOrigin = new Vector3D();
   private final Vector3D regionNormal = new Vector3D();
   private final AxisAngle regionOrientation = new AxisAngle();

   public PlanarRegionCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      fromLocalToWorldTransform.setToZero();
      fromWorldToLocalTransform.setToZero();
      concaveHullsVertices.clear();
      convexPolygons.clear();
   }

   @Override
   public void set(PlanarRegionMessage message)
   {
      regionOrigin.set(message.getRegionOrigin());
      regionNormal.set(message.getRegionNormal());
      EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal, regionOrientation);

      fromLocalToWorldTransform.set(regionOrientation, regionOrigin);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      Polygon2DMessage concaveHullMessage = message.getConcaveHull();
      concaveHullsVertices.clear();
      for (int i = 0; i < concaveHullMessage.getVertices().size(); i++)
         concaveHullsVertices.add().set(concaveHullMessage.getVertices().get(i));

      PreallocatedList<Polygon2DMessage> convexPolygonsMessage = message.getConvexPolygons();
      convexPolygons.clear();
      for (int i = 0; i < convexPolygonsMessage.size(); i++)
      {
         Polygon2DMessage convexPolygonMessage = convexPolygonsMessage.get(i);
         ConvexPolygon2D convexPolygon = convexPolygons.add();
         for (int vertexIndex = 0; vertexIndex < convexPolygonMessage.getVertices().size(); vertexIndex++)
            convexPolygon.addVertex(convexPolygonMessage.getVertices().get(vertexIndex));
         convexPolygons.getLast().update();
      }

      regionId = message.getRegionId();
   }

   @Override
   public void set(PlanarRegionCommand command)
   {
      fromLocalToWorldTransform.set(command.getTransformToWorld());
      fromWorldToLocalTransform.set(command.getTransformFromWorld());

      RecyclingArrayList<Point2D> originalConcaveHullVertices = command.getConcaveHullsVertices();
      concaveHullsVertices.clear();
      for (int i = 0; i < originalConcaveHullVertices.size(); i++)
         concaveHullsVertices.add().set(originalConcaveHullVertices.get(i));

      RecyclingArrayList<ConvexPolygon2D> convexPolygons = command.getConvexPolygons();
      this.convexPolygons.clear();
      for (int i = 0; i < convexPolygons.size(); i++)
         this.convexPolygons.add().set(convexPolygons.get(i));

      regionId = command.getRegionId();
   }

   @Override
   public Class<PlanarRegionMessage> getMessageClass()
   {
      return PlanarRegionMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return !concaveHullsVertices.isEmpty() && !convexPolygons.isEmpty();
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return fromLocalToWorldTransform;
   }

   public RigidBodyTransform getTransformFromWorld()
   {
      return fromWorldToLocalTransform;
   }

   public RecyclingArrayList<ConvexPolygon2D> getConvexPolygons()
   {
      return convexPolygons;
   }

   public RecyclingArrayList<Point2D> getConcaveHullsVertices()
   {
      return concaveHullsVertices;
   }

   public int getRegionId()
   {
      return regionId;
   }

   public void getPlanarRegion(PlanarRegion planarRegionToPack)
   {
      planarRegionToPack.set(fromLocalToWorldTransform, convexPolygons, regionId);
   }
}
