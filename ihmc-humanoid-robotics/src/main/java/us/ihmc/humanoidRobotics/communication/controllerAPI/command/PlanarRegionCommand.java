package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.packets.walking.PlanarRegionMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;

import java.util.List;

public class PlanarRegionCommand implements Command<PlanarRegionCommand, PlanarRegionMessage>
{
   public static final int NO_REGION_ID = -1;

   private int regionId = NO_REGION_ID;
   private RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   private RecyclingArrayList<Point2D> concaveHullsVertices = new RecyclingArrayList<Point2D>(20, Point2D.class);
   private RecyclingArrayList<ConvexPolygon2D> convexPolygons = new RecyclingArrayList<ConvexPolygon2D>(10, ConvexPolygon2D.class);

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
      fromLocalToWorldTransform.set(message.getTransformToWorld());
      fromWorldToLocalTransform.set(message.getTransformFromWorld());

      Point2D[] originalConcaveHullVertices = message.getConcaveHullVertices();
      concaveHullsVertices.clear();
      for (int i = 0; i < originalConcaveHullVertices.length; i++)
         concaveHullsVertices.add().set(originalConcaveHullVertices[i]);

      List<ConvexPolygon2D> convexPolygons = message.getConvexPolygons();
      this.convexPolygons.clear();
      for (int i = 0; i < convexPolygons.size(); i++)
         this.convexPolygons.add().set(convexPolygons.get(i));

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
}
