package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;

@RosMessagePacket(documentation = "This message specifies a planar region in the world.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE)
public class PlanarRegionMessage extends Packet<PlanarRegionMessage>
{
   public static final int NO_REGION_ID = -1;

   private int regionId = NO_REGION_ID;
   private RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();
   private Point2D[] concaveHullsVertices;
   /**
    * List of the convex polygons representing this planar region. They are in the local frame of
    * the plane.
    */
   private List<ConvexPolygon2D> convexPolygons;


   /**
    * Empty constructor for serialization
    */
   public PlanarRegionMessage()
   {
   }

   public PlanarRegionMessage(RigidBodyTransform transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      this(transformToWorld, new Point2D[0], planarRegionConvexPolygons);
   }

   public PlanarRegionMessage(RigidBodyTransform transformToWorld, Point2D[] concaveHullVertices, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
      this.concaveHullsVertices = concaveHullVertices;
      convexPolygons = planarRegionConvexPolygons;
   }

   public PlanarRegionMessage(RigidBodyTransform transformToWorld, ConvexPolygon2D convexPolygon)
   {
      fromWorldToLocalTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
      concaveHullsVertices = new Point2D[0];
      convexPolygons = new ArrayList<>();
      convexPolygons.add(convexPolygon);
   }

   public PlanarRegionMessage(PlanarRegionMessage planarRegionMessage)
   {
      this.fromWorldToLocalTransform.set(planarRegionMessage.fromWorldToLocalTransform);
      this.fromLocalToWorldTransform.set(planarRegionMessage.fromLocalToWorldTransform);
      this.regionId = planarRegionMessage.regionId;

      this.concaveHullsVertices = new Point2D[planarRegionMessage.concaveHullsVertices.length];
      for (int i = 0; i < planarRegionMessage.concaveHullsVertices.length; i++)
      {
         this.concaveHullsVertices[i] = new Point2D(planarRegionMessage.concaveHullsVertices[i]);
      }

      this.convexPolygons = new ArrayList<>();
      for (int i = 0; i < planarRegionMessage.convexPolygons.size(); i++)
      {
         this.convexPolygons.add(new ConvexPolygon2D(planarRegionMessage.convexPolygons.get(i)));
      }
   }

   public PlanarRegionMessage(PlanarRegion planarRegion)
   {
      planarRegion.getTransformToWorld(fromLocalToWorldTransform);
      this.fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
      this.regionId = planarRegion.getRegionId();

      this.concaveHullsVertices = new Point2D[planarRegion.getConcaveHullSize()];
      for (int i = 0; i < planarRegion.getConcaveHullSize(); i++)
      {
         this.concaveHullsVertices[i] = new Point2D(planarRegion.getConcaveHullVertex(i));
      }

      this.convexPolygons = new ArrayList<>();
      for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
      {
         this.convexPolygons.add(new ConvexPolygon2D(planarRegion.getConvexPolygon(i)));
      }
   }

   @Override
   public PlanarRegionMessage clone()
   {
      return new PlanarRegionMessage(this);
   }

   public void setTransformToWorld(RigidBodyTransform transformToWorld)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
   }

   public void setConvexPolygons(List<ConvexPolygon2D> convexPolygons)
   {
      this.convexPolygons.clear();
      this.convexPolygons.addAll(convexPolygons);
   }

   public void setConcaveHullsVertices(Point2D[] concaveHullsVertices)
   {
      this.concaveHullsVertices = concaveHullsVertices;
   }

   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return fromLocalToWorldTransform;
   }

   public RigidBodyTransform getTransformFromWorld()
   {
      return fromWorldToLocalTransform;
   }

   public Point2D[] getConcaveHullVertices()
   {
      return concaveHullsVertices;
   }

   public List<ConvexPolygon2D> getConvexPolygons()
   {
      return convexPolygons;
   }

   public int getRegionId()
   {
      return regionId;
   }

   @Override
   public String toString()
   {
      String ret = "TODO";
      return ret;
   }

   @Override
   public boolean epsilonEquals(PlanarRegionMessage planarRegionMessage, double epsilon)
   {
      boolean transformToWorldEquals = fromLocalToWorldTransform.epsilonEquals(planarRegionMessage.fromLocalToWorldTransform, epsilon);
      boolean transformToLocalEquals = fromWorldToLocalTransform.epsilonEquals(planarRegionMessage.fromWorldToLocalTransform, epsilon);

      boolean convexPolygonsEqual = true;
      if (this.convexPolygons.size() != planarRegionMessage.convexPolygons.size())
         convexPolygonsEqual = false;

      if (convexPolygonsEqual)
      {
         for (int i = 0; i < convexPolygons.size(); i++)
         {
            if (convexPolygons.get(i).epsilonEquals(planarRegionMessage.convexPolygons.get(i), epsilon))
               convexPolygonsEqual = false;
         }
      }

      boolean concaveHullEqual = true;
      if (concaveHullsVertices.length != planarRegionMessage.concaveHullsVertices.length)
         concaveHullEqual = false;

      if (concaveHullEqual)
      {
         for (int i = 0; i < concaveHullsVertices.length; i++)
         {
            if (concaveHullsVertices[i].epsilonEquals(planarRegionMessage.concaveHullsVertices[i], epsilon));
            concaveHullEqual = false;
         }
      }

      boolean idEqual = regionId == planarRegionMessage.regionId;

      return transformToWorldEquals && transformToLocalEquals && convexPolygonsEqual && concaveHullEqual && idEqual;
   }

   /** {@inheritDoc} */
   @Override
   public String validateMessage()
   {
      return PacketValidityChecker.validatePlanarRegionMessage(this);
   }
}
