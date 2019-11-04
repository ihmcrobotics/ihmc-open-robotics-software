package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionCommand implements Command<PlanarRegionCommand, PlanarRegionMessage>
{
   public static final int NO_REGION_ID = -1;

   private long sequenceId;
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
      sequenceId = 0;
      fromLocalToWorldTransform.setToZero();
      fromWorldToLocalTransform.setToZero();
      concaveHullsVertices.clear();
      convexPolygons.clear();
   }

   @Override
   public void setFromMessage(PlanarRegionMessage message)
   {
      sequenceId = message.getSequenceId();
      setRegionProperties(message.getRegionId(), message.getRegionOrigin(), message.getRegionNormal());

      concaveHullsVertices.clear();

      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
         addConcaveHullVertex().set(message.getVertexBuffer().get(vertexIndex));

      convexPolygons.clear();

      for (int polygonIndex = 0; polygonIndex < message.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = convexPolygons.add();
         convexPolygon.clear();
         upperBound += message.getConvexPolygonsSize().get(polygonIndex);

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            convexPolygon.addVertex(message.getVertexBuffer().get(vertexIndex));
         }
         convexPolygon.update();
      }
   }

   @Override
   public void set(PlanarRegionCommand command)
   {
      sequenceId = command.sequenceId;
      fromLocalToWorldTransform.set(command.getTransformToWorld());
      fromWorldToLocalTransform.set(command.getTransformFromWorld());

      RecyclingArrayList<Point2D> originalConcaveHullVertices = command.getConcaveHullsVertices();
      concaveHullsVertices.clear();
      for (int i = 0; i < originalConcaveHullVertices.size(); i++)
         addConcaveHullVertex().set(originalConcaveHullVertices.get(i));

      RecyclingArrayList<ConvexPolygon2D> convexPolygons = command.getConvexPolygons();
      this.convexPolygons.clear();
      for (int i = 0; i < convexPolygons.size(); i++)
         addConvexPolygon().set(convexPolygons.get(i));

      regionId = command.getRegionId();
   }

   public void setRegionProperties(int id, Tuple3DReadOnly origin, Tuple3DReadOnly normal)
   {
      regionId = id;
      regionOrigin.set(origin);
      regionNormal.set(normal);
      EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal, regionOrientation);

      fromLocalToWorldTransform.set(regionOrientation, regionOrigin);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
   }

   public Point2D addConcaveHullVertex()
   {
      return concaveHullsVertices.add();
   }

   public ConvexPolygon2D addConvexPolygon()
   {
      return this.convexPolygons.add();
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

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
