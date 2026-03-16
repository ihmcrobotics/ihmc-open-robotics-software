package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import perception_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionCommand implements Command<PlanarRegionCommand, PlanarRegionMessage>
{
   public static final int NO_REGION_ID = -1;

   private long sequenceId;
   private int regionId = NO_REGION_ID;
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   private final ConvexPolygon2D convexHull = new ConvexPolygon2D();
   private final ConvexPolygon2D scaledConvexHullForPlanning = new ConvexPolygon2D();
   private final ConvexPolygon2D scaledConvexHullForController = new ConvexPolygon2D();
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
      for (int i = 0; i < convexPolygons.size(); i++)
         convexPolygons.get(i).clear();
      convexPolygons.clear();
      convexHull.clear();
   }

   @Override
   public void setFromMessage(PlanarRegionMessage message)
   {
      sequenceId = message.getSequenceId();
      setRegionProperties(message.getRegionId(), message.getRegionOrigin(), message.getRegionNormal(), message.getRegionOrientation());

      convexHull.clear();
      concaveHullsVertices.clear();

      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
      {
         addConcaveHullVertex().set(message.getVertexBuffer().get(vertexIndex));
         convexHull.addVertex(message.getVertexBuffer().get(vertexIndex));
      }

      convexHull.update();
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
      convexHull.clear();
      for (int i = 0; i < originalConcaveHullVertices.size(); i++)
      {
         addConcaveHullVertex().set(originalConcaveHullVertices.get(i));
         convexHull.addVertex(originalConcaveHullVertices.get(i));
      }
      convexHull.update();

      RecyclingArrayList<ConvexPolygon2D> convexPolygons = command.getConvexPolygons();
      this.convexPolygons.clear();
      for (int i = 0; i < convexPolygons.size(); i++)
         addConvexPolygon().set(convexPolygons.get(i));

      regionId = command.getRegionId();

      regionOrigin.set(command.regionOrigin);
      regionNormal.set(command.regionNormal);
      regionOrientation.set(command.regionOrientation);
   }

   public void setRegionProperties(int id, Tuple3DReadOnly origin, Tuple3DReadOnly normal, QuaternionReadOnly orientation)
   {
      regionId = id;
      regionOrigin.set(origin);
      regionNormal.set(normal);
      regionOrientation.set(orientation);

      fromLocalToWorldTransform.set(regionOrientation, regionOrigin);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
   }

   public void setRegionProperties(int id, RigidBodyTransform transform)
   {
      regionId = id;
      regionOrigin.set(transform.getTranslation());
      regionNormal.set(transform.getM02(), transform.getM12(), transform.getM22());
      regionOrientation.get(transform.getRotation());

      fromLocalToWorldTransform.set(transform);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
   }

   public void setPlanarRegionId(int id)
   {
      regionId = id;
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

   public Vector3D getRegionNormal()
   {
      return regionNormal;
   }

   public RigidBodyTransform getTransformFromWorld()
   {
      return fromWorldToLocalTransform;
   }

   public RecyclingArrayList<ConvexPolygon2D> getConvexPolygons()
   {
      return convexPolygons;
   }

   public ConvexPolygon2D getConvexHull()
   {
      return convexHull;
   }

   public ConvexPolygon2D getScaledConvexHullForPlanning()
   {
      return scaledConvexHullForPlanning;
   }

   public ConvexPolygon2D getScaledConvexHullForController()
   {
      return scaledConvexHullForController;
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
      planarRegionToPack.set(fromLocalToWorldTransform, convexPolygons, concaveHullsVertices, regionId);
   }

   public void updateScaledConvexHullForPlanner(ConvexPolygonScaler scaler, double distance)
   {
      scaler.scaleConvexPolygon(convexHull, distance, scaledConvexHullForPlanning);
   }

   public void updateScaledConvexHullForController(ConvexPolygonScaler scaler, double distance)
   {
      scaler.scaleConvexPolygon(convexHull, distance, scaledConvexHullForController);
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public String toString()
   {
      StringBuffer buffer = new StringBuffer();

      buffer.append("transformToWorld:\n" + fromLocalToWorldTransform + "\n");
      buffer.append("number of polygons: " + convexPolygons.size() + "\n");

      int maxNumberOfPolygonsToPrint = 5;
      for (int i = 0; i < Math.min(maxNumberOfPolygonsToPrint, convexPolygons.size()); i++)
      {
         buffer.append(convexPolygons.get(i) + "\n");
      }
      if (convexPolygons.size() > maxNumberOfPolygonsToPrint)
      {
         buffer.append("...\n");
      }

      return buffer.toString();
   }
}
