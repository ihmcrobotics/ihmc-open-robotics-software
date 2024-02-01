package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.StepConstraintMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.PlanarRegionNormal;
import us.ihmc.robotics.geometry.PlanarRegionOrigin;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;

public class StepConstraintRegionCommand implements Command<StepConstraintRegionCommand, StepConstraintMessage>
{
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   private final RecyclingArrayList<Point2D> concaveHullVertices = new RecyclingArrayList<Point2D>(20, Point2D.class);
   private final RecyclingArrayList<ConcavePolygon2D> holes = new RecyclingArrayList<>(10, ConcavePolygon2D.class);

   private final PlanarRegionOrigin regionOrigin = new PlanarRegionOrigin(fromLocalToWorldTransform);
   private final PlanarRegionNormal regionNormal = new PlanarRegionNormal(fromLocalToWorldTransform);

   public StepConstraintRegionCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      fromLocalToWorldTransform.setToZero();
      fromWorldToLocalTransform.setToZero();
      concaveHullVertices.clear();
      for (int i = 0; i < holes.size(); i++)
         holes.get(i).clear();
      holes.clear();
   }

   @Override
   public void setFromMessage(StepConstraintMessage message)
   {
      clear();

      setRegionTransformProperties(message.getRegionOrigin(), message.getRegionOrientation());

      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
         addConcaveHullVertex().set(message.getVertexBuffer().get(vertexIndex));

      for (int polygonIndex = 0; polygonIndex < message.getNumberOfHolesInRegion(); polygonIndex++)
      {
         ConcavePolygon2D hole = holes.add();
         hole.clear();
         upperBound += message.getHolePolygonsSize().get(polygonIndex);

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            hole.addVertex(message.getVertexBuffer().get(vertexIndex));
         }
         hole.update();
      }
   }

   @Override
   public void set(StepConstraintRegionCommand command)
   {
      clear();

      fromLocalToWorldTransform.set(command.getTransformToWorld());
      fromWorldToLocalTransform.set(command.getTransformFromWorld());

      RecyclingArrayList<Point2D> originalConcaveHullVertices = command.getConcaveHullVertices();
      for (int i = 0; i < originalConcaveHullVertices.size(); i++)
         addConcaveHullVertex().set(originalConcaveHullVertices.get(i));

      RecyclingArrayList<ConcavePolygon2D> convexPolygons = command.getHolesInRegion();
      for (int i = 0; i < convexPolygons.size(); i++)
         addHoleInRegion().set(convexPolygons.get(i));
   }

   public void setRegionTransformProperties(Tuple3DReadOnly origin, Orientation3DReadOnly orientation)
   {
      fromLocalToWorldTransform.set(orientation, origin);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
   }

   public Point2D addConcaveHullVertex()
   {
      return concaveHullVertices.add();
   }

   public ConcavePolygon2D addHoleInRegion()
   {
      return this.holes.add();
   }
   
   @Override
   public Class<StepConstraintMessage> getMessageClass()
   {
      return StepConstraintMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return !concaveHullVertices.isEmpty() && !holes.isEmpty();
   }

   public Point3DReadOnly getRegionOrigin()
   {
      return regionOrigin;
   }

   public Vector3DReadOnly getRegionNormal()
   {
      return regionNormal;
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return fromLocalToWorldTransform;
   }

   public RigidBodyTransform getTransformFromWorld()
   {
      return fromWorldToLocalTransform;
   }

   public RecyclingArrayList<ConcavePolygon2D> getHolesInRegion()
   {
      return holes;
   }

   public RecyclingArrayList<Point2D> getConcaveHullVertices()
   {
      return concaveHullVertices;
   }

   public void addOffset(Vector3DReadOnly offset)
   {
      fromLocalToWorldTransform.prependTranslation(offset);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
   }

   public void getStepConstraintRegion(StepConstraintRegion stepConstraintRegion)
   {
      stepConstraintRegion.set(fromLocalToWorldTransform, concaveHullVertices, holes);
   }

   @Override
   public long getSequenceId()
   {
      return -1;
   }
}
