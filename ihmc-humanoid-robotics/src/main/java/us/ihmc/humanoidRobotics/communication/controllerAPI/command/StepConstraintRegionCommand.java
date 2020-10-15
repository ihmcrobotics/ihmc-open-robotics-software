package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.StepConstraintMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;

public class StepConstraintRegionCommand implements Command<StepConstraintRegionCommand, StepConstraintMessage>
{
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   private final RecyclingArrayList<Point2D> concaveHullsVertices = new RecyclingArrayList<Point2D>(20, Point2D.class);
   private final RecyclingArrayList<ConcavePolygon2D> holes = new RecyclingArrayList<>(10, ConcavePolygon2D.class);

   private final Vector3D regionOrigin = new Vector3D();
   private final Vector3D regionNormal = new Vector3D();
   private final AxisAngle regionOrientation = new AxisAngle();

   public StepConstraintRegionCommand()
   {
      clear();
   }

   @Override
   public void clear()
   {
      fromLocalToWorldTransform.setToZero();
      fromWorldToLocalTransform.setToZero();
      concaveHullsVertices.clear();
      holes.clear();
   }

   @Override
   public void setFromMessage(StepConstraintMessage message)
   {
      setRegionProperties(message.getRegionOrigin(), message.getRegionNormal());

      concaveHullsVertices.clear();

      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
         addConcaveHullVertex().set(message.getVertexBuffer().get(vertexIndex));

      holes.clear();

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
      fromLocalToWorldTransform.set(command.getTransformToWorld());
      fromWorldToLocalTransform.set(command.getTransformFromWorld());

      RecyclingArrayList<Point2D> originalConcaveHullVertices = command.getConcaveHullsVertices();
      concaveHullsVertices.clear();
      for (int i = 0; i < originalConcaveHullVertices.size(); i++)
         addConcaveHullVertex().set(originalConcaveHullVertices.get(i));

      RecyclingArrayList<ConcavePolygon2D> convexPolygons = command.getHolesInRegion();
      this.holes.clear();
      for (int i = 0; i < convexPolygons.size(); i++)
         addHoleInRegion().set(convexPolygons.get(i));
   }

   public void setRegionProperties(Tuple3DReadOnly origin, Tuple3DReadOnly normal)
   {
      regionOrigin.set(origin);
      regionNormal.set(normal);
      EuclidGeometryTools.orientation3DFromZUpToVector3D(regionNormal, regionOrientation);

      fromLocalToWorldTransform.set(regionOrientation, regionOrigin);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
   }

   public Point2D addConcaveHullVertex()
   {
      return concaveHullsVertices.add();
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
      return !concaveHullsVertices.isEmpty() && !holes.isEmpty();
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

   public RecyclingArrayList<Point2D> getConcaveHullsVertices()
   {
      return concaveHullsVertices;
   }


   public void getStepConstraintRegion(StepConstraintRegion stepConstraintRegion)
   {
      stepConstraintRegion.set(fromLocalToWorldTransform, concaveHullsVertices, holes);
   }

   @Override
   public long getSequenceId()
   {
      return -1;
   }
}
