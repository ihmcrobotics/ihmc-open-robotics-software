package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import controller_msgs.StepConstraintMessage;
import controller_msgs.StepConstraintsListMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class StepConstraintMessageConverter
{
   public static StepConstraintsListMessage convertToStepConstraintsListMessage(List<StepConstraintRegion> constraintRegions)
   {
      StepConstraintsListMessage message = new StepConstraintsListMessage();

      convertToStepConstraintsListMessage(constraintRegions, message);

       return message;
   }

   public static void convertToStepConstraintsListMessage(List<StepConstraintRegion> constraintRegions, StepConstraintsListMessage message)
   {
      message.getRegionOrigin().clear();
      message.getRegionOrientation().clear();
      message.getRegionNormal().clear();
      message.getConcaveHullsSize().clear();
      message.getNumberOfHolesInRegion().clear();
      message.getHolePolygonsSize().clear();
      var vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();
      for (int i = 0; i < constraintRegions.size(); i++)
      {
         StepConstraintRegion constraintRegion = constraintRegions.get(i);

         constraintRegion.getTransformToWorld().get(message.getRegionOrientation().add().getQuaternion(), message.getRegionOrigin().add().getPoint());
         constraintRegion.getNormal(message.getRegionNormal().add().getVector());

         message.getConcaveHullsSize().add(constraintRegion.getConcaveHullSize());
         message.getNumberOfHolesInRegion().add(constraintRegion.getNumberOfHolesInRegion());

         for (int vertexIndex = 0; vertexIndex < constraintRegion.getConcaveHullSize(); vertexIndex++)
         {
            vertexBuffer.add().getPoint().set(constraintRegion.getConcaveHullVertexInRegionFrame(vertexIndex), 0.0);
         }

         for (int polygonIndex = 0; polygonIndex < constraintRegion.getNumberOfHolesInRegion(); polygonIndex++)
         {
            ConcavePolygon2DReadOnly convexPolygon = constraintRegion.getHoleInConstraintRegion(polygonIndex);
            message.getConcaveHullsSize().add(convexPolygon.getNumberOfVertices());

            for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
            {
               vertexBuffer.add().getPoint().set(convexPolygon.getVertex(vertexIndex), 0.0);
            }
         }
      }
   }

   public static StepConstraintsListMessage convertToStepConstraintsListMessageFromPlanarRegions(List<PlanarRegion> constraintRegions)
   {
      StepConstraintsListMessage message = new StepConstraintsListMessage();

      var vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();
      for (PlanarRegion constraintRegion : constraintRegions)
      {
         constraintRegion.getTransformToWorld().get(message.getRegionOrientation().add().getQuaternion(), message.getRegionOrigin().add().getPoint());
         constraintRegion.getNormal(message.getRegionNormal().add().getVector());

         message.getConcaveHullsSize().add(constraintRegion.getConcaveHullSize());
         message.getNumberOfHolesInRegion().add(0);

         for (int vertexIndex = 0; vertexIndex < constraintRegion.getConcaveHullSize(); vertexIndex++)
         {
            vertexBuffer.add().getPoint().set(constraintRegion.getConcaveHullVertex(vertexIndex), 0.0);
         }
      }

      return message;
   }

   public static StepConstraintMessage convertToStepConstraintMessage(PlanarRegion constraintRegion)
   {
      StepConstraintMessage message = new StepConstraintMessage();

      message.getRegionOrigin().getPoint().set(constraintRegion.getPoint());
      message.getRegionOrientation().getQuaternion().set(constraintRegion.getTransformToWorld().getRotation());

      constraintRegion.getNormal(message.getRegionNormal().getVector());

      message.setConcaveHullSize(constraintRegion.getConcaveHullSize());
      message.setNumberOfHolesInRegion(0);

      var vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();

      for (int vertexIndex = 0; vertexIndex < constraintRegion.getConcaveHullSize(); vertexIndex++)
      {
         vertexBuffer.add().getPoint().set(constraintRegion.getConcaveHullVertex(vertexIndex), 0.0);
      }

      return message;
   }

   public static StepConstraintRegion convertToStepConstraintRegion(StepConstraintMessage message)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      if (Math.abs(AngleTools.trimAngleMinusPiToPi(message.getRegionOrientation().getQuaternion().getAngle())) < 1.0e-3)
      {
         Vector3D regionNormal = new Vector3D(message.getRegionNormal().getVector());
         AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal);
         transformToWorld.set(regionOrientation, message.getRegionOrigin().getPoint());
      }
      else
      {
         transformToWorld.set(message.getRegionOrientation().getQuaternion(), message.getRegionOrigin().getPoint());
      }

      var vertexBuffer = message.getVertexBuffer();

      List<Point2D> concaveHullVertices = new ArrayList<>();
      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
      {
         concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex).getPoint()));
      }

      List<ConcavePolygon2DReadOnly> holes = new ArrayList<>();

      for (int polygonIndex = 0; polygonIndex < message.getNumberOfHolesInRegion(); polygonIndex++)
      {
         upperBound += message.getHolePolygonsSize().get(polygonIndex);
         ConcavePolygon2D convexPolygon = new ConcavePolygon2D();

         for (; vertexIndex < upperBound; vertexIndex++)
            convexPolygon.addVertex(vertexBuffer.get(vertexIndex).getPoint());
         convexPolygon.update();
         holes.add(convexPolygon);
      }

      return new StepConstraintRegion(transformToWorld, Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices), holes);
   }

   public static List<StepConstraintRegion> convertToStepConstraintRegionList(StepConstraintsListMessage message)
   {
      if (message == null)
         return null;

      int vertexIndex = 0;
      var normals = message.getRegionNormal();
      var origins = message.getRegionOrigin();

      var vertexBuffer = message.getVertexBuffer();

      List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

      int upperBound = 0;
      int convexPolygonIndexStart = 0;

      for (int regionIndex = 0; regionIndex < message.getConcaveHullsSize().size(); regionIndex++)
      {
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         if (message.getRegionOrientation().isEmpty()
             || Math.abs(AngleTools.trimAngleMinusPiToPi(message.getRegionOrientation().get(regionIndex).getQuaternion().getAngle())) < 1.0e-3)
         {
            AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normals.get(regionIndex).getVector());
            transformToWorld.set(regionOrientation, origins.get(regionIndex).getPoint());
         }
         else
         {
            transformToWorld.set(message.getRegionOrientation().get(regionIndex).getQuaternion(), message.getRegionOrigin().get(regionIndex).getPoint());
         }

         upperBound += message.getConcaveHullsSize().get(regionIndex);
         List<Point2D> concaveHullVertices = new ArrayList<>();

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex).getPoint()));
         }

         List<ConcavePolygon2DReadOnly> holes = new ArrayList<>();

         int holePolygonIndexStart = 0;
         for (; holePolygonIndexStart < message.getNumberOfHolesInRegion().get(regionIndex); holePolygonIndexStart++)
         {
            upperBound += message.getHolePolygonsSize().get(convexPolygonIndexStart + holePolygonIndexStart);
            ConcavePolygon2D convexPolygon = new ConcavePolygon2D();

            for (; vertexIndex < upperBound; vertexIndex++)
               convexPolygon.addVertex(vertexBuffer.get(vertexIndex).getPoint());
            convexPolygon.update();
            holes.add(convexPolygon);
         }
         convexPolygonIndexStart += holePolygonIndexStart;

         StepConstraintRegion planarRegion = new StepConstraintRegion(transformToWorld, Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices), holes);
         stepConstraintRegions.add(planarRegion);
      }

      return stepConstraintRegions;
   }
}
