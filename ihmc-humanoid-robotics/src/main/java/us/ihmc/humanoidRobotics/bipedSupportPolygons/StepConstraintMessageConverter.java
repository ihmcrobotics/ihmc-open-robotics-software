package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import controller_msgs.msg.dds.StepConstraintMessage;
import controller_msgs.msg.dds.StepConstraintsListMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class StepConstraintMessageConverter
{
   public static StepConstraintMessage convertToStepConstraintMessage(StepConstraintRegion constraintRegion)
   {
      StepConstraintMessage message = new StepConstraintMessage();

      message.getRegionOrigin().set(constraintRegion.getRegionOriginInWorld());
      message.getRegionOrientation().set(constraintRegion.getTransformToWorld().getRotation());

      constraintRegion.getNormal(message.getRegionNormal());

      message.setConcaveHullSize(constraintRegion.getConcaveHullSize());
      message.setNumberOfHolesInRegion(constraintRegion.getNumberOfHolesInRegion());

      Object<Point3D> vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();

      for (int vertexIndex = 0; vertexIndex < constraintRegion.getConcaveHullSize(); vertexIndex++)
      {
         vertexBuffer.add().set(constraintRegion.getConcaveHullVertexInRegionFrame(vertexIndex), 0.0);
      }

      for (int polygonIndex = 0; polygonIndex < constraintRegion.getNumberOfHolesInRegion(); polygonIndex++)
      {
         ConcavePolygon2DReadOnly convexPolygon = constraintRegion.getHoleInConstraintRegion(polygonIndex);
         message.getHolePolygonsSize().add(convexPolygon.getNumberOfVertices());

         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            vertexBuffer.add().set(convexPolygon.getVertex(vertexIndex), 0.0);
         }
      }

      return message;
   }

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
      message.getConcaveHullsSize().reset();
      message.getNumberOfHolesInRegion().reset();
      message.getHolePolygonsSize().reset();
      Object<Point3D> vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();
      for (int i = 0; i < constraintRegions.size(); i++)
      {
         StepConstraintRegion constraintRegion = constraintRegions.get(i);

         constraintRegion.getTransformToWorld().get(message.getRegionOrientation().add(), message.getRegionOrigin().add());
         constraintRegion.getNormal(message.getRegionNormal().add());

         message.getConcaveHullsSize().add(constraintRegion.getConcaveHullSize());
         message.getNumberOfHolesInRegion().add(constraintRegion.getNumberOfHolesInRegion());

         for (int vertexIndex = 0; vertexIndex < constraintRegion.getConcaveHullSize(); vertexIndex++)
         {
            vertexBuffer.add().set(constraintRegion.getConcaveHullVertexInRegionFrame(vertexIndex), 0.0);
         }

         for (int polygonIndex = 0; polygonIndex < constraintRegion.getNumberOfHolesInRegion(); polygonIndex++)
         {
            ConcavePolygon2DReadOnly convexPolygon = constraintRegion.getHoleInConstraintRegion(polygonIndex);
            message.getConcaveHullsSize().add(convexPolygon.getNumberOfVertices());

            for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
            {
               vertexBuffer.add().set(convexPolygon.getVertex(vertexIndex), 0.0);
            }
         }
      }
   }

   public static StepConstraintsListMessage convertToStepConstraintsListMessageFromPlanarRegions(List<PlanarRegion> constraintRegions)
   {
      StepConstraintsListMessage message = new StepConstraintsListMessage();

      Object<Point3D> vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();
      for (PlanarRegion constraintRegion : constraintRegions)
      {
         constraintRegion.getTransformToWorld().get(message.getRegionOrientation().add(), message.getRegionOrigin().add());
         constraintRegion.getNormal(message.getRegionNormal().add());

         message.getConcaveHullsSize().add(constraintRegion.getConcaveHullSize());
         message.getNumberOfHolesInRegion().add(0);

         for (int vertexIndex = 0; vertexIndex < constraintRegion.getConcaveHullSize(); vertexIndex++)
         {
            vertexBuffer.add().set(constraintRegion.getConcaveHullVertex(vertexIndex), 0.0);
         }
      }

      return message;
   }

   public static StepConstraintMessage convertToStepConstraintMessage(PlanarRegion constraintRegion)
   {
      StepConstraintMessage message = new StepConstraintMessage();

      message.getRegionOrigin().set(constraintRegion.getPoint());
      message.getRegionOrientation().set(constraintRegion.getTransformToWorld().getRotation());

      constraintRegion.getNormal(message.getRegionNormal());

      message.setConcaveHullSize(constraintRegion.getConcaveHullSize());
      message.setNumberOfHolesInRegion(0);

      Object<Point3D> vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();

      for (int vertexIndex = 0; vertexIndex < constraintRegion.getConcaveHullSize(); vertexIndex++)
      {
         vertexBuffer.add().set(constraintRegion.getConcaveHullVertex(vertexIndex), 0.0);
      }

      return message;
   }

   public static StepConstraintRegion convertToStepConstraintRegion(StepConstraintMessage message)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      if (Math.abs(AngleTools.trimAngleMinusPiToPi(message.getRegionOrientation().getAngle())) < 1.0e-3)
      {
         Vector3D regionNormal = new Vector3D(message.getRegionNormal());
         AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal);
         transformToWorld.set(regionOrientation, message.getRegionOrigin());
      }
      else
      {
         transformToWorld.set(message.getRegionOrientation(), message.getRegionOrigin());
      }

      Object<Point3D> vertexBuffer = message.getVertexBuffer();

      List<Point2D> concaveHullVertices = new ArrayList<>();
      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
      {
         concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex)));
      }

      List<ConcavePolygon2DReadOnly> holes = new ArrayList<>();

      for (int polygonIndex = 0; polygonIndex < message.getNumberOfHolesInRegion(); polygonIndex++)
      {
         upperBound += message.getHolePolygonsSize().get(polygonIndex);
         ConcavePolygon2D convexPolygon = new ConcavePolygon2D();

         for (; vertexIndex < upperBound; vertexIndex++)
            convexPolygon.addVertex(vertexBuffer.get(vertexIndex));
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
      Object<Vector3D> normals = message.getRegionNormal();
      Object<Point3D> origins = message.getRegionOrigin();

      Object<Point3D> vertexBuffer = message.getVertexBuffer();

      List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

      int upperBound = 0;
      int convexPolygonIndexStart = 0;

      for (int regionIndex = 0; regionIndex < message.getConcaveHullsSize().size(); regionIndex++)
      {
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         if (message.getRegionOrientation().isEmpty()
             || Math.abs(AngleTools.trimAngleMinusPiToPi(message.getRegionOrientation().get(regionIndex).getAngle())) < 1.0e-3)
         {
            AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normals.get(regionIndex));
            transformToWorld.set(regionOrientation, origins.get(regionIndex));
         }
         else
         {
            transformToWorld.set(message.getRegionOrientation().get(regionIndex), message.getRegionOrigin().get(regionIndex));
         }

         upperBound += message.getConcaveHullsSize().get(regionIndex);
         List<Point2D> concaveHullVertices = new ArrayList<>();

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex)));
         }

         List<ConcavePolygon2DReadOnly> holes = new ArrayList<>();

         int holePolygonIndexStart = 0;
         for (; holePolygonIndexStart < message.getNumberOfHolesInRegion().get(regionIndex); holePolygonIndexStart++)
         {
            upperBound += message.getHolePolygonsSize().get(convexPolygonIndexStart + holePolygonIndexStart);
            ConcavePolygon2D convexPolygon = new ConcavePolygon2D();

            for (; vertexIndex < upperBound; vertexIndex++)
               convexPolygon.addVertex(vertexBuffer.get(vertexIndex));
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
