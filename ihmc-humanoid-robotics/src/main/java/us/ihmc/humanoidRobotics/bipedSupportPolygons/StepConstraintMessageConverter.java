package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import controller_msgs.msg.dds.StepConstraintMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.geometry.AngleTools;
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
}
