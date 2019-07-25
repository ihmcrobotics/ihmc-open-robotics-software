package us.ihmc.javaFXVisualizers;

import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.Arrays;

public class JavaFXGraphicTools
{
   public static void setNodeTransformFromPose(Node node, FramePose3DBasics pose)
   {
      node.getTransforms().clear();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.set(pose.getOrientation(), pose.getPosition());
      Affine affine = JavaFXTools.convertRigidBodyTransformToAffine(rigidBodyTransform);
      node.getTransforms().add(affine);
   }

   public static void setNodeTransformFromPose(Node node, FramePose3DBasics pose, double scale)
   {
      node.getTransforms().clear();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.set(pose.getOrientation(), pose.getPosition());
      Affine affine = JavaFXTools.convertRigidBodyTransformToAffine(rigidBodyTransform);
      affine.appendScale(scale, scale);
      node.getTransforms().add(affine);
   }

   public static void setNodePosition(Node node, Tuple3DReadOnly position)
   {
      node.setTranslateX(position.getX());
      node.setTranslateY(position.getY());
      node.setTranslateZ(position.getZ());
   }

   public static void drawPlanarRegion(JavaFXMeshBuilder meshBuilder, PlanarRegion planarRegion, double lineWidth)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);

      meshBuilder.addMultiLine(transformToWorld, Arrays.asList(planarRegion.getConcaveHull()), lineWidth, true);

      for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      {
         meshBuilder.addPolygon(transformToWorld, convexPolygon);
      }
   }

   public static void drawArrow(JavaFXMeshBuilder meshBuilder,
                                Tuple3DReadOnly position,
                                Orientation3DReadOnly orientation,
                                double length,
                                double radius,
                                double cylinderToConeLengthRatio,
                                double coneDiameterMultiplier)
   {
      double cylinderLength = cylinderToConeLengthRatio * length;
      double coneLength = (1.0 - cylinderToConeLengthRatio) * length;
      double coneRadius = coneDiameterMultiplier * radius;

      AxisAngle axisAngle = new AxisAngle(orientation);
      meshBuilder.addCylinder(cylinderLength, radius, position, axisAngle);

      Vector3D coneBaseTranslation = new Vector3D(0.0, 0.0, 1.0);
      orientation.transform(coneBaseTranslation);
      coneBaseTranslation.scale(length);
      coneBaseTranslation.scale(cylinderToConeLengthRatio);
      Point3D coneBase = new Point3D(position);
      coneBase.add(coneBaseTranslation);

      meshBuilder.addCone(coneLength, coneRadius, coneBase, axisAngle);
   }

   public static void drawBoxEdges(JavaFXMeshBuilder meshBuilder, BoundingBox3DReadOnly boundingBox, double lineWidth)
   {
      drawBoxEdges(meshBuilder, GeometryTools.convertBoundingBox3DToBox3D(boundingBox), lineWidth);
   }

   public static void drawBoxEdges(JavaFXMeshBuilder meshBuilder, Box3DReadOnly box, double lineWidth)
   {
      ArrayList<Point3DReadOnly> orderedVertices = new ArrayList<>();

      orderedVertices.add(box.getVertex(0)); // x+y+z+  draw top
      orderedVertices.add(box.getVertex(1)); // x-y-z+
      orderedVertices.add(box.getVertex(3)); // x-y+z+
      orderedVertices.add(box.getVertex(2)); // x+y-z+
      orderedVertices.add(box.getVertex(0)); // x+y+z+

      orderedVertices.add(box.getVertex(4)); // x+y+z-  go down

      orderedVertices.add(box.getVertex(5)); // x-y-z-  leg 1
      orderedVertices.add(box.getVertex(1)); // x-y-z+
      orderedVertices.add(box.getVertex(5)); // x-y-z-

      orderedVertices.add(box.getVertex(7)); // x-y+z-  leg 2
      orderedVertices.add(box.getVertex(3)); // x-y+z+
      orderedVertices.add(box.getVertex(7)); // x-y+z-

      orderedVertices.add(box.getVertex(6)); // x+y-z-  leg 3
      orderedVertices.add(box.getVertex(2)); // x+y-z+
      orderedVertices.add(box.getVertex(6)); // x+y-z-

      orderedVertices.add(box.getVertex(4)); // x+y+z-  leg 4

      meshBuilder.addMultiLine(orderedVertices, lineWidth, false);
   }
}