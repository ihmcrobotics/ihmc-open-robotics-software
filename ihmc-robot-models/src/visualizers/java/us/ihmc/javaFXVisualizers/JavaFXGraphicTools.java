package us.ihmc.javaFXVisualizers;

import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotics.geometry.GeometryTools;

import java.util.ArrayList;

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

   public static void drawBoxEdges(JavaFXMeshBuilder meshBuilder, BoundingBox3DReadOnly boundingBox, double lineWidth)
   {
      Point3DReadOnly minPoint = boundingBox.getMinPoint();
      Point3DReadOnly maxPoint = boundingBox.getMaxPoint();

      Point3D boxCenter = GeometryTools.midpoint(minPoint, maxPoint);
      Vector3D size = GeometryTools.vector(minPoint, maxPoint);

      Box3D box = new Box3D(boxCenter, new Quaternion(), size.getX(), size.getY(), size.getZ());

      drawBoxEdges(meshBuilder, box, lineWidth);
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