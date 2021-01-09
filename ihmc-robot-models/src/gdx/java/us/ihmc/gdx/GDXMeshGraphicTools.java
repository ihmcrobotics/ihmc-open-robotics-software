package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.ArrayList;

public class GDXMeshGraphicTools
{
   public static void drawArrow(GDXMultiColorMeshBuilder meshBuilder,
                                Tuple3DReadOnly position,
                                Orientation3DReadOnly orientation,
                                double length,
                                double radius,
                                double cylinderToConeLengthRatio,
                                double coneDiameterMultiplier,
                                Color color)
   {
      double cylinderLength = cylinderToConeLengthRatio * length;
      double coneLength = (1.0 - cylinderToConeLengthRatio) * length;
      double coneRadius = coneDiameterMultiplier * radius;

      AxisAngle axisAngle = new AxisAngle(orientation);
      meshBuilder.addCylinder(cylinderLength, radius, position, axisAngle, color);

      Vector3D coneBaseTranslation = new Vector3D(0.0, 0.0, 1.0);
      orientation.transform(coneBaseTranslation);
      coneBaseTranslation.scale(length);
      coneBaseTranslation.scale(cylinderToConeLengthRatio);
      Point3D coneBase = new Point3D(position);
      coneBase.add(coneBaseTranslation);

      meshBuilder.addCone(coneLength, coneRadius, coneBase, axisAngle, color);
   }


   public static void drawBoxEdges(GDXMultiColorMeshBuilder meshBuilder, Box3DReadOnly box, double lineWidth, Color color)
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

      meshBuilder.addMultiLine(orderedVertices, lineWidth, color, false);
   }
}
