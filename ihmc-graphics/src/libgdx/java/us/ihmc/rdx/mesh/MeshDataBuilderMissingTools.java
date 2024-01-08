package us.ihmc.rdx.mesh;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataBuilder;

public class MeshDataBuilderMissingTools
{
   /**
    * Accepts vertices to draw a box in the order given by Euclid Box3D.
    *
    * @param eightVertices
    * @param lineWidth
    */
   public static void addMultiLineBox(Point3DReadOnly[] eightVertices, double lineWidth, MeshDataBuilder meshDataBuilder)
   {
      if (eightVertices.length != 8)
         throw new RuntimeException("There should be 8 vertices in this array");

      meshDataBuilder.addLine(eightVertices[0], eightVertices[1], lineWidth); // x+y+z+  draw top
      meshDataBuilder.addLine(eightVertices[1], eightVertices[3], lineWidth); // x-y-z+
      meshDataBuilder.addLine(eightVertices[3], eightVertices[2], lineWidth); // x-y+z+
      meshDataBuilder.addLine(eightVertices[2], eightVertices[0], lineWidth); // x+y-z+
      meshDataBuilder.addLine(eightVertices[0], eightVertices[4], lineWidth); // x+y+z+
      meshDataBuilder.addLine(eightVertices[4], eightVertices[5], lineWidth); // x+y+z-  go down
      meshDataBuilder.addLine(eightVertices[5], eightVertices[1], lineWidth); // x-y-z-  leg 1
      meshDataBuilder.addLine(eightVertices[1], eightVertices[5], lineWidth); // x-y-z+
      meshDataBuilder.addLine(eightVertices[5], eightVertices[7], lineWidth); // x-y-z-
      meshDataBuilder.addLine(eightVertices[7], eightVertices[3], lineWidth); // x-y+z-  leg 2
      meshDataBuilder.addLine(eightVertices[3], eightVertices[7], lineWidth); // x-y+z+
      meshDataBuilder.addLine(eightVertices[7], eightVertices[6], lineWidth); // x-y+z-
      meshDataBuilder.addLine(eightVertices[6], eightVertices[2], lineWidth); // x+y-z-  leg 3
      meshDataBuilder.addLine(eightVertices[2], eightVertices[6], lineWidth); // x+y-z+
      meshDataBuilder.addLine(eightVertices[6], eightVertices[4], lineWidth); // x+y-z-
   }
}