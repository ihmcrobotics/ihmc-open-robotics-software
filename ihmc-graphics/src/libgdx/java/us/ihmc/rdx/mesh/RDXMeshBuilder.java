package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Mesh;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;

public class RDXMeshBuilder extends MeshDataBuilder
{
   public void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius)
   {
      addMesh(MeshDataGenerator.ArcTorus(startAngle, endAngle, majorRadius, minorRadius, 25));
   }

   public void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, int resolution)
   {
      addMesh(MeshDataGenerator.ArcTorus(startAngle, endAngle, majorRadius, minorRadius, resolution));
   }

   public Mesh generateMesh()
   {
      return RDXMeshDataInterpreter.interpretMeshData(generateMeshDataHolder());
   }
}