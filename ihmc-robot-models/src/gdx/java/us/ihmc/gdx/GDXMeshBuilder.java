package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Mesh;
import us.ihmc.graphicsDescription.MeshDataBuilder;

public class GDXMeshBuilder extends MeshDataBuilder
{
   public Mesh generateMesh()
   {
      return GDXMeshDataInterpreter.interpretMeshData(generateMeshDataHolder());
   }
}