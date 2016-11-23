package us.ihmc.javaFXToolkit.shapes;

import javafx.scene.shape.Mesh;
import us.ihmc.graphics3DDescription.MeshDataBuilder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;

public class JavaFXMeshBuilder extends MeshDataBuilder
{
   public Mesh generateMesh()
   {
      return JavaFXMeshDataInterpreter.interpretMeshData(generateMeshDataHolder());
   }
}
