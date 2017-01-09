package us.ihmc.javaFXToolkit.shapes;

import javafx.scene.shape.Mesh;
import us.ihmc.graphics3DDescription.MeshDataBuilder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;

/**
 * Extension of {@link MeshDataBuilder} that can generate JavaFX {@link Mesh}.
 */
public class JavaFXMeshBuilder extends MeshDataBuilder
{
   public Mesh generateMesh()
   {
      return JavaFXMeshDataInterpreter.interpretMeshData(generateMeshDataHolder());
   }
}
