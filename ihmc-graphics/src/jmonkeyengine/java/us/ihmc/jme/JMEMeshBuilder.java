package us.ihmc.jme;

import com.jme3.scene.Mesh;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEMeshDataInterpreter;

public class JMEMeshBuilder extends MeshDataBuilder
{
   public Mesh generateMesh()
   {
      return JMEMeshDataInterpreter.interpretMeshData(generateMeshDataHolder());
   }
}