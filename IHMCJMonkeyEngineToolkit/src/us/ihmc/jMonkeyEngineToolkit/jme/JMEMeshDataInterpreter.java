package us.ihmc.jMonkeyEngineToolkit.jme;

import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.util.BufferUtils;

import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;

public class JMEMeshDataInterpreter
{

   public static Mesh interpretMeshData(MeshDataHolder meshData)
   {
      if (meshData == null)
      {
//         // Null meshes are problematic. But empty ones seem to work ok.
         return new Mesh();
      }

      Vector3f[] vertices = JMEDataTypeUtils.vecMathTuple3fArrayToJMEVector3fArray(meshData.getVertices());
      Vector2f[] textureCoords = JMEDataTypeUtils.texCoord2fArrayToJMEVector2fArray(meshData.getTexturePoints());
      Vector3f[] normals = JMEDataTypeUtils.vecMathTuple3fArrayToJMEVector3fArray(meshData.getVertexNormals());
      int[] triangleIndices = meshData.getTriangleIndices();

      Mesh mesh = new Mesh();
      mesh.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
      if(textureCoords != null)
         mesh.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(textureCoords));
      mesh.setBuffer(Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
      mesh.setBuffer(Type.Index, 3, BufferUtils.createIntBuffer(triangleIndices));
      mesh.updateBound();

      return mesh;
   }
}
