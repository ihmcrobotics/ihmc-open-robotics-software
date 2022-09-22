package us.ihmc.gdx.tools;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMesh;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D32;

import java.util.ArrayList;

public class GDXModelInstanceScaler
{
   private final ModelData modelData;
   private record MeshRecord(int floatsPerVertex, int numberOfVertices, ArrayList<Point3D32> originalVertices) { }
   private final ArrayList<MeshRecord> meshRecords = new ArrayList<>();
   private ModelInstance modelInstance;

   public GDXModelInstanceScaler(String modelFileName, double startingScaleFactor)
   {
      this(GDXModelLoader.loadModelData(modelFileName), startingScaleFactor);
   }

   public GDXModelInstanceScaler(ModelData modelData, double startingScaleFactor)
   {
      this.modelData = modelData;

      for (int i = 0; i < modelData.meshes.size; i++)
      {
         ModelMesh modelMesh = modelData.meshes.get(i);

         int floatsPerVertex = GDXTools.calculateFloatsPerVertex(modelMesh);
         int numberOfVertices = modelMesh.vertices.length / floatsPerVertex;
         // Each vertex is 8 floats: x,y,z,nx,ny,nz,u,v

         ArrayList<Point3D32> originalMeshVertices = new ArrayList<>();
         for (int j = 0; j < numberOfVertices; j++)
         {
            originalMeshVertices.add(new Point3D32(modelMesh.vertices[floatsPerVertex * j],
                                                   modelMesh.vertices[floatsPerVertex * j + 1],
                                                   modelMesh.vertices[floatsPerVertex * j + 2]));
         }
         meshRecords.add(new MeshRecord(floatsPerVertex, numberOfVertices, originalMeshVertices));
      }

      scale(startingScaleFactor);
   }

   public void scale(double scaleFactor)
   {
      float scaleFactorFloat = (float) scaleFactor;

      for (int i = 0; i < modelData.meshes.size; i++)
      {
         ModelMesh modelMesh = modelData.meshes.get(i);
         MeshRecord meshRecord = meshRecords.get(i);

         for (int j = 0; j < meshRecord.numberOfVertices(); j++)
         {
            int floatsPerVertex = meshRecord.floatsPerVertex();
            Point3D32 originalVertex = meshRecord.originalVertices().get(j);
            modelMesh.vertices[floatsPerVertex * j]     = originalVertex.getX32() * scaleFactorFloat;
            modelMesh.vertices[floatsPerVertex * j + 1] = originalVertex.getY32() * scaleFactorFloat;
            modelMesh.vertices[floatsPerVertex * j + 2] = originalVertex.getZ32() * scaleFactorFloat;
         }
      }

      Model model = new Model(modelData);
      modelInstance = new ModelInstance(model);
   }

   public Matrix4 getPoseTransform()
   {
      return modelInstance.transform;
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }
}
