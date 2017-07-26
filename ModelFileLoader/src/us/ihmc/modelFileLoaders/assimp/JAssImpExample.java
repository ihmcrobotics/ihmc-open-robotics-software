package us.ihmc.modelFileLoaders.assimp;

import jassimp.AiMesh;
import jassimp.AiScene;
import jassimp.Jassimp;
import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import java.io.IOException;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class JAssImpExample extends Application
{
   static
   {
      NativeLibraryLoader.loadLibrary("jassimp", "jassimp");
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("Jassimp Example");

      View3DFactory view3DFactory = new View3DFactory(1024, 768);
      view3DFactory.addCameraController();

      AiScene aiScene = Jassimp.importFile("Valkyrie/resources/models/val_description/model/meshes/torso/torso.dae");

      List<AiMesh> meshes = aiScene.getMeshes();

      MeshDataHolder[] meshDataHolders = new MeshDataHolder[meshes.size()];

      for (int i = 0; i < meshes.size(); i++)
      {
         AiMesh aiMesh = meshes.get(i);
         int totalNumberOfVertices = aiMesh.getNumVertices();
         int numberOfTriangles = aiMesh.getNumFaces();
         int totalNumberOfTexturePoints = aiMesh.getNumUVComponents(2);

         Point3D32[] vertices = new Point3D32[totalNumberOfVertices];
         TexCoord2f[] texturePoints = new TexCoord2f[totalNumberOfVertices];
         Vector3D32[] vertexNormals = new Vector3D32[totalNumberOfVertices];
         int[] triangleIndices = new int[3 * totalNumberOfVertices];

         if(totalNumberOfTexturePoints != 0)
         {
            System.out.println("Total number of texture points for mesh [" + aiMesh.getName() + "]: " + totalNumberOfTexturePoints);
         }

         for (int j = 0; j < numberOfTriangles; j++)
         {
            for (int k = 0; k < 3; k++)
            {
               int currentIndex = k + (3 * j);
               int faceVertexIndex = aiMesh.getFaceVertex(j, k);

               vertices[currentIndex] = new Point3D32(aiMesh.getPositionX(faceVertexIndex), aiMesh.getPositionY(faceVertexIndex),
                                                      aiMesh.getPositionZ(faceVertexIndex));

               vertexNormals[currentIndex] = new Vector3D32(aiMesh.getNormalX(faceVertexIndex), aiMesh.getNormalY(faceVertexIndex),
                                                            aiMesh.getNormalZ(faceVertexIndex));

               texturePoints[currentIndex] = new TexCoord2f();

               for (int l = 0; l < 3; l++)
               {
                  triangleIndices[currentIndex + (3 * l)] = currentIndex;
               }
            }
         }

         meshDataHolders[i] = new MeshDataHolder(vertices, texturePoints, triangleIndices, vertexNormals);
      }

      for (int i = 0; i < meshDataHolders.length; i++)
      {
         double hue = i / (meshDataHolders.length - 1.0) * 360.0;
         MeshDataHolder meshDataHolder = meshDataHolders[i];
         Material material = new PhongMaterial(Color.hsb(hue, 0.9, 0.9));
         MeshView meshView = new MeshView();
         meshView.setMesh(JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder));
         meshView.setMaterial(material);

         view3DFactory.addNodeToView(meshView);
      }

      primaryStage.setScene(view3DFactory.getScene());
      primaryStage.show();
   }

   public static void main(String[] args) throws IOException
   {
      launch(args);
   }
}
