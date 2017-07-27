package us.ihmc.modelFileLoaders.assimp;

import jassimp.*;
import javafx.application.Application;
import javafx.scene.effect.BlendMode;
import javafx.scene.image.Image;
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

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class JAssImpExample extends Application
{

   private final AiBuiltInWrapperProvider wrapperProvider = new AiBuiltInWrapperProvider();

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("Jassimp Example");

      View3DFactory view3DFactory = new View3DFactory(1024, 768);
      view3DFactory.addCameraController();

      Jassimp.setLibraryLoader(new IHMCJassimpNativeLibraryLoader());

      String meshFileName = "Valkyrie/resources/models/val_description/model/meshes/torso/torso.dae";

      HashSet<AiPostProcessSteps> aiPostProcessSteps = new HashSet<>();
      aiPostProcessSteps.add(AiPostProcessSteps.FLIP_UVS);
      AiScene aiScene = Jassimp.importFile(meshFileName, aiPostProcessSteps);

      int numMaterials = aiScene.getNumMaterials();

      List<AiMesh> meshes = aiScene.getMeshes();

      MeshDataHolder[] meshDataHolders = new MeshDataHolder[meshes.size()];
      Material[] materials = new Material[meshes.size()];

      for (int i = 0; i < meshes.size(); i++)
      {
         AiMesh aiMesh = meshes.get(i);

         int materialIndex = aiMesh.getMaterialIndex();
         if (materialIndex >= 0)
         {
            AiMaterial aiMaterial = aiScene.getMaterials().get(materialIndex);

            AiColor diffuseColor = aiMaterial.getDiffuseColor(wrapperProvider);
            AiColor specularColor = aiMaterial.getSpecularColor(wrapperProvider);
            float shininess = aiMaterial.getShininess();

            int numDiffuseTextures = aiMaterial.getNumTextures(AiTextureType.DIFFUSE);

            int textureUVIndex = aiMaterial.getTextureUVIndex(AiTextureType.DIFFUSE, 0);

            int numUVComponents = aiMesh.getNumUVComponents(textureUVIndex);

            Image diffuseMap = null;
            for (int j = 0; j < numDiffuseTextures; j++)
            {
               String textureFile = aiMaterial.getTextureFile(AiTextureType.DIFFUSE, j);
               Path path = Paths.get(meshFileName);
               Path textureLocation = path.getParent().resolve(textureFile);
               diffuseMap = new Image(new FileInputStream(textureLocation.toFile()), 2048, 2048, true, true);

               AiBlendMode blendMode = aiMaterial.getBlendMode();
               System.out.println("blend mode: " + blendMode);
            }

            PhongMaterial material = new PhongMaterial(aiColorToJFXColor(diffuseColor));
            material.setSpecularPower(shininess);
            material.setSpecularColor(aiColorToJFXColor(specularColor));
            material.setDiffuseMap(diffuseMap);
            //            material.setSelfIlluminationMap(diffuseMap);
            //            material.setSpecularMap(diffuseMap);
            //            material.setBumpMap(diffuseMap);

            materials[i] = material;
         }
         else
         {
            double hue = i / (meshDataHolders.length - 1.0) * 360.0;
            Material material = new PhongMaterial(Color.hsb(hue, 0.9, 0.9));
            materials[i] = material;
         }

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

               float texCoordU = aiMesh.getTexCoordU(faceVertexIndex, 0);
               float texCoordV = aiMesh.getTexCoordV(faceVertexIndex, 0);

//               System.out.println("[u, v]: " + texCoordU + ", " + texCoordV);
               texturePoints[currentIndex] = new TexCoord2f(texCoordU, texCoordV);

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
         MeshDataHolder meshDataHolder = meshDataHolders[i];
         MeshView meshView = new MeshView();
         meshView.setMesh(JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder));
         meshView.setMaterial(materials[i]);

         view3DFactory.addNodeToView(meshView);
      }

      primaryStage.setScene(view3DFactory.getScene());
      primaryStage.show();
   }

   private Color aiColorToJFXColor(AiColor aiColor)
   {
      float alpha = aiColor.getAlpha();
      int red = (int) (aiColor.getRed() * 255.0);
      int blue = (int) (aiColor.getBlue() * 255.0);
      int green = (int) (aiColor.getGreen() * 255.0);

      return Color.rgb(red, green, blue, alpha);
   }

   public static void main(String[] args) throws IOException
   {
      launch(args);
   }
}
