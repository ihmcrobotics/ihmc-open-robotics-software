package us.ihmc.javaFXToolkit.graphics;

import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.List;

import jassimp.AiBuiltInWrapperProvider;
import jassimp.AiClassLoaderIOSystem;
import jassimp.AiColor;
import jassimp.AiMaterial;
import jassimp.AiMesh;
import jassimp.AiPostProcessSteps;
import jassimp.AiScene;
import jassimp.AiTextureType;
import jassimp.IHMCJassimp;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;

public class JAssImpJavaFXTools
{

   private static final AiBuiltInWrapperProvider wrapperProvider = new AiBuiltInWrapperProvider();

   public static Color aiColorToJFXColor(AiColor aiColor)
   {
      float alpha = aiColor.getAlpha();
      int red = (int) (aiColor.getRed() * 255.0);
      int blue = (int) (aiColor.getBlue() * 255.0);
      int green = (int) (aiColor.getGreen() * 255.0);

      return Color.rgb(red, green, blue, alpha);
   }

   public static MeshView[] getJavaFxMeshes(String meshFileName)
   {
      URL meshResource = JAssImpJavaFXTools.class.getClassLoader().getResource(meshFileName);

      HashSet<AiPostProcessSteps> aiPostProcessSteps = new HashSet<>();
      aiPostProcessSteps.add(AiPostProcessSteps.FLIP_UVS);
      aiPostProcessSteps.add(AiPostProcessSteps.OPTIMIZE_GRAPH);
      aiPostProcessSteps.add(AiPostProcessSteps.OPTIMIZE_MESHES);

      AiScene aiScene;
      try
      {
         aiScene = IHMCJassimp.importFile(meshFileName, aiPostProcessSteps, new AiClassLoaderIOSystem(JAssImpJavaFXTools.class.getClassLoader()));

         List<AiMesh> meshes = aiScene.getMeshes();

         MeshDataHolder[] meshDataHolders = new MeshDataHolder[meshes.size()];
         Material[] materials = new Material[meshes.size()];

         for (int i = 0; i < meshes.size(); i++)
         {
            AiMesh aiMesh = meshes.get(i);

            AiMaterial aiMaterial = null;
            int uvIndexToUse = 0;

            int materialIndex = aiMesh.getMaterialIndex();
            if (materialIndex >= 0)
            {
               aiMaterial = aiScene.getMaterials().get(materialIndex);
            }

            if (aiMaterial != null)
            {
               int numDiffuseTextures = aiMaterial.getNumTextures(AiTextureType.DIFFUSE);

               AiColor diffuseColor = aiMaterial.getDiffuseColor(wrapperProvider);
               AiColor specularColor = aiMaterial.getSpecularColor(wrapperProvider);
               float shininess = aiMaterial.getShininess();

               Image diffuseMap = null;
               for (int j = 0; j < numDiffuseTextures; j++)
               {
                  String textureFile = aiMaterial.getTextureFile(AiTextureType.DIFFUSE, j);
                  Path path = Paths.get(meshResource.toURI()); // TODO: Do not use Path, gives weird slashes on windows
                  Path textureLocation = path.getParent().resolve(textureFile);

                  diffuseMap = new Image(textureLocation.toUri().toURL().openStream());
                  uvIndexToUse = j;
               }

               PhongMaterial material = new PhongMaterial(aiColorToJFXColor(diffuseColor));
               material.setSpecularPower(shininess);
               material.setSpecularColor(aiColorToJFXColor(specularColor));
               material.setDiffuseMap(diffuseMap);

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

            Point3D32[] vertices = new Point3D32[totalNumberOfVertices];
            TexCoord2f[] texturePoints = new TexCoord2f[totalNumberOfVertices];
            Vector3D32[] vertexNormals = new Vector3D32[totalNumberOfVertices];
            int[] triangleIndices = new int[3 * totalNumberOfVertices];

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

                  if (aiMaterial != null && aiMesh.getNumUVComponents(aiMaterial.getTextureUVIndex(AiTextureType.DIFFUSE, uvIndexToUse)) == 2)
                  {
                     float texCoordU = aiMesh.getTexCoordU(faceVertexIndex, uvIndexToUse);
                     float texCoordV = aiMesh.getTexCoordV(faceVertexIndex, uvIndexToUse);

                     texturePoints[currentIndex] = new TexCoord2f(texCoordU, texCoordV);
                  }
                  else
                  {
                     texturePoints[currentIndex] = new TexCoord2f();
                  }

                  for (int l = 0; l < 3; l++)
                  {
                     triangleIndices[currentIndex + (3 * l)] = currentIndex;
                  }
               }
            }

            meshDataHolders[i] = new MeshDataHolder(vertices, texturePoints, triangleIndices, vertexNormals);
         }
         
         MeshView[] meshViews = new MeshView[meshDataHolders.length];
         
         for (int i = 0; i < meshDataHolders.length; i++)
         {
            MeshDataHolder meshDataHolder = meshDataHolders[i];
            MeshView meshView = new MeshView();
            meshView.setMesh(JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder));
            meshView.setMaterial(materials[i]);
            meshViews[i] = meshView;
         }
         
         return meshViews;
      }
      catch (IOException | URISyntaxException e)
      {
         e.printStackTrace();
         return null;
      }
   }

}
