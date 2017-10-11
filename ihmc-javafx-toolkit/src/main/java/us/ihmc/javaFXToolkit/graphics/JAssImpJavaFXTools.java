package us.ihmc.javaFXToolkit.graphics;

import jassimp.*;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.javaFXToolkit.JavaFXTools;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashSet;
import java.util.List;

public class JAssImpJavaFXTools
{
   private static final AiBuiltInWrapperProvider builtinWrapperProvider = (AiBuiltInWrapperProvider) Jassimp.BUILTIN;

   public static Color aiColorToJFXColor(AiColor aiColor)
   {
      float alpha = aiColor.getAlpha();
      int red = (int) (aiColor.getRed() * 255.0);
      int blue = (int) (aiColor.getBlue() * 255.0);
      int green = (int) (aiColor.getGreen() * 255.0);

      return Color.rgb(red, green, blue, alpha);
   }

   public static void convertAssimpMatrix4fToEuclidAffine(AiMatrix4f aiMatrix4f, AffineTransform affineTransform, boolean print)
   {
      double[] matrix = new double[16];

      for (int i = 0; i < 4; i++)
      {
         for (int j = 0; j < 4; j++)
         {
            float v = aiMatrix4f.get(i, j);
            if (print)
               System.out.println("Matrix value: [i,j]: [" + i + ", " + j + "]: " + v);
            matrix[4 * i + j] = v;
         }
      }

      affineTransform.set(matrix);
   }

   public static MeshView[] getJavaFxMeshes(String meshFileName) throws URISyntaxException, IOException
   {
      HashSet<AiPostProcessSteps> aiPostProcessSteps = new HashSet<>();
      aiPostProcessSteps.add(AiPostProcessSteps.FLIP_UVS);
      //      aiPostProcessSteps.add(AiPostProcessSteps.OPTIMIZE_GRAPH); // There are bugs with OPTIMIZE GRAPH that messes up transforms.
      aiPostProcessSteps.add(AiPostProcessSteps.OPTIMIZE_MESHES);

      AiScene aiScene;
      try
      {
         aiScene = IHMCJassimp.importFile(meshFileName, aiPostProcessSteps, new AiClassLoaderIOSystem(JAssImpJavaFXTools.class.getClassLoader()));

         AiNode sceneRoot = aiScene.getSceneRoot(builtinWrapperProvider);

         AiMatrix4f transform = sceneRoot.getTransform(builtinWrapperProvider);

         AffineTransform affineTransform = new AffineTransform();

         convertAssimpMatrix4fToEuclidAffine(transform, affineTransform, false);

         AiMetadataEntry up_axis = sceneRoot.getMetadata().get("UP_AXIS");
         //Hacky fix for Collada bug
         if (meshFileName.toLowerCase().endsWith(".dae") && up_axis != null && up_axis.getMetaDataType() == AiMetadataEntry.AiMetadataType.AI_AISTRING)
         {
            String upAxisString = (String) up_axis.getData();
            if (upAxisString.contains("UP_Z"))
            {
               affineTransform.appendRollRotation(Math.PI / 2.0);
            }

            if (upAxisString.contains("UP_X"))
            {
               affineTransform.appendPitchRotation(Math.PI / 2.0);
            }
         }

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

               AiColor diffuseColor = aiMaterial.getDiffuseColor(builtinWrapperProvider);
               AiColor specularColor = aiMaterial.getSpecularColor(builtinWrapperProvider);
               float shininess = aiMaterial.getShininess();

               Image diffuseMap = null;
               for (int j = 0; j < numDiffuseTextures; j++)
               {
                  String textureFile = aiMaterial.getTextureFile(AiTextureType.DIFFUSE, j);
                  String textureLocation = meshFileName.substring(0, meshFileName.lastIndexOf("/")) + "/" + textureFile;
                  URI normalize = new URI(textureLocation).normalize();

                  diffuseMap = new Image(JAssImpJavaFXTools.class.getClassLoader().getResourceAsStream(normalize.toString()));
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
            Affine javaFxAffineToPack = new Affine();
            JavaFXTools.convertEuclidAffineToJavaFXAffine(affineTransform, javaFxAffineToPack);
            MeshDataHolder meshDataHolder = meshDataHolders[i];
            MeshView meshView = new MeshView();
            meshView.setMesh(JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder));
            meshView.setMaterial(materials[i]);
            meshView.getTransforms().add(javaFxAffineToPack);
            meshViews[i] = meshView;
         }
         
         return meshViews;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }

}
