package us.ihmc.jMonkeyEngineToolkit.jme;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import org.apache.commons.lang3.SystemUtils;
import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jme3.material.Material;
import com.jme3.material.MaterialList;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.material.RenderState.FaceCullMode;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture.WrapMode;
import com.jme3.texture.Texture2D;
import com.jme3.texture.plugins.AWTLoader;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.HeightBasedTerrainBlend;
import us.ihmc.graphicsDescription.appearance.HeightBasedTerrainBlend.TextureDefinition;
import us.ihmc.graphicsDescription.appearance.SDFAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.tools.ClassLoaderTools;
import us.ihmc.tools.io.files.FileTools;

public class JMEAppearanceMaterial
{
   private static final boolean DEBUG = false;

   private static final int alphaMapSize = 512;
   private static final String PHONG_ILLUMINATED_JME_MAT = "Common/MatDefs/Light/Lighting.j3md";
   private static AWTLoader awtLoader = new AWTLoader();

   private static String GAZEBO_MATERIAL_CACHE = null;

   private static boolean isMaterialFile(File file)
   {
      if (!file.isFile())
      {
         return false;
      }

      int i = file.getName().lastIndexOf('.');
      if (i > 0)
      {
         String ext = file.getName().substring(i + 1);
         return "material".equals(ext);
      }
      return false;
   }

   private static Material createMaterialFromHeightBasedTerrainBlend(JMEAssetLocator assetLocator, HeightBasedTerrainBlend appearanceDefinition)
   {
      Material material = new Material(assetLocator.getAssetManager(), "Common/MatDefs/Terrain/TerrainLighting.j3md");

      ArrayList<ImmutablePair<Double, Double>> blendMap = appearanceDefinition.getBlends();
      if (blendMap.size() > 4)
      {
         throw new RuntimeException("Only 4 blends are supported");
      }

      HeightMap heightMap = appearanceDefinition.getHeightMap();
      BoundingBox3d boundingBox = heightMap.getBoundingBox();

      double xMin = boundingBox.getXMin();
      double xStep = (boundingBox.getXMax() - boundingBox.getXMin()) / ((double) alphaMapSize);
      double yMin = boundingBox.getYMin();
      double yStep = (boundingBox.getYMax() - boundingBox.getYMin()) / ((double) alphaMapSize);

      BufferedImage alphaMap = new BufferedImage(alphaMapSize, alphaMapSize, BufferedImage.TYPE_INT_ARGB);
      for (int x = 0; x < alphaMapSize; x++)
      {
         double xCoor = xMin + x * xStep;
         for (int y = 0; y < alphaMapSize; y++)
         {
            double yCoor = yMin + y * yStep;
            double height = heightMap.heightAt(xCoor, yCoor, 0.0);
            int color = 255;

            int layer = 0;
            double fade = 1.0;
            double layerHeight = Double.MIN_VALUE;

            for (int i = 0; i < blendMap.size(); i++)
            {
               double minHeight = blendMap.get(i).getLeft();
               double fadeHeight = blendMap.get(i).getRight();

               if (height > minHeight && minHeight > layerHeight)
               {
                  layer++;
                  layerHeight = minHeight;

                  if (height < (minHeight + fadeHeight))
                  {
                     fade = (height - minHeight) / fadeHeight;
                  }

               }

            }

            color = ((int) (fade * 255.0)) << (8 * layer);
            if (layer > 0)
            {
               color |= ((int) ((1.0 - fade) * 255.0)) << (8 * (layer - 1));
            }

            color = color | (255 << 24);
            alphaMap.setRGB(y, x, color);

         }
      }

      Image alphaImage = awtLoader.load(alphaMap, true);
      Texture alphaTexture = new Texture2D(alphaImage);
      material.setTexture("AlphaMap", alphaTexture);

      ArrayList<TextureDefinition> textures = appearanceDefinition.getTextures();
      for (int i = 0; i < textures.size(); i++)
      {
         TextureDefinition texture = textures.get(textures.size() - (i + 1));
         Texture diffuse = assetLocator.loadTexture(texture.getDiffuse());
         Texture normal = assetLocator.loadTexture(texture.getNormal());
         double scale = texture.getScale();

         diffuse.setWrap(WrapMode.Repeat);
         normal.setWrap(WrapMode.Repeat);

         String ext = "";
         if (i > 0)
         {
            ext = "_" + i;
         }

         material.setTexture("DiffuseMap" + ext, diffuse);
         material.setTexture("NormalMap" + ext, normal);
         material.setFloat("DiffuseMap_" + i + "_scale", (float) scale);

      }

      //      material.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);

      return material;
   }

   private static void updateOgreMaterials(File file, MaterialList materials, JMEAssetLocator contentMan)
   {
      String matPath = null;
      try
      {
         matPath = file.getCanonicalPath();
         printIfDebug("updateOgreMaterials: matPath = " + matPath);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      if (isMaterialFile(file))
      {
         if (SystemUtils.IS_OS_WINDOWS)
         {
            matPath = stripDiskRootFromPath(matPath);
         }
         materials.putAll(contentMan.loadOgreAsset(matPath));
      }
      else
      {
         for (File subFile : file.listFiles())
         {
            if (isMaterialFile(subFile))
            {
               if (SystemUtils.IS_OS_WINDOWS)
               {
                  matPath = stripDiskRootFromPath(matPath);
               }
               materials.putAll(contentMan.loadOgreAsset(subFile.getPath()));
            }
            else if (subFile.isDirectory())
            {
               updateOgreMaterials(subFile, materials, contentMan);
            }
         }
      }
   }

   public static Material createMaterialFromSDFAppearance(JMEAssetLocator contentMan, SDFAppearance appearanceDefinition)
   {
      MaterialList materials = new MaterialList();

      String ogreShaderDir = updateGazeboMaterialCache();

      for (String appearanceDefinitionURL : appearanceDefinition.getUrls())
      {
         if (appearanceDefinitionURL.contains("axl"))
         {
            updateOgreMaterials(new File(JMEAppearanceMaterial.class.getClassLoader().getResource(appearanceDefinitionURL).getFile()), materials, contentMan);
         }
         else
         {
            String pathname = ogreShaderDir + appearanceDefinitionURL;
            File file = new File(pathname);
            // The Polaris is missing texture files, so this is to avoid excessive error messages
            if (!file.exists() && !pathname.contains("polaris") && !pathname.contains("gazebo.material"))
            {
               System.err.println("createMaterialFromSDFAppearance(): Can't find material file " + pathname);
            }
            updateOgreMaterials(file, materials, contentMan);
         }
      }

      Material mat = materials.get(appearanceDefinition.getName());

      if (mat == null)
      {
         System.err.println("Cannot load material " + appearanceDefinition.getName());
         mat = createMaterial(contentMan, YoAppearance.White());
      }
      //      mat.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);

      return mat;
   }

   private static String stripDiskRootFromPath(String path)
   {
      File[] roots = File.listRoots();
      for (File root : roots)
      {
         if (path.startsWith(root.getAbsolutePath()))
         {
            return path.replace(root.getAbsolutePath(), "");
         }
      }
      return path;
   }

   private synchronized static String updateGazeboMaterialCache()
   {

      if (GAZEBO_MATERIAL_CACHE != null)
      {
         return GAZEBO_MATERIAL_CACHE;
      }
      else
      {
         Path temporaryDirectoryPath = FileTools.getTemporaryDirectoryPath();
        
         printIfDebug("temporaryDirectoryPathName = " + temporaryDirectoryPath);
         
         Path ogreCacheDirectoryPath = temporaryDirectoryPath.resolve("SCSCache").resolve("ogre_materials");
         
         printIfDebug("ogreCacheDirectoryName = " + ogreCacheDirectoryPath);

         File cacheDir = ogreCacheDirectoryPath.toFile();
         if (!cacheDir.exists())
         {
            cacheDir.mkdir();
         }

         try
         {
            Path path = cacheDir.toPath();
            ClassLoaderTools.copyToFileSystem(path, "models/gazebo/media/materials");
            GAZEBO_MATERIAL_CACHE = cacheDir.getAbsolutePath() + File.separator + "models" + File.separator + "gazebo" + File.separator + "media" + File.separator + "materials" + File.separator;
         }
         catch (IOException e)
         {
            e.printStackTrace();
            GAZEBO_MATERIAL_CACHE = "";
         }
         return GAZEBO_MATERIAL_CACHE;

      }
   }

   private static void printIfDebug(String string)
   {
      if (DEBUG) System.out.println(string);
   }

   public static Material createMaterialFromBufferedImage(JMEAssetLocator contentMan, BufferedImage bufferedImage)
   {
      Material material = new Material(contentMan.getAssetManager(), PHONG_ILLUMINATED_JME_MAT);
      Image image = awtLoader.load(bufferedImage, true);
      Texture texture = new Texture2D(image);
      material.setTexture("DiffuseMap", texture);
      if (bufferedImage.getColorModel().hasAlpha())
      {
         material.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
         material.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
      }

      //      material.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
      return material;
   }

   public static Material createMaterialFromFileURL(JMEAssetLocator contentMan, String path)
   {
      Material material = new Material(contentMan.getAssetManager(), PHONG_ILLUMINATED_JME_MAT);
      Texture texture = contentMan.loadTexture(path);
      material.setTexture("DiffuseMap", texture);
      //      material.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
      return material;
   }

   public static Material createMaterialFromYoAppearanceTexture(JMEAssetLocator contentMan, YoAppearanceTexture appearanceDefinition)
   {
      Material material;
      if (appearanceDefinition.getPath() != null)
      {
         material = createMaterialFromFileURL(contentMan, appearanceDefinition.getPath());
      }
      else
      {
         material = createMaterialFromBufferedImage(contentMan, appearanceDefinition.getBufferedImage());
      }

      //      material.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);
      return material;

   }

   public static Material createMaterialFromYoAppearanceRGBColor(JMEAssetLocator contentMan, YoAppearanceRGBColor appearanceDefinition)
   {
      MutableColor rgb = appearanceDefinition.getColor();

      return createMaterialFromProperties(contentMan, rgb, rgb, rgb, 0.0, appearanceDefinition.getTransparency());
   }

   public static Material createMaterialFromYoAppearanceMaterial(JMEAssetLocator contentMan, YoAppearanceMaterial appearanceMaterial)
   {
      return createMaterialFromProperties(contentMan, appearanceMaterial.getDiffuseColor(), appearanceMaterial.getAmbientColor(),
            appearanceMaterial.getSpecularColor(), appearanceMaterial.getShininess(), appearanceMaterial.getTransparency());
   }

   public static Material createMaterialFromProperties(JMEAssetLocator contentMan, MutableColor diffuse, MutableColor ambient, MutableColor specular, double shininess,
         double transparancy)
   {
      Material material = new Material(contentMan.getAssetManager(), PHONG_ILLUMINATED_JME_MAT);

      double alpha = 1.0 - transparancy;

      material.setBoolean("UseMaterialColors", true);
      material.setColor("Diffuse", JMEDataTypeUtils.jMEColorRGBAFromVecMathColor3f(diffuse, alpha));
      material.setColor("Ambient", JMEDataTypeUtils.jMEColorRGBAFromVecMathColor3f(ambient, alpha));
      material.setColor("Specular", JMEDataTypeUtils.jMEColorRGBAFromVecMathColor3f(specular, alpha));
      material.setFloat("Shininess", (float) shininess);

      if (alpha < 0.99)
      {
         material.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
         // Turning off culling, because transparency allows to see through stuff.
      }
      material.getAdditionalRenderState().setFaceCullMode(FaceCullMode.Off);

      return material;
   }

   public static Material createMaterial(JMEAssetLocator assetManager, AppearanceDefinition appearanceDefinition)
   {
      if (appearanceDefinition instanceof YoAppearanceMaterial)
      {
         return createMaterialFromYoAppearanceMaterial(assetManager, (YoAppearanceMaterial) appearanceDefinition);
      }

      else if (appearanceDefinition instanceof YoAppearanceRGBColor)
      {
         return createMaterialFromYoAppearanceRGBColor(assetManager, (YoAppearanceRGBColor) appearanceDefinition);
      }

      else if (appearanceDefinition instanceof YoAppearanceTexture)
      {
         return createMaterialFromYoAppearanceTexture(assetManager, (YoAppearanceTexture) appearanceDefinition);
      }

      else if (appearanceDefinition instanceof SDFAppearance)
      {
         return createMaterialFromSDFAppearance(assetManager, (SDFAppearance) appearanceDefinition);
      }
      else if (appearanceDefinition instanceof HeightBasedTerrainBlend)
      {
         return createMaterialFromHeightBasedTerrainBlend(assetManager, (HeightBasedTerrainBlend) appearanceDefinition);
      }

      throw new RuntimeException("Appearance not implemented");
   }

}
