package us.ihmc.rdx.tools.assimp;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.model.data.ModelMaterial;
import com.badlogic.gdx.graphics.g3d.model.data.ModelTexture;
import com.badlogic.gdx.utils.Array;
import org.lwjgl.assimp.AIMaterial;
import org.lwjgl.assimp.AIMaterialProperty;
import org.lwjgl.assimp.AIString;
import org.lwjgl.assimp.Assimp;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.resources.ResourceTools;

import java.nio.file.Paths;

public class RDXAssimpMaterialLoader
{
   private final AIMaterial assimpMaterial;
   private final String basePath;
   private ModelMaterial modelMaterial;

   public RDXAssimpMaterialLoader(AIMaterial assimpMaterial, String basePath)
   {
      this.assimpMaterial = assimpMaterial;
      this.basePath = basePath;
   }

   public ModelMaterial load()
   {
      modelMaterial = new ModelMaterial();

      int numberAllocated = assimpMaterial.mNumAllocated();
      LogTools.debug("Number allocated: {}", numberAllocated);
      int numberOfMaterialProperties = assimpMaterial.mNumProperties();
      LogTools.debug("Number of material properties: {}", numberOfMaterialProperties);

      loadTextureInformation();

      for (int j = 0; j < numberOfMaterialProperties; j++)
      {
         AIMaterialProperty assimpMaterialProperty = new AIMaterialProperty(MemoryUtil.memByteBuffer(assimpMaterial.mProperties().get(j),
                                                                                                     AIMaterialProperty.SIZEOF));
         String materialPropertyKey = assimpMaterialProperty.mKey().dataString();
         int valueType = assimpMaterialProperty.mType();

         if (materialPropertyKey.equals(Assimp.AI_MATKEY_NAME))
         {
            modelMaterial.id = MemoryUtil.memUTF8(assimpMaterialProperty.mData()).trim();
         }
         else if (materialPropertyKey.equals(Assimp.AI_MATKEY_COLOR_DIFFUSE))
         {
            modelMaterial.diffuse = loadColor(assimpMaterialProperty);

            float desiredMax = 0.588235f; // This to get lighting looking good in engine
            float max = Math.max(Math.max(modelMaterial.diffuse.r, modelMaterial.diffuse.g), modelMaterial.diffuse.b);
            float ratio = desiredMax / max;

            modelMaterial.diffuse.r = ratio * modelMaterial.diffuse.r;
            modelMaterial.diffuse.g = ratio * modelMaterial.diffuse.g;
            modelMaterial.diffuse.b = ratio * modelMaterial.diffuse.b;
            modelMaterial.diffuse.a = 1.0f;
         }
         else if (materialPropertyKey.equals(Assimp.AI_MATKEY_COLOR_AMBIENT))
         {
            modelMaterial.ambient = loadColor(assimpMaterialProperty);
            modelMaterial.ambient.r = 0.050876f;
            modelMaterial.ambient.g = 0.050876f;
            modelMaterial.ambient.b = 0.050876f;
            modelMaterial.ambient.a = 1.0f;
         }
         else if (materialPropertyKey.equals(Assimp.AI_MATKEY_COLOR_EMISSIVE))
         {
            modelMaterial.emissive = loadColor(assimpMaterialProperty);
            modelMaterial.emissive.r = 0.0f;
            modelMaterial.emissive.g = 0.0f;
            modelMaterial.emissive.b = 0.0f;
            modelMaterial.emissive.a = 1.0f;
         }
         else if (materialPropertyKey.equals(Assimp.AI_MATKEY_COLOR_SPECULAR))
         {
            modelMaterial.specular = loadColor(assimpMaterialProperty);
            modelMaterial.specular.r = 0.8f;
            modelMaterial.specular.g = 0.8f;
            modelMaterial.specular.b = 0.8f;
            modelMaterial.specular.a = 1.0f;
         }
         else if (materialPropertyKey.equals(Assimp.AI_MATKEY_COLOR_REFLECTIVE))
         {
            modelMaterial.reflection = loadColor(assimpMaterialProperty);
         }
         else if (materialPropertyKey.equals(Assimp.AI_MATKEY_SHININESS))
         {
            modelMaterial.shininess = assimpMaterialProperty.mData().getFloat();
            modelMaterial.shininess = 1.111111f;
         }
         else if (materialPropertyKey.equals(Assimp.AI_MATKEY_OPACITY))
         {
            modelMaterial.opacity = assimpMaterialProperty.mData().getFloat();
         }
         else if (materialPropertyKey.equals(Assimp._AI_MATKEY_TEXTURE_BASE))
         {
            // Loaded separately
         }
         else if (materialPropertyKey.equals(Assimp._AI_MATKEY_UVWSRC_BASE))
         {
            ModelTexture modelTexture = modelMaterial.textures.get(modelMaterial.textures.size - 1);
            int uvwsrc = assimpMaterialProperty.mData().getInt();
            // TODO: What to do with this?
         }

         String valueAsString = "";
         if (valueType == Assimp.aiPTI_Float)
         {
            int dataLength = assimpMaterialProperty.mDataLength() / Float.BYTES;
            for (int i = 0; i < dataLength; i++)
            {
               float floatValue = assimpMaterialProperty.mData().getFloat(i);
               valueAsString += floatValue + " ";
            }
         }
         else if (valueType == Assimp.aiPTI_Double)
         {
            int dataLength = assimpMaterialProperty.mDataLength() / Double.BYTES;
            for (int i = 0; i < dataLength; i++)
            {
               double doubleValue = assimpMaterialProperty.mData().getDouble(i);
               valueAsString += doubleValue + " ";
            }
         }
         else if (valueType == Assimp.aiPTI_String)
         {
            String stringFromMemUTF8 = MemoryUtil.memUTF8(assimpMaterialProperty.mData()).trim();
            valueAsString = stringFromMemUTF8;
         }
         else if (valueType == Assimp.aiPTI_Integer)
         {
            int dataLength = assimpMaterialProperty.mDataLength() / Integer.BYTES;
            for (int i = 0; i < dataLength; i++)
            {
               int intValue = assimpMaterialProperty.mData().getInt(i);
               valueAsString += intValue + " ";
            }
         }
         else if (valueType == Assimp.aiPTI_Buffer)
         {
            valueAsString += " buffer of length " + assimpMaterialProperty.mDataLength();
         }
         LogTools.debug("Property: {}: {}", materialPropertyKey, valueAsString);
      }

      return modelMaterial;
   }

   private void loadTextureInformation()
   {
      loadTextureInformation(Assimp.aiTextureType_NONE);
      loadTextureInformation(Assimp.aiTextureType_DIFFUSE);
      loadTextureInformation(Assimp.aiTextureType_SPECULAR);
      loadTextureInformation(Assimp.aiTextureType_AMBIENT);
      loadTextureInformation(Assimp.aiTextureType_EMISSIVE);
      loadTextureInformation(Assimp.aiTextureType_HEIGHT);
      loadTextureInformation(Assimp.aiTextureType_NORMALS);
      loadTextureInformation(Assimp.aiTextureType_SHININESS);
      loadTextureInformation(Assimp.aiTextureType_OPACITY);
      loadTextureInformation(Assimp.aiTextureType_DISPLACEMENT);
      loadTextureInformation(Assimp.aiTextureType_LIGHTMAP);
      loadTextureInformation(Assimp.aiTextureType_REFLECTION);
      loadTextureInformation(Assimp.aiTextureType_UNKNOWN);

   }

   private void loadTextureInformation(int textureType)
   {
      int numberOfMaterialTexturesOfType = Assimp.aiGetMaterialTextureCount(assimpMaterial, textureType);

      if (numberOfMaterialTexturesOfType > 0)
      {
         LogTools.debug("Number of {} textures: {}", getTextureTypeString(textureType), numberOfMaterialTexturesOfType);

         if (modelMaterial.textures == null)
            modelMaterial.textures = new Array<>();

         for (int i = 0; i < numberOfMaterialTexturesOfType; i++)
         {
            AIString path = AIString.create();
            int[] mapping = new int[1];
            int[] uvSourceChannelIndex = new int[1];
            float[] blendFactor = new float[1];
            int[] blendWithPreviousTextureOperation = new int[1];
            int[] mapmode = null;
            int[] flags = new int[1];
            Assimp.aiGetMaterialTexture(assimpMaterial,
                                        textureType,
                                        i,
                                        path,
                                        mapping,
                                        uvSourceChannelIndex,
                                        blendFactor,
                                        blendWithPreviousTextureOperation,
                                        mapmode,
                                        flags);
            String textureFilePath = ResourceTools.sanitizeResourcePath(Paths.get(basePath).resolve(path.dataString()).toString());
            LogTools.debug("Path: {}", textureFilePath);
            LogTools.debug("Mapping: {}", mapping[0]);
            LogTools.debug("UV source channel index: {}", uvSourceChannelIndex[0]);
            LogTools.debug("Blend factor: {}", blendFactor[0]);
            LogTools.debug("Blend with previous texture operation: {}", blendWithPreviousTextureOperation[0]);
            LogTools.debug("Flags: {}", flags[0]);

            ModelTexture modelTexture = new ModelTexture();
            modelTexture.fileName = textureFilePath;
            modelTexture.id = "texture" + modelMaterial.textures.size;
            modelTexture.usage = getModelTextureTypeCode(textureType);
            modelMaterial.textures.add(modelTexture);
         }
      }
   }

   private String getTextureTypeString(int textureType)
   {
      switch (textureType)
      {
         case Assimp.aiTextureType_NONE:
            return "None";
         case Assimp.aiTextureType_DIFFUSE:
            return "Diffuse";
         case Assimp.aiTextureType_SPECULAR:
            return "Specular";
         case Assimp.aiTextureType_AMBIENT:
            return "Ambient";
         case Assimp.aiTextureType_EMISSIVE:
            return "Emissive";
         case Assimp.aiTextureType_HEIGHT:
            return "Height";
         case Assimp.aiTextureType_NORMALS:
            return "Normals";
         case Assimp.aiTextureType_SHININESS:
            return "Shininess";
         case Assimp.aiTextureType_OPACITY:
            return "Opacity";
         case Assimp.aiTextureType_DISPLACEMENT:
            return "Displacement";
         case Assimp.aiTextureType_LIGHTMAP:
            return "Lightmap";
         case Assimp.aiTextureType_REFLECTION:
            return "Reflection";
         case Assimp.aiTextureType_UNKNOWN:
            return "Unknown";
      }
      return "NotAType";
   }

   private int getModelTextureTypeCode(int textureType)
   {
      switch (textureType)
      {
         case Assimp.aiTextureType_NONE:
            return ModelTexture.USAGE_NONE;
         case Assimp.aiTextureType_DIFFUSE:
            return ModelTexture.USAGE_DIFFUSE;
         case Assimp.aiTextureType_SPECULAR:
            return ModelTexture.USAGE_SPECULAR;
         case Assimp.aiTextureType_AMBIENT:
            return ModelTexture.USAGE_AMBIENT;
         case Assimp.aiTextureType_EMISSIVE:
            return ModelTexture.USAGE_EMISSIVE;
         case Assimp.aiTextureType_HEIGHT:
            return ModelTexture.USAGE_BUMP; // TODO: Does HEIGHT == BUMP?
         case Assimp.aiTextureType_NORMALS:
            return ModelTexture.USAGE_NORMAL;
         case Assimp.aiTextureType_SHININESS:
            return ModelTexture.USAGE_SHININESS;
         case Assimp.aiTextureType_OPACITY:
            return ModelTexture.USAGE_TRANSPARENCY;
         case Assimp.aiTextureType_DISPLACEMENT:
            return -1; // FIXME: Not supported by libGDX
         case Assimp.aiTextureType_LIGHTMAP:
            return -1; // FIXME: Not supported by libGDX
         case Assimp.aiTextureType_REFLECTION:
            return ModelTexture.USAGE_REFLECTION;
         case Assimp.aiTextureType_UNKNOWN:
            return ModelTexture.USAGE_UNKNOWN;
      }
      return -1;
   }

   private Color loadColor(AIMaterialProperty assimpMaterialProperty)
   {
      int valueType = assimpMaterialProperty.mType();

      if (valueType != Assimp.aiPTI_Float)
         LogTools.error("Implement other types!");

      int materialPropertyDataLength = assimpMaterialProperty.mDataLength();
      boolean isThreeFloats = materialPropertyDataLength == 3 * Float.BYTES;
      boolean isFourFloats = materialPropertyDataLength == 4 * Float.BYTES;

      Color color = new Color(0.5f, 0.5f, 0.5f, 1.0f);
      if (isThreeFloats || isFourFloats)
      {
         color.r = assimpMaterialProperty.mData().getFloat();
         color.g = assimpMaterialProperty.mData().getFloat(Float.BYTES);
         color.b = assimpMaterialProperty.mData().getFloat(2 * Float.BYTES);
         if (isFourFloats)
            color.a = assimpMaterialProperty.mData().getFloat(3 * Float.BYTES);
         else
            color.a = 1.0f;
      }
      else
      {
         LogTools.error("What to do with {} float(s)?", materialPropertyDataLength / Float.BYTES);
      }
      return color;
   }

   public AIMaterial getAssimpMaterial()
   {
      return assimpMaterial;
   }

   public ModelMaterial getModelMaterial()
   {
      return modelMaterial;
   }
}
