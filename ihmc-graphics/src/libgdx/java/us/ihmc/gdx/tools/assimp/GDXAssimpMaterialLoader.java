package us.ihmc.gdx.tools.assimp;

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

import java.nio.file.Path;
import java.nio.file.Paths;

public class GDXAssimpMaterialLoader
{
   private final AIMaterial assimpMaterial;
   private final String basePath;

   public GDXAssimpMaterialLoader(AIMaterial assimpMaterial, String basePath)
   {
      this.assimpMaterial = assimpMaterial;
      this.basePath = basePath;
   }

   public ModelMaterial load()
   {
      ModelMaterial modelMaterial = new ModelMaterial();

      int numberAllocated = assimpMaterial.mNumAllocated();
      LogTools.info("Number allocated: {}", numberAllocated);
      int numberOfMaterialProperties = assimpMaterial.mNumProperties();
      LogTools.info("Number of material properties: {}", numberOfMaterialProperties);

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
            modelMaterial.diffuse.r = 0.588235f;
            modelMaterial.diffuse.g = 0.588235f;
            modelMaterial.diffuse.b = 0.588235f;
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
//            LogTools.info(stringFromMemUTF8);
//            AIString aiString = AIString.create();
//            Assimp.aiGetMaterialString(assimpMaterial, materialPropertyKey, assimpMaterialProperty.mType(), textureFileNameIndex, aiString);
//            String textureFilePath = aiString.dataString().trim();
            String rawTextureFileName = MemoryUtil.memUTF8(assimpMaterialProperty.mData()).trim();

            // TODO: Where are these codes defined?  Maybe the order they come in
            boolean isDiffuse = rawTextureFileName.charAt(0) == '7';
            boolean isAmbient = rawTextureFileName.charAt(0) == '-';

            String filtered = rawTextureFileName.charAt(0) == '.' ? rawTextureFileName : rawTextureFileName.substring(1);
            String stringFromMemUTF8 = filtered.trim();
            String textureFilePath = ResourceTools.sanitizeResourcePath(stringFromMemUTF8);
//            if (textureFilePath.startsWith("/"))
//               textureFilePath = textureFilePath.substring(1);
            //            textureFilePath = Paths.get(basePath).resolve("extremities_diffuse_unplugged.jpg").normalize().toString();

            if (modelMaterial.textures == null)
               modelMaterial.textures = new Array<>();

            ModelTexture modelTexture = new ModelTexture();
            modelTexture.fileName = textureFilePath;
            modelTexture.id = "texture" + modelMaterial.textures.size;
            if (isDiffuse)
               modelTexture.usage = ModelTexture.USAGE_DIFFUSE;
            else if (isAmbient)
               modelTexture.usage = ModelTexture.USAGE_AMBIENT;
            else
               LogTools.error("Implement this usage! {}", rawTextureFileName.charAt(0));
            modelMaterial.textures.add(modelTexture);
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
            String stringFromMemUTF8 = MemoryUtil.memUTF8(assimpMaterialProperty.mData()).substring(1).trim(); // FIXME: Why substring 1 hack required?
//            AIString aiString = AIString.create();
//            Assimp.aiGetMaterialString(assimpMaterial, materialPropertyKey, assimpMaterialProperty.mType(), assimpMaterialProperty.mIndex(), aiString);
//            String stringFromGetMaterialString = aiString.dataString().trim();
//            if (!stringFromGetMaterialString.isEmpty())
//            {
//               valueAsString = stringFromGetMaterialString;
//            }
//            else
//            {
               valueAsString = stringFromMemUTF8;
//            }
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
         LogTools.info("Property: {}: {}", materialPropertyKey, valueAsString);
      }

      return modelMaterial;
   }

   private Color loadColor(AIMaterialProperty assimpMaterialProperty)
   {
      int valueType = assimpMaterialProperty.mType();
      if (valueType != Assimp.aiPTI_Float)
         LogTools.error("Implement other types!");
      if (assimpMaterialProperty.mDataLength() != 3 * Float.BYTES
          && assimpMaterialProperty.mDataLength() != 4 * Float.BYTES)
         LogTools.error("Implement other lengths!");

      Color color = new Color();
      color.r = assimpMaterialProperty.mData().getFloat(0);
      color.g = assimpMaterialProperty.mData().getFloat(1);
      color.b = assimpMaterialProperty.mData().getFloat(2);
      if (assimpMaterialProperty.mDataLength() == 4 * Float.BYTES)
         color.a = assimpMaterialProperty.mData().getFloat(3);
      else
         color.a = 1.0f;
      return color;
   }
}
