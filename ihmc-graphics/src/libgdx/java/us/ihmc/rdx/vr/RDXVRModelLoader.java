package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.graphics.g3d.model.NodePart;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.ObjectMap;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opengl.GL41;
import org.lwjgl.openvr.*;

public class RDXVRModelLoader
{
   private static final ObjectMap<String, Model> models = new ObjectMap<>();

   public static Model loadRenderModel(String name)
   {
      if (models.containsKey(name))
         return models.get(name);

      // FIXME we load the models synchronously cause we are lazy
      int error = 0;
      PointerBuffer modelPointer = PointerBuffer.allocateDirect(1);
      do
      {
         error = VRRenderModels.VRRenderModels_LoadRenderModel_Async(name, modelPointer);
      }
      while (error == VR.EVRRenderModelError_VRRenderModelError_Loading);

      if (error != VR.EVRRenderModelError_VRRenderModelError_None)
         return null;
      RenderModel renderModel = new RenderModel(modelPointer.getByteBuffer(RenderModel.SIZEOF));

      error = 0;
      PointerBuffer texturePointer = PointerBuffer.allocateDirect(1);
      do
      {
         error = VRRenderModels.VRRenderModels_LoadTexture_Async(renderModel.diffuseTextureId(), texturePointer);
      }
      while (error == VR.EVRRenderModelError_VRRenderModelError_Loading);

      if (error != VR.EVRRenderModelError_VRRenderModelError_None)
      {
         VRRenderModels.VRRenderModels_FreeRenderModel(renderModel);
         return null;
      }

      RenderModelTextureMap renderModelTexture = new RenderModelTextureMap(texturePointer.getByteBuffer(RenderModelTextureMap.SIZEOF));

      // convert to a Model
      Mesh mesh = new Mesh(true,
                           renderModel.unVertexCount(),
                           renderModel.unTriangleCount() * 3,
                           VertexAttribute.Position(),
                           VertexAttribute.Normal(),
                           VertexAttribute.TexCoords(0));
      MeshPart meshPart = new MeshPart(name, mesh, 0, renderModel.unTriangleCount() * 3, GL41.GL_TRIANGLES);
      RenderModelVertex.Buffer vertices = renderModel.rVertexData();
      float[] packedVertices = new float[8 * renderModel.unVertexCount()];
      int i = 0;
      while (vertices.remaining() > 0)
      {
         RenderModelVertex v = vertices.get();
         packedVertices[i++] = v.vPosition().v(0);
         packedVertices[i++] = v.vPosition().v(1);
         packedVertices[i++] = v.vPosition().v(2);

         packedVertices[i++] = v.vNormal().v(0);
         packedVertices[i++] = v.vNormal().v(1);
         packedVertices[i++] = v.vNormal().v(2);

         packedVertices[i++] = v.rfTextureCoord().get(0);
         packedVertices[i++] = v.rfTextureCoord().get(1);
      }
      mesh.setVertices(packedVertices);
      short[] indices = new short[renderModel.unTriangleCount() * 3];
      renderModel.IndexData().get(indices);
      mesh.setIndices(indices);

      Pixmap pixmap = new Pixmap(renderModelTexture.unWidth(), renderModelTexture.unHeight(), Pixmap.Format.RGBA8888);
      byte[] pixels = new byte[renderModelTexture.unWidth() * renderModelTexture.unHeight() * 4];
      renderModelTexture.rubTextureMapData(pixels.length).get(pixels);
      pixmap.getPixels().put(pixels);
      pixmap.getPixels().position(0);
      com.badlogic.gdx.graphics.Texture texture = new Texture(new PixmapTextureData(pixmap, pixmap.getFormat(), true, true));
      Material material = new Material(new TextureAttribute(TextureAttribute.Diffuse, texture));

      Model model = new Model();
      model.meshes.add(mesh);
      model.meshParts.add(meshPart);
      model.materials.add(material);
      Node node = new Node();
      node.id = name;
      node.parts.add(new NodePart(meshPart, material));
      model.nodes.add(node);
      model.manageDisposable(mesh);
      model.manageDisposable(texture);

      VRRenderModels.VRRenderModels_FreeRenderModel(renderModel);
      VRRenderModels.VRRenderModels_FreeTexture(renderModelTexture);

      models.put(name, model);

      return model;
   }
}
