package us.ihmc.rdx.simulation.environment.isaac;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.PixmapIO;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttribute;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Attribute;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRFloatAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.opengl.GL11;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Writes a USD ASCII (.usda) stage: prims, transforms, triangle meshes read back from libGDX
 * renderables, and UsdPreviewSurface materials. Textures are read back from the GPU into
 * {@code textures/*.png} beside the stage, so any model RDX can draw (glb, g3dj, procedural) exports.
 */
public class RDXUSDAWriter
{
   public static final String LOOKS_PATH = "/World/Looks";

   private final StringBuilder out = new StringBuilder();
   private final Path directory;
   private int depth = 0;
   private final Map<Mesh, float[]> vertexCache = new IdentityHashMap<>();
   private final Map<Mesh, short[]> indexCache = new IdentityHashMap<>();
   private final Map<Texture, String> textureFiles = new IdentityHashMap<>();
   /** Material description to its prim name, so identical materials are written once. */
   private final Map<String, String> materials = new LinkedHashMap<>();
   private final Matrix4 relative = new Matrix4();
   private int meshCount = 0;

   public RDXUSDAWriter(Path directory)
   {
      this.directory = directory;
   }

   public void header(String comment)
   {
      out.append("#usda 1.0\n(\n");
      out.append("    defaultPrim = \"World\"\n");
      out.append("    doc = \"").append(comment.replace("\"", "'")).append("\"\n");
      out.append("    metersPerUnit = 1\n");
      out.append("    upAxis = \"Z\"\n");
      out.append(")\n\n");
   }

   public void beginPrim(String type, String name, String... apiSchemas)
   {
      StringBuilder line = new StringBuilder("def ").append(type).append(" \"").append(name).append('"');
      if (apiSchemas.length > 0)
      {
         line.append(" (\n");
         line.append(indent(depth + 1)).append("prepend apiSchemas = [");
         for (int i = 0; i < apiSchemas.length; i++)
            line.append(i == 0 ? "" : ", ").append('"').append(apiSchemas[i]).append('"');
         line.append("]\n").append(indent(depth)).append(')');
      }
      line(line.toString());
      line("{");
      depth++;
   }

   public void endPrim()
   {
      depth--;
      line("}");
   }

   public void line(String text)
   {
      out.append(indent(depth)).append(text).append('\n');
   }

   public void attribute(String declaration, String value)
   {
      line(declaration + " = " + value);
   }

   public void relationship(String name, String... targets)
   {
      if (targets.length == 1)
      {
         line("rel " + name + " = <" + targets[0] + ">");
         return;
      }
      StringBuilder value = new StringBuilder("[");
      for (int i = 0; i < targets.length; i++)
         value.append(i == 0 ? "" : ", ").append('<').append(targets[i]).append('>');
      line("rel " + name + " = " + value.append(']'));
   }

   /** Translate then orient, which PhysX reads for rigid bodies without decomposing a matrix. */
   public void transform(RigidBodyTransformReadOnly transform)
   {
      attribute("double3 xformOp:translate", tuple(transform.getTranslation()));
      attribute("quatd xformOp:orient", quaternion(new Quaternion(transform.getRotation())));
      attribute("uniform token[] xformOpOrder", "[\"xformOp:translate\", \"xformOp:orient\"]");
   }

   public void transformWithScale(RigidBodyTransformReadOnly transform, double scaleX, double scaleY, double scaleZ)
   {
      attribute("double3 xformOp:translate", tuple(transform.getTranslation()));
      attribute("quatd xformOp:orient", quaternion(new Quaternion(transform.getRotation())));
      attribute("double3 xformOp:scale", "(" + fmt(scaleX) + ", " + fmt(scaleY) + ", " + fmt(scaleZ) + ")");
      attribute("uniform token[] xformOpOrder", "[\"xformOp:translate\", \"xformOp:orient\", \"xformOp:scale\"]");
   }

   /** libGDX is column-major with column vectors. USD's row-vector rows are the same 16 floats in order. */
   private void matrix(Matrix4 matrix)
   {
      float[] v = matrix.val;
      StringBuilder value = new StringBuilder("(");
      for (int row = 0; row < 4; row++)
      {
         value.append(row == 0 ? "(" : ", (");
         for (int column = 0; column < 4; column++)
            value.append(column == 0 ? "" : ", ").append(fmt(v[row * 4 + column]));
         value.append(')');
      }
      attribute("matrix4d xformOp:transform", value.append(')').toString());
      attribute("uniform token[] xformOpOrder", "[\"xformOp:transform\"]");
   }

   /**
    * One Mesh prim per triangle part. Each renderable's world transform is written relative to
    * {@code parentToWorld}, the world transform of the prim these meshes are written under.
    *
    * @param collision true writes convex-hull colliders that are not rendered, with no material.
    */
   public int meshes(Array<Renderable> renderables, Matrix4 parentToWorld, String prefix, boolean collision)
   {
      Matrix4 worldToParent = new Matrix4(parentToWorld).inv();
      int written = 0;
      for (Renderable renderable : renderables)
      {
         relative.set(worldToParent).mul(renderable.worldTransform);
         if (mesh(prefix + "_" + written, renderable.meshPart, renderable.material, relative, collision))
            written++;
      }
      return written;
   }

   private boolean mesh(String name, MeshPart part, Material material, Matrix4 transform, boolean collision)
   {
      if (part == null || part.mesh == null || part.primitiveType != GL20.GL_TRIANGLES || part.size < 3)
         return false;
      Mesh mesh = part.mesh;
      VertexAttributes attributes = mesh.getVertexAttributes();
      int stride = mesh.getVertexSize() / 4;
      VertexAttribute position = attributes.findByUsage(VertexAttributes.Usage.Position);
      if (position == null)
         return false;
      VertexAttribute normal = attributes.findByUsage(VertexAttributes.Usage.Normal);
      VertexAttribute uv = null;
      for (VertexAttribute attribute : attributes)
      {
         if (attribute.usage == VertexAttributes.Usage.TextureCoordinates && (uv == null || attribute.unit == 0))
            uv = attribute;
      }
      VertexAttribute color = attributes.findByUsage(VertexAttributes.Usage.ColorUnpacked);
      VertexAttribute packedColor = attributes.findByUsage(VertexAttributes.Usage.ColorPacked);

      float[] vertices = vertexCache.computeIfAbsent(mesh, m ->
      {
         float[] array = new float[m.getNumVertices() * stride];
         m.getVertices(array);
         return array;
      });
      short[] indices = indexCache.computeIfAbsent(mesh, m ->
      {
         short[] array = new short[m.getNumIndices()];
         if (array.length > 0)
            m.getIndices(array);
         return array;
      });

      int corners = part.size - part.size % 3;
      int[] remap = new int[mesh.getNumVertices()];
      Arrays.fill(remap, -1);
      int[] used = new int[corners];
      int usedCount = 0;
      int[] faceIndices = new int[corners];
      for (int k = 0; k < corners; k++)
      {
         int vertex = indices.length > 0 ? indices[part.offset + k] & 0xFFFF : part.offset + k;
         if (vertex >= remap.length)
            return false;
         if (remap[vertex] < 0)
         {
            remap[vertex] = usedCount;
            used[usedCount++] = vertex;
         }
         faceIndices[k] = remap[vertex];
      }

      TextureAttribute texture = textureAttribute(material);
      boolean vertexColors = !collision && texture == null && (color != null || packedColor != null);
      List<String> schemas = new ArrayList<>();
      if (collision)
      {
         schemas.add("PhysicsCollisionAPI");
         schemas.add("PhysicsMeshCollisionAPI");
      }
      else if (!vertexColors && material != null)
      {
         schemas.add("MaterialBindingAPI");
      }
      beginPrim("Mesh", name, schemas.toArray(new String[0]));
      meshCount++;
      matrix(transform);
      StringBuilder counts = new StringBuilder("[");
      for (int f = 0; f < corners / 3; f++)
         counts.append(f == 0 ? "3" : ", 3");
      attribute("int[] faceVertexCounts", counts.append(']').toString());
      StringBuilder faces = new StringBuilder("[");
      for (int k = 0; k < corners; k++)
         faces.append(k == 0 ? "" : ", ").append(faceIndices[k]);
      attribute("int[] faceVertexIndices", faces.append(']').toString());
      attribute("point3f[] points", vectors(vertices, used, usedCount, stride, position.offset / 4, 3, null));
      attribute("uniform token subdivisionScheme", "\"none\"");
      if (collision)
      {
         attribute("uniform token physics:approximation", "\"convexHull\"");
         attribute("uniform token purpose", "\"guide\"");
         endPrim();
         return true;
      }
      if (normal != null)
      {
         line("normal3f[] normals = " + vectors(vertices, used, usedCount, stride, normal.offset / 4, 3, null) + " (");
         line("    interpolation = \"vertex\"");
         line(")");
      }
      if (uv != null && texture != null)
      {
         float[] uvTransform = {texture.offsetU, texture.offsetV, texture.scaleU, texture.scaleV};
         line("texCoord2f[] primvars:st = " + vectors(vertices, used, usedCount, stride, uv.offset / 4, 2, uvTransform) + " (");
         line("    interpolation = \"vertex\"");
         line(")");
      }
      if (vertexColors)
      {
         Color base = baseColor(material);
         StringBuilder colors = new StringBuilder("[");
         Color vertexColor = new Color();
         for (int i = 0; i < usedCount; i++)
         {
            int start = used[i] * stride;
            if (color != null)
               vertexColor.set(vertices[start + color.offset / 4], vertices[start + color.offset / 4 + 1], vertices[start + color.offset / 4 + 2], 1.0f);
            else
               Color.abgr8888ToColor(vertexColor, vertices[start + packedColor.offset / 4]);
            colors.append(i == 0 ? "(" : ", (").append(fmt(vertexColor.r * base.r)).append(", ").append(fmt(vertexColor.g * base.g))
                  .append(", ").append(fmt(vertexColor.b * base.b)).append(')');
         }
         line("color3f[] primvars:displayColor = " + colors.append(']') + " (");
         line("    interpolation = \"vertex\"");
         line(")");
      }
      else if (material != null)
      {
         relationship("material:binding", LOOKS_PATH + "/" + material(material, texture));
      }
      endPrim();
      return true;
   }

   /** {@code uvTransform} is offset U, offset V, scale U, scale V. USD's t runs up the image, libGDX's v runs down. */
   private static String vectors(float[] vertices, int[] used, int count, int stride, int offset, int size, float[] uvTransform)
   {
      StringBuilder value = new StringBuilder(count * size * 10).append('[');
      for (int i = 0; i < count; i++)
      {
         int start = used[i] * stride + offset;
         value.append(i == 0 ? "(" : ", (");
         if (uvTransform != null)
         {
            float u = uvTransform[0] + vertices[start] * uvTransform[2];
            float v = uvTransform[1] + vertices[start + 1] * uvTransform[3];
            value.append(fmt(u)).append(", ").append(fmt(1.0f - v));
         }
         else
         {
            for (int c = 0; c < size; c++)
               value.append(c == 0 ? "" : ", ").append(fmt(vertices[start + c]));
         }
         value.append(')');
      }
      return value.append(']').toString();
   }

   private static TextureAttribute textureAttribute(Material material)
   {
      if (material == null)
         return null;
      Attribute attribute = material.get(PBRTextureAttribute.BaseColorTexture);
      if (attribute == null)
         attribute = material.get(TextureAttribute.Diffuse);
      if (!(attribute instanceof TextureAttribute texture) || texture.textureDescription.texture == null)
         return null;
      return texture;
   }

   private static Color baseColor(Material material)
   {
      Color color = new Color(Color.WHITE);
      if (material == null)
         return color;
      Attribute attribute = material.get(PBRColorAttribute.BaseColorFactor);
      if (attribute == null)
         attribute = material.get(ColorAttribute.Diffuse);
      if (attribute instanceof ColorAttribute colorAttribute)
         color.set(colorAttribute.color);
      Attribute blending = material.get(BlendingAttribute.Type);
      if (blending instanceof BlendingAttribute blendingAttribute && blendingAttribute.blended)
         color.a *= blendingAttribute.opacity;
      return color;
   }

   private static float floatAttribute(Material material, long type, float fallback)
   {
      Attribute attribute = material.get(type);
      return attribute instanceof PBRFloatAttribute value ? value.value : fallback;
   }

   /** Returns the material's prim name, writing nothing yet: {@link #looks()} writes them all. */
   private String material(Material material, TextureAttribute texture)
   {
      Color color = baseColor(material);
      String textureFile = texture == null ? null : textureFile(texture.textureDescription.texture);
      float metallic = floatAttribute(material, PBRFloatAttribute.Metallic, 0.0f);
      float roughness = floatAttribute(material, PBRFloatAttribute.Roughness, 0.6f);
      String key = String.join("|", fmt(color.r), fmt(color.g), fmt(color.b), fmt(color.a), String.valueOf(textureFile), fmt(metallic), fmt(roughness));
      return materials.computeIfAbsent(key, k -> "Material_" + materials.size());
   }

   private String textureFile(Texture texture)
   {
      return textureFiles.computeIfAbsent(texture, t ->
      {
         String fileName = "textures/texture_" + textureFiles.size() + ".png";
         int width = t.getWidth();
         int height = t.getHeight();
         Pixmap pixmap = new Pixmap(width, height, Pixmap.Format.RGBA8888);
         try
         {
            Files.createDirectories(directory.resolve("textures"));
            t.bind();
            Gdx.gl.glPixelStorei(GL20.GL_PACK_ALIGNMENT, 1);
            ByteBuffer pixels = pixmap.getPixels();
            pixels.position(0);
            // Row 0 on the GPU is the first row uploaded, the top of the image, as in the PNG.
            GL11.glGetTexImage(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, pixels);
            PixmapIO.writePNG(new FileHandle(directory.resolve(fileName).toFile()), pixmap);
            return fileName;
         }
         catch (IOException | RuntimeException exception)
         {
            LogTools.warn("Could not export a {}x{} texture: {}", width, height, exception.getMessage());
            return null;
         }
         finally
         {
            pixmap.dispose();
         }
      });
   }

   /** Writes every material the meshes bound, under {@link #LOOKS_PATH}. Call once, inside /World. */
   public void looks()
   {
      beginPrim("Scope", "Looks");
      for (Map.Entry<String, String> entry : materials.entrySet())
      {
         String[] fields = entry.getKey().split("\\|", -1);
         String name = entry.getValue();
         String path = LOOKS_PATH + "/" + name;
         String textureFile = "null".equals(fields[4]) ? null : fields[4];
         beginPrim("Material", name);
         attribute("token outputs:surface.connect", "<" + path + "/Surface.outputs:surface>");
         beginPrim("Shader", "Surface");
         attribute("uniform token info:id", "\"UsdPreviewSurface\"");
         if (textureFile == null)
            attribute("color3f inputs:diffuseColor", "(" + fields[0] + ", " + fields[1] + ", " + fields[2] + ")");
         else
            attribute("color3f inputs:diffuseColor.connect", "<" + path + "/BaseColor.outputs:rgb>");
         attribute("float inputs:opacity", fields[3]);
         attribute("float inputs:metallic", fields[5]);
         attribute("float inputs:roughness", fields[6]);
         line("token outputs:surface");
         endPrim();
         if (textureFile != null)
         {
            beginPrim("Shader", "BaseColor");
            attribute("uniform token info:id", "\"UsdUVTexture\"");
            attribute("asset inputs:file", "@./" + textureFile + "@");
            attribute("float2 inputs:st.connect", "<" + path + "/TexCoord.outputs:result>");
            attribute("token inputs:sourceColorSpace", "\"sRGB\"");
            attribute("token inputs:wrapS", "\"repeat\"");
            attribute("token inputs:wrapT", "\"repeat\"");
            attribute("float4 inputs:scale", "(" + fields[0] + ", " + fields[1] + ", " + fields[2] + ", 1)");
            line("float3 outputs:rgb");
            endPrim();
            beginPrim("Shader", "TexCoord");
            attribute("uniform token info:id", "\"UsdPrimvarReader_float2\"");
            attribute("string inputs:varname", "\"st\"");
            line("float2 outputs:result");
            endPrim();
         }
         endPrim();
      }
      endPrim();
   }

   public int getMeshCount()
   {
      return meshCount;
   }

   public int getTextureCount()
   {
      return (int) textureFiles.values().stream().filter(file -> file != null).count();
   }

   public void save(Path file) throws IOException
   {
      Files.writeString(file, out);
   }

   /** USD prim names are identifiers: letters, digits and underscores, not starting with a digit. */
   public static String primName(String raw)
   {
      String name = raw.replaceAll("[^A-Za-z0-9_]", "_");
      if (name.isEmpty() || Character.isDigit(name.charAt(0)))
         name = "_" + name;
      return name;
   }

   public static String tuple(Tuple3DReadOnly tuple)
   {
      return "(" + fmt(tuple.getX()) + ", " + fmt(tuple.getY()) + ", " + fmt(tuple.getZ()) + ")";
   }

   /** USD quaternions are real part first. */
   public static String quaternion(QuaternionReadOnly quaternion)
   {
      return "(" + fmt(quaternion.getS()) + ", " + fmt(quaternion.getX()) + ", " + fmt(quaternion.getY()) + ", " + fmt(quaternion.getZ()) + ")";
   }

   public static String fmt(double value)
   {
      if (!Double.isFinite(value))
         return value > 0 ? "inf" : value < 0 ? "-inf" : "0";
      if (value == Math.rint(value) && Math.abs(value) < 1.0e15)
         return Long.toString((long) value);
      return String.format(Locale.ROOT, "%.9g", value).replaceFirst("\\.?0+(e|$)", "$1");
   }

   public static String fmt(float value)
   {
      if (!Float.isFinite(value))
         return "0";
      if (value == Math.rint(value) && Math.abs(value) < 1.0e7f)
         return Integer.toString((int) value);
      return Float.toString(value);
   }

   private static String indent(int depth)
   {
      return "    ".repeat(depth);
   }
}
