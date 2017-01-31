package us.ihmc.jMonkeyEngineToolkit.tralala;


import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;

import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Format;
import com.jme3.scene.VertexBuffer.Type;
import com.jme3.scene.VertexBuffer.Usage;
import com.jme3.util.BufferUtils;

/**
 * Immutable class that transforms the mesh. It is immutable to minimize
 * get/set/construct array operations, and doesnt copy data unless it has to
 * modify it. Since it is immutable remember to use the returned value, because
 * the class internal state / method arguments dont change. <p/> <p/> <p/> <p/>
 * <p/> <p/> <p/> <p/> <p/>
 * <p/>
 * @toDo Implement "slice along mirror" for symmetry.
 * @toDo Support animation. Bug : Only half of the mesh has animation (the
 * symmetric part has no animation).
 * @toDo ability to apply Transformations to specific points (e.g not affecting
 * the whole mesh but a part of it).
 * @toDo select uvs, vertexes etc based on mouse click.
 */
public final class MeshData
{
   private float[] vertexArray;
   private float[] normalArray;
   private float[][] uvArrays;
   private short[] indexArray;
   private float[] boneWeightArray;
   private byte[] boneIndexArray;
   private int maxNumOfWeights;
   public static final int MAX_TEX_COORDS = 8;

   /** Creates an empty mesh */
   private MeshData()
   {
   }

   /** Creates a mesh without animation, and only 1 tex coord.*/
   public MeshData(float[] vertexArray, float[] normalArray, float[] uvArray, short[] indexArray)
   {
      this.vertexArray = vertexArray;
      this.normalArray = normalArray;
      this.uvArrays = new float[1][];
      this.uvArrays[0] = uvArray;
      this.indexArray = indexArray;
   }

   /** Creates a mesh with the specified values.*/
   public MeshData(float[] vertexArray, float[] normalArray, float[][] uvArrays, short[] indexArray, float[] boneWeightArray, byte[] boneIndexArray, int maxNumOfWeights)
   {
      this.maxNumOfWeights = maxNumOfWeights;
      this.vertexArray = vertexArray;
      this.normalArray = normalArray;
      this.uvArrays = new float[uvArrays.length][];
      for (int t = 0; t < uvArrays.length; t++)
      {
         this.uvArrays[t] = uvArrays[t];
      }
      this.indexArray = indexArray;
      this.boneWeightArray = boneWeightArray;
      this.boneIndexArray = boneIndexArray;
   }

   /** creates MeshData from mesh  */
   public MeshData(Mesh mesh)
   {
      this.maxNumOfWeights = mesh.getMaxNumWeights();
      vertexArray = BufferUtils.getFloatArray(mesh.getFloatBuffer(Type.Position));
      normalArray = BufferUtils.getFloatArray(mesh.getFloatBuffer(Type.Normal));

      int numOfTexcoordinates = 0;
      for (int i = 0; i < MAX_TEX_COORDS; i++)
      {
         if (mesh.getBuffer(Utilities.getTexCoordType(i)) != null) numOfTexcoordinates++;
      }

      this.uvArrays = new float[numOfTexcoordinates][];
      for (int i = 0; i < numOfTexcoordinates; i++)
      {
         uvArrays[i] = BufferUtils.getFloatArray(mesh.getFloatBuffer(Utilities.getTexCoordType(i)));
      }

      indexArray = Utilities.getShortArray(mesh.getShortBuffer(Type.Index));
      boneWeightArray = BufferUtils.getFloatArray(mesh.getFloatBuffer(Type.BoneWeight));
      VertexBuffer boneIndexBuffer = mesh.getBuffer(Type.BoneIndex);
      if (boneIndexBuffer != null) boneIndexArray = (Utilities.getByteArray((ByteBuffer) boneIndexBuffer.getData()));
   }

   /**Merges 2 meshes into 1. The second's mesh indexes change to point to their new position (after mesh1's array).
    * @param shareSkeleton if the resulting mesh will have the skeleton of mesh1, (or mesh2 if mesh1 skeleton is null).
    */
   public MeshData merge(MeshData mesh2)
   {
      MeshData mesh1 = this;

      MeshData resultMesh = new MeshData();
      resultMesh.vertexArray = Utilities.merge(mesh1.vertexArray, mesh2.vertexArray);
      resultMesh.normalArray = Utilities.merge(mesh1.normalArray, mesh2.normalArray);
      resultMesh.uvArrays = new float[mesh1.uvArrays.length][];
      for (int i = 0; i < mesh1.uvArrays.length; i++)
      {
         resultMesh.uvArrays[i] = Utilities.merge(mesh1.uvArrays[i], mesh2.uvArrays[i]);
      }

      //make second's mesh indexes point to their new position (after mesh1's array).*/
      resultMesh.indexArray = new short[mesh1.indexArray.length + mesh2.indexArray.length];
      System.arraycopy(mesh1.indexArray, 0, resultMesh.indexArray, 0, mesh1.indexArray.length);
      for (int i = 0; i < mesh2.indexArray.length; i++)
      {
         resultMesh.indexArray[mesh1.indexArray.length + i] = (short) (mesh2.indexArray[i] + mesh1.getNumberOfVertexElements());
      }

      resultMesh.boneWeightArray = mesh1.boneWeightArray;
      resultMesh.boneIndexArray = mesh1.boneIndexArray;

      return resultMesh;
   }

   /**
    * @param planeOrigin  the planeOrigin e.g (0,0,0)
    * @param planeNormal  the planeNormal is a vector that points upward from the plane e.g Vector3f.UNIT_Z
    */
   public MeshData mirror(Vector3f planeOrigin, Vector3f planeNormal)
   {
      MeshData mesh = this;
      MeshData resultMesh = mesh.shallowClone();
      resultMesh.vertexArray = new float[mesh.vertexArray.length];
      resultMesh.normalArray = new float[mesh.normalArray.length];

      //mirror positions.
      for (int i = 0; i < mesh.getNumberOfVertexElements(); i++)
      {
         Vector3f position = mesh.getVertex(i);
         Vector3f summetricPosition = Utilities.getSummetricPosition(position, planeOrigin, planeNormal);
         resultMesh.setVertex(summetricPosition, i);
      }

      //mirror normals.
      for (int i = 0; i < mesh.getNumberOfNormalElements(); i++)
      {
         Vector3f normalPosition = mesh.getNormal(i);
         Vector3f summetricNormalPosition = Utilities.getSummetricPosition(normalPosition, planeOrigin, planeNormal);
         resultMesh.setNormal(summetricNormalPosition, i);
      }

      resultMesh = resultMesh.flipIndexes();
      return resultMesh;
   }

   /** Applies symmetry modifier to mesh, and creates a new mesh from the 2 submeshes.
    * @param planeOrigin  the planeOrigin e.g (0,0,0)
    * @param planeNormal  the planeNormal is a vector that points upward from the plane e.g Vector3f.UNIT_Z
    * @param weldThreshold how much to smooth the vertices near the plane suggested amount from 0.001 until 0.1. 
    *                      Set it to negative number to disable welding.
    */
   public MeshData createSymmetricMesh(Vector3f planeOrigin, Vector3f planeNormal, float weldThreshold)
   {
      MeshData mesh = this.shallowClone();
      MeshData mirrorMesh = mesh.mirror(planeOrigin, planeNormal);

      float[] weldedMeshVertexArray = new float[mesh.vertexArray.length];
      float[] weldedMirrorMeshVertexArray = new float[mirrorMesh.vertexArray.length];
      float[] weldedMeshNormalArray = new float[mesh.normalArray.length];
      float[] weldedMirrorMeshNormalArray = new float[mirrorMesh.normalArray.length];

      for (int i = 0; i < mesh.getNumberOfIndexElements(); i++)
      {
         int index = mesh.getIndex(i);
         Vector3f p1 = mesh.getVertex(index);
         Vector3f mp1 = mirrorMesh.getVertex(index);
         Vector3f n1 = mesh.getNormal(index);
         Vector3f mn1 = mirrorMesh.getNormal(index);

         if (Utilities.getDistanceOfPointFromPlane(p1, planeOrigin, planeNormal) <= weldThreshold)
         {
            p1 = p1.clone().interpolateLocal(mp1, 0.5f);
            n1 = n1.clone().interpolateLocal(mn1, 0.5f);

            mp1 = p1;
            mn1 = n1;
         }//if

         Utilities.setInArray(p1, weldedMeshVertexArray, index);
         Utilities.setInArray(mp1, weldedMirrorMeshVertexArray, index);
         Utilities.setInArray(n1, weldedMeshNormalArray, index);
         Utilities.setInArray(mn1, weldedMirrorMeshNormalArray, index);
      }
      mesh.vertexArray = weldedMeshVertexArray;
      mesh.normalArray = weldedMeshNormalArray;
      mirrorMesh.vertexArray = weldedMirrorMeshVertexArray;
      mirrorMesh.normalArray = weldedMirrorMeshNormalArray;
      MeshData resultMesh = mesh.merge(mirrorMesh);
      return resultMesh;
   }

   public MeshData scale(Vector3f size)
   {
      return scale(size, getCenter());
   }

   public MeshData scale(Vector3f size, Vector3f pivot)
   {
      MeshData returned = this.shallowClone();
      float[] vertex = returned.getVertexArrayCopy();
      for (int i = 0; i < returned.getNumberOfVertexElements(); i++)
      {
         Vector3f v = returned.getVertex(i);
         Utilities.setInArray(new Vector3f((v.x - pivot.x) * size.x + pivot.x, (v.y - pivot.y) * size.y + pivot.y, (v.z - pivot.z) * size.z + pivot.z), vertex, i);
      }
      returned.vertexArray = vertex;
      return returned;
   }
   
   public MeshData translate(Vector3f translate)
   {
      MeshData returned = this.shallowClone();
      float[] vertex = returned.getVertexArrayCopy();
      for (int i = 0; i < returned.getNumberOfVertexElements(); i++)
      {
         Vector3f v = returned.getVertex(i);
         Utilities.setInArray(v.add(translate), vertex, i);
      }
      returned.vertexArray = vertex;
      return returned;
   }
   
   /** Usage :
      Vector3f min = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
      Vector3f max = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);
      getBounds(min,max);
    */
   public void getBounds(Vector3f min, Vector3f max)
   {
      for (int i = 0; i < this.getNumberOfVertexElements(); i++)
      {
         Vector3f v = this.getVertex(i);
         if (v.x < min.x) min.x = v.x;
         if (v.y < min.y) min.y = v.y;
         if (v.z < min.z) min.z = v.z;
         if (v.x > max.x) max.x = v.x;
         if (v.y > max.y) max.y = v.y;
         if (v.z > max.z) max.z = v.z;
      }
   }

   public Vector3f getCenter()
   {
      Vector3f min = new Vector3f(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE);
      Vector3f max = new Vector3f(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE);
      getBounds(min, max);
      return min.addLocal(max.subtractLocal(min).divideLocal(2));
   }

   /** Changes the order of indexes, e.g from 1,2,3 it becomes 3,2,1. 
    *  This will allow an invisible mesh to be seen.*/
   public MeshData flipIndexes()
   {
      MeshData mesh = this;
      MeshData result = mesh.shallowClone();
      result.indexArray = new short[mesh.indexArray.length];
      for (int i = 0; i < mesh.indexArray.length; i += 3)
      {
         result.indexArray[i] = mesh.indexArray[i + 2];
         result.indexArray[i + 1] = mesh.indexArray[i + 1];
         result.indexArray[i + 2] = mesh.indexArray[i];
      }
      return result;
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public MeshData flipU(int TexCoordNumber)
   {
      MeshData mesh = this;
      MeshData result = mesh.shallowClone();
      result.uvArrays[TexCoordNumber] = new float[mesh.uvArrays[TexCoordNumber].length];
      for (int i = 0; i < mesh.uvArrays[TexCoordNumber].length; i += 2)
      {
         result.uvArrays[TexCoordNumber][i] = (1 - mesh.uvArrays[TexCoordNumber][i]) % 1;
         result.uvArrays[TexCoordNumber][i + 1] = mesh.uvArrays[TexCoordNumber][i + 1] % 1;
      }
      return result;
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public MeshData flipV(int TexCoordNumber)
   {
      MeshData mesh = this;
      MeshData result = mesh.shallowClone();
      result.uvArrays[TexCoordNumber] = new float[mesh.uvArrays[TexCoordNumber].length];
      for (int i = 0; i < mesh.uvArrays[TexCoordNumber].length; i += 2)
      {
         result.uvArrays[TexCoordNumber][i] = mesh.uvArrays[TexCoordNumber][i] % 1;
         result.uvArrays[TexCoordNumber][i + 1] = (1 - mesh.uvArrays[TexCoordNumber][i + 1]) % 1;
      }
      return result;
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public MeshData moveUV(Vector2f uvMoveOffset, int TexCoordNumber)
   {
      MeshData mesh = this;
      MeshData result = mesh.shallowClone();
      result.uvArrays[TexCoordNumber] = new float[mesh.uvArrays[TexCoordNumber].length];
      for (int i = 0; i < mesh.uvArrays[TexCoordNumber].length; i += 2)
      {
         result.uvArrays[TexCoordNumber][i] = (mesh.uvArrays[TexCoordNumber][i] + uvMoveOffset.x) % 1;
         result.uvArrays[TexCoordNumber][i + 1] = (mesh.uvArrays[TexCoordNumber][i + 1] + uvMoveOffset.y) % 1;
      }
      return result;
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public MeshData rotateUV(float uvRotateAngle, int TexCoordNumber)
   {
      MeshData mesh = this;
      MeshData result = mesh.shallowClone();
      result.uvArrays[TexCoordNumber] = new float[mesh.uvArrays[TexCoordNumber].length];
      for (int i = 0; i < mesh.uvArrays[TexCoordNumber].length; i += 2)
      {
         result.uvArrays[TexCoordNumber][i] = (mesh.uvArrays[TexCoordNumber][i] * FastMath.cos(uvRotateAngle) - mesh.uvArrays[TexCoordNumber][i + 1] * FastMath.sin(uvRotateAngle)) % 1;
         result.uvArrays[TexCoordNumber][i + 1] = (mesh.uvArrays[TexCoordNumber][i] * FastMath.sin(uvRotateAngle) + mesh.uvArrays[TexCoordNumber][i + 1] * FastMath.cos(uvRotateAngle)) % 1;
      }
      return result;
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public MeshData scaleUV(Vector2f uvScaleAmount, int TexCoordNumber)
   {
      MeshData mesh = this;
      MeshData result = mesh.shallowClone();
      result.uvArrays[TexCoordNumber] = new float[mesh.uvArrays[TexCoordNumber].length];
      for (int i = 0; i < mesh.uvArrays[TexCoordNumber].length; i += 2)
      {
         result.uvArrays[TexCoordNumber][i] = (mesh.uvArrays[TexCoordNumber][i] * uvScaleAmount.x) % 1;
         result.uvArrays[TexCoordNumber][i + 1] = (mesh.uvArrays[TexCoordNumber][i + 1] * uvScaleAmount.y) % 1;
      }
      return result;
   }

   /** switches u with v.
    * @param TexCoordNumber which array of tex coords to affect. Usual choice : 0
    */
   public MeshData switchUV(int TexCoordNumber)
   {
      MeshData mesh = this;
      MeshData result = mesh.shallowClone();
      result.uvArrays[TexCoordNumber] = new float[mesh.uvArrays[TexCoordNumber].length];
      for (int i = 0; i < mesh.uvArrays[TexCoordNumber].length; i += 2)
      {
         result.uvArrays[TexCoordNumber][i] = result.uvArrays[TexCoordNumber][i + 1];
         result.uvArrays[TexCoordNumber][i + 1] = result.uvArrays[TexCoordNumber][i];
      }
      return result;
   }

   /** sets mesh to have the same data as this.*/
   public Mesh save(Mesh mesh)
   {
      if (vertexArray != null)
      {
         VertexBuffer vb = mesh.getBuffer(Type.Position);
         if (vb != null) vb.updateData(BufferUtils.createFloatBuffer(vertexArray));
         else mesh.setBuffer(Type.Position, 3, vertexArray);
      }

      if (normalArray != null)
      {
         VertexBuffer vb = mesh.getBuffer(Type.Normal);
         if (vb != null) vb.updateData(BufferUtils.createFloatBuffer(normalArray));
         else mesh.setBuffer(Type.Normal, 3, normalArray);
      }

      for (int i = 0; i < uvArrays.length; i++)
      {
         if (uvArrays[i] != null)
         {
            VertexBuffer vb = mesh.getBuffer(Utilities.getTexCoordType(i));
            if (vb != null) vb.updateData(BufferUtils.createFloatBuffer(uvArrays[i]));
            else mesh.setBuffer(Type.TexCoord, 2, uvArrays[i]);
         }
      }

      if (indexArray != null)
      {
         VertexBuffer vb = mesh.getBuffer(Type.Index);
         if (vb != null) vb.updateData(BufferUtils.createShortBuffer(indexArray));
         else mesh.setBuffer(Type.Index, 3, indexArray);
      }

      if (boneIndexArray != null && boneWeightArray != null)
      {
         // Setup bone weight buffer
         FloatBuffer weights = FloatBuffer.allocate(mesh.getVertexCount() * 4);
         VertexBuffer weightsBuf = new VertexBuffer(Type.BoneWeight);
         weightsBuf.setupData(Usage.CpuOnly, 4, Format.Float, weights);
         mesh.setBuffer(weightsBuf);

         // Setup bone index buffer
         ByteBuffer indices = ByteBuffer.allocate(mesh.getVertexCount() * 4);
         VertexBuffer indicesBuf = new VertexBuffer(Type.BoneIndex);
         indicesBuf.setupData(Usage.CpuOnly, 4, Format.UnsignedByte, indices);
         mesh.setBuffer(indicesBuf);

         //generateBindPose
         VertexBuffer bindPos = new VertexBuffer(Type.BindPosePosition);
         bindPos.setupData(Usage.CpuOnly, 3, Format.Float, BufferUtils.createFloatBuffer(vertexArray));
         mesh.setBuffer(bindPos);

         VertexBuffer bindNorm = new VertexBuffer(Type.BindPoseNormal);
         bindNorm.setupData(Usage.CpuOnly, 3, Format.Float, BufferUtils.createFloatBuffer(normalArray));
         mesh.setBuffer(bindNorm);

         for (int i = 0; i < mesh.getVertexCount() * 4; i += 4)
         {
            // assign vertex to bone index 0
            indices.array()[i + 0] = boneIndexArray[i];
            indices.array()[i + 1] = boneIndexArray[i + 1];
            indices.array()[i + 2] = boneIndexArray[i + 2];
            indices.array()[i + 3] = boneIndexArray[i + 3];

            // set weight to 1 only for first entry
            weights.array()[i + 0] = boneWeightArray[i];
            weights.array()[i + 1] = boneWeightArray[i + 1];
            weights.array()[i + 2] = boneWeightArray[i + 2];
            weights.array()[i + 3] = boneWeightArray[i + 3];
         }
      }

      if (vertexArray != null) mesh.updateBound();
      return mesh;
   }

   /** Removes duplicate and unindexed vertex, uv, normals if it can. lossless*/
   public MeshData removeDuplicateData()
   {
      short[] returnedIndexArray = this.getIndexArrayCopy();
      ArrayList<Vector3f> vertex = new ArrayList<Vector3f>();
      for (int i = 0; i < getNumberOfVertexElements(); i++)
      {
         vertex.add(getVertex(i));
      }

      for (int i = 0; i < getNumberOfVertexElements(); i++)
      {
         Vector3f vi = vertex.get(i);
         for (int j = i + 1; j < getNumberOfVertexElements(); j++)
         {
            Vector3f vj = vertex.get(j);
            if (vi.equals(vj) && getNormal(i).equals(getNormal(j)) && getUv(i).equals(getUv(j)))
            {
               returnedIndexArray[j] = (short) i;
            }
         }
      }
      MeshData returned = this.shallowClone();
      returned.indexArray = returnedIndexArray;
      returned = returned.compress();

      return returned;
   }

   /** removes all unindexed vertex/normals/uvs. lossless
    * The indexes change to match the new structure.
    */
   public MeshData compress()
   {
      MeshData mesh = this;
      int[] numOfUnusedElements = new int[mesh.getNumberOfVertexElements()];
      Arrays.fill(numOfUnusedElements, -1); //-1 = unused. != -1, counts the number of unused spaces before it.

      //find those unindexed elements.
      for (int i = 0; i < mesh.getNumberOfIndexElements(); i++)
      {
         numOfUnusedElements[mesh.getIndex(i)] = 0; //0 = used but there are 0 unused elements behind it. Will update later.
      }

      //Update numOfUnusedElements to show the number of unused elements behind it.
      int numOfUsed = 0;
      int numOfUnused = 0;
      for (int i = 0; i < numOfUnusedElements.length; i++)
      {
         if (numOfUnusedElements[i] == -1) numOfUnused++;
         else
         {
            numOfUsed++;
            numOfUnusedElements[i] = numOfUnused;
         }
      }

      //Compress Vertex Array / Normal Array / uvArray
      int m = 0;
      float[] compressedVertexArray = new float[numOfUsed * 3];
      float[] compressedNormalArray = new float[numOfUsed * 3];
      float[][] compressedUvArray = new float[mesh.uvArrays.length][];
      for (int t = 0; t < compressedUvArray.length; t++)
      {
         compressedUvArray[t] = new float[numOfUsed * 2];
      }

      for (int i = 0; i < numOfUnusedElements.length; i++)
      {
         if (numOfUnusedElements[i] != -1)
         {
            compressedVertexArray[m * 3] = mesh.vertexArray[i * 3];
            compressedVertexArray[m * 3 + 1] = mesh.vertexArray[i * 3 + 1];
            compressedVertexArray[m * 3 + 2] = mesh.vertexArray[i * 3 + 2];

            compressedNormalArray[m * 3] = mesh.normalArray[i * 3];
            compressedNormalArray[m * 3 + 1] = mesh.normalArray[i * 3 + 1];
            compressedNormalArray[m * 3 + 2] = mesh.normalArray[i * 3 + 2];

            for (int t = 0; t < compressedUvArray.length; t++)
            {
               compressedUvArray[t][m * 2] = mesh.uvArrays[t][i * 2];
               compressedUvArray[t][m * 2 + 1] = mesh.uvArrays[t][i * 2 + 1];
            }
            m++;
         }
      }

      //Update Compressed Index Array
      short[] compressedIndexArray = new short[mesh.indexArray.length];
      for (int i = 0; i < mesh.indexArray.length; i++)
      {
         compressedIndexArray[i] = (short) (mesh.indexArray[i] - numOfUnusedElements[mesh.indexArray[i]]);
      }

      return new MeshData(compressedVertexArray, compressedNormalArray, compressedUvArray, compressedIndexArray, mesh.boneWeightArray, mesh.boneIndexArray, mesh.maxNumOfWeights);
   }

   /** For each triangle, when this index is refered in a triangle, the triangle indexes are returned.*/
   public LinkedHashSet<Short> getAdjacentIndices(short index)
   {
      MeshData mesh = this;
      LinkedHashSet<Short> otherIndexes = new LinkedHashSet<Short>();

      for (int i = 0; i < mesh.indexArray.length; i += 3)
      {
         if (index == mesh.indexArray[i])
         {
            otherIndexes.add(mesh.indexArray[i + 1]);
            otherIndexes.add(mesh.indexArray[i + 2]);
         }

         if (index == mesh.indexArray[i + 1])
         {
            otherIndexes.add(mesh.indexArray[i]);
            otherIndexes.add(mesh.indexArray[i + 2]);
         }

         if (index == mesh.indexArray[i + 2])
         {
            otherIndexes.add(mesh.indexArray[i]);
            otherIndexes.add(mesh.indexArray[i + 1]);
         }
      }
      return otherIndexes;
   }

   public Mesh createMesh()
   {
      Mesh mesh = new Mesh();
      return save(mesh);
   }

   private MeshData shallowClone()
   {
      return new MeshData(vertexArray, normalArray, uvArrays, indexArray, boneWeightArray, boneIndexArray, maxNumOfWeights);
   }

   public MeshData setIndexArray(short[] indexArray)
   {
      MeshData mesh = this.shallowClone();
      mesh.indexArray = indexArray;
      return mesh;
   }

   public MeshData setNormalArray(float[] normalArray)
   {
      MeshData mesh = this.shallowClone();
      mesh.normalArray = normalArray;
      return mesh;
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public MeshData setUvArray(float[] uvArray, int TexCoordNumber)
   {
      MeshData mesh = this.shallowClone();
      mesh.uvArrays[TexCoordNumber] = uvArray;
      return mesh;
   }

   public MeshData setVertexArray(float[] vertexArray)
   {
      MeshData mesh = this.shallowClone();
      mesh.vertexArray = vertexArray;
      return mesh;
   }

   public MeshData setBoneWeightArray(float[] boneWeightArray)
   {
      MeshData mesh = this.shallowClone();
      mesh.boneWeightArray = boneWeightArray;
      return mesh;
   }

   public MeshData setBoneIndexArray(byte[] boneIndexArray)
   {
      MeshData mesh = this.shallowClone();
      mesh.boneIndexArray = boneIndexArray;
      return mesh;
   }

   public Vector3f getVertex(int i)
   {
      return Utilities.getVector3FromArray(vertexArray, i);
   }

   public Vector3f getNormal(int i)
   {
      return Utilities.getVector3FromArray(normalArray, i);
   }

   public Vector2f getUv(int i)
   {
      return getUv(i, 0);
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public Vector2f getUv(int i, int TexCoordNumber)
   {
      return Utilities.getVector2FromArray(uvArrays[TexCoordNumber], i);
   }

   public short getIndex(int i)
   {
      return indexArray[i];
   }

   private void setVertex(Vector3f p, int i)
   {
      Utilities.setInArray(p, vertexArray, i);
   }

   private void setNormal(Vector3f p, int i)
   {
      Utilities.setInArray(p, normalArray, i);
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   private void setUv(Vector2f p, int i, int TexCoordNumber)
   {
      Utilities.setInArray(p, uvArrays[TexCoordNumber], i);
   }

   private void setIndex(Short p, int i)
   {
      indexArray[i] = p;
   }

   public int getNumberOfVertexElements()
   {
      if (vertexArray == null) return 0;
      return vertexArray.length / 3;
   }

   public int getNumberOfNormalElements()
   {
      if (normalArray == null) return 0;
      return normalArray.length / 3;
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public int getNumberOfUvElements(int TexCoordNumber)
   {
      if (uvArrays[TexCoordNumber] == null) return 0;
      return uvArrays[TexCoordNumber].length / 2;
   }

   public int getNumberOfIndexElements()
   {
      if (indexArray == null) return 0;
      return indexArray.length;
   }

   public float[] getVertexArrayCopy()
   {
      if (vertexArray == null) return null;
      return vertexArray.clone();
   }

   public float[] getNormalArrayCopy()
   {
      if (normalArray == null) return null;
      return normalArray.clone();
   }

   /** @param TexCoordNumber which array of tex coords to affect. Usual choice : 0*/
   public float[] getUvArrayCopy(int TexCoordNumber)
   {
      if (uvArrays[TexCoordNumber] == null) return null;
      return uvArrays[TexCoordNumber].clone();
   }

   public short[] getIndexArrayCopy()
   {
      if (indexArray == null) return null;
      return indexArray.clone();
   }

   public float[] getBoneWeightArrayCopy()
   {
      if (boneWeightArray == null) return null;
      return boneWeightArray.clone();
   }

   public byte[] getBoneIndexArrayCopy()
   {
      if (boneIndexArray == null) return null;
      return boneIndexArray.clone();
   }
}