package us.ihmc.javaFXToolkit.shapes.meshGenerators;

public class CylinderMeshGenerator
{
   public static FXMeshDataHolder cylinderMesh(double height, double radius)
   {
      return cylinderMesh(height, radius, DEFAULT_DIVISIONS);
   }

   public static FXMeshDataHolder cylinderMesh(double height, double radius, int divisions)
   {
      return cylinderMesh((float) height, (float) radius, divisions);
   }

   public static FXMeshDataHolder cylinderMesh(float height, float radius)
   {
      return cylinderMesh(height, radius, DEFAULT_DIVISIONS);
   }

   public static FXMeshDataHolder cylinderMesh(float height, float radius, int divisions)
   {
      FXMeshDataHolder cylinderMesh = new FXMeshDataHolder();
      cylinderMesh.setVertexCoordinates(generatePoints(height, radius, divisions));
      cylinderMesh.setTextureCoordinates(generateTexCoords(divisions));
      cylinderMesh.setFaceIndices(generatreFaces(divisions));
      cylinderMesh.setFaceSmoothingGroups(generateFaceSmoothingGroups(divisions));
      return cylinderMesh;
   }

   public static final int DEFAULT_DIVISIONS = 64;

   public static final float[] defaultTexCoords = generateTexCoords(DEFAULT_DIVISIONS);
   public static final int[] defaultFaces = generatreFaces(DEFAULT_DIVISIONS);
   public static final int[] defaultFaceSmoothingGroups = generateFaceSmoothingGroups(DEFAULT_DIVISIONS);

   public static float[] generatePoints(float height, float radius, int divisions)
   {
      float points[] = new float[(divisions * 2 + 2) * 3];
      double dAngle = -2.0 * Math.PI / divisions;
      int index = 0;


      for (int i = 0; i < divisions; i++)
      {
         double a = dAngle * i;
         points[index++] = (float) (Math.sin(a) * radius);
         points[index++] = (float) (Math.cos(a) * radius);
         points[index++] = height;
      }

      for (int i = 0; i < divisions; i++)
      {
         double a = dAngle * i;
         points[index++] = (float) (Math.sin(a) * radius);
         points[index++] = (float) (Math.cos(a) * radius);
         points[index++] = 0.0f;
      }

      // add cap central points
      points[index++] = 0.0f;
      points[index++] = 0.0f;
      points[index++] = height;
      points[index++] = 0.0f;
      points[index++] = 0.0f;
      points[index++] = 0.0f;

      return points;
   }

   public static float[] generateTexCoords(int divisions)
   {
      float texCoords[] = new float[((divisions + 1) * 4 + 1) * 2];
      float textureDelta = 1.0f / 256;
      float dA = -1.0f / divisions;

      int index = 0;

      for (int i = 0; i < divisions; i++)
      {
         texCoords[index++] = 1 - dA * i;
         texCoords[index++] = 1 - textureDelta;
      }

      // top edge
      texCoords[index++] = 0;
      texCoords[index++] = 1 - textureDelta;

      for (int i = 0; i < divisions; i++)
      {
         texCoords[index++] = 1 - dA * i;
         texCoords[index++] = textureDelta;
      }

      // bottom edge
      texCoords[index++] = 0;
      texCoords[index++] = textureDelta;

      // add cap central points
      // bottom cap
      for (int i = 0; i <= divisions; i++)
      {
         double a = (i < divisions) ? (dA * i * 2) * Math.PI : 0;
         texCoords[index++] = 0.5f + (float) (Math.sin(a) * 0.5f);
         texCoords[index++] = 0.5f + (float) (Math.cos(a) * 0.5f);
      }

      // top cap
      for (int i = 0; i <= divisions; ++i)
      {
         double a = (i < divisions) ? (dA * i * 2) * Math.PI : 0;
         texCoords[index++] = 0.5f + (float) (Math.sin(a) * 0.5f);
         texCoords[index++] = 0.5f - (float) (Math.cos(a) * 0.5f);
      }

      texCoords[index++] = 0.5f;
      texCoords[index++] = 0.5f;

      return texCoords;
   }

   public static int[] generatreFaces(int divisions)
   {
      int faces[] = new int[divisions * 24];

      int fIndex = 0;

      for (int p0 = 0; p0 < divisions; p0++)
      {
         int p1 = p0 + 1;
         int p2 = p0 + divisions;
         int p3 = p1 + divisions;

         faces[fIndex + 0] = p0;
         faces[fIndex + 1] = p0;
         faces[fIndex + 2] = p2;
         faces[fIndex + 3] = p2 + 1;
         faces[fIndex + 4] = p1 == divisions ? 0 : p1;
         faces[fIndex + 5] = p1;
         fIndex += 6;

         faces[fIndex + 0] = p3 % divisions == 0 ? p3 - divisions : p3;
         faces[fIndex + 1] = p3 + 1;
         faces[fIndex + 2] = p1 == divisions ? 0 : p1;
         faces[fIndex + 3] = p1;
         faces[fIndex + 4] = p2;
         faces[fIndex + 5] = p2 + 1;
         fIndex += 6;

      }
      // build cap faces
      int tStart = (divisions + 1) * 2;
      int t1 = (divisions + 1) * 4;
      int p1 = divisions * 2;

      // bottom cap
      for (int p0 = 0; p0 < divisions; p0++)
      {
         int p2 = p0 + 1;
         int t0 = tStart + p0;
         int t2 = t0 + 1;

         // add p0, p1, p2
         faces[fIndex + 0] = p0;
         faces[fIndex + 1] = t0;
         faces[fIndex + 2] = p2 == divisions ? 0 : p2;
         faces[fIndex + 3] = t2;
         faces[fIndex + 4] = p1;
         faces[fIndex + 5] = t1;
         fIndex += 6;
      }

      p1 = divisions * 2 + 1;
      tStart = (divisions + 1) * 3;

      // top cap
      for (int p0 = 0; p0 < divisions; p0++)
      {
         int p2 = p0 + 1 + divisions;
         int t0 = tStart + p0;
         int t2 = t0 + 1;

         faces[fIndex + 0] = p0 + divisions;
         faces[fIndex + 1] = t0;
         faces[fIndex + 2] = p1;
         faces[fIndex + 3] = t1;
         faces[fIndex + 4] = p2 % divisions == 0 ? p2 - divisions : p2;
         faces[fIndex + 5] = t2;
         fIndex += 6;
      }

      return faces;
   }

   public static int[] generateFaceSmoothingGroups(int divisions)
   {
      int smoothing[] = new int[divisions * 4];
     for (int i = 0; i < divisions * 2; ++i) 
         smoothing[i] = 1;
     for (int i = divisions * 2; i < divisions * 4; ++i) 
         smoothing[i] = 2;
     return smoothing;
   }
}