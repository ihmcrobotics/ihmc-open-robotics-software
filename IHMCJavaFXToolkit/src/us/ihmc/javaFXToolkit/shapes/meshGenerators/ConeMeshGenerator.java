package us.ihmc.javaFXToolkit.shapes.meshGenerators;

public class ConeMeshGenerator
{
   public static final int DEFAULT_DIVISIONS = 128;

   public static final float[] defaultTexCoords = generateTexCoords(DEFAULT_DIVISIONS);
   public static final int[] defaultFaces = generatreFaces(DEFAULT_DIVISIONS);
   public static final int[] defaultFaceSmoothingGroups = generateFaceSmoothingGroups(DEFAULT_DIVISIONS);

   public static float[] generatePoints(float radius, float height, int divisions)
   {
      float points[] = new float[(divisions + 2) * 3];
      double dAngle = -2.0 * Math.PI / divisions;
      int index = 0;

      for (int i = 0; i < divisions; i++)
      {
         double a = dAngle * i;
         points[index++] = (float) (Math.sin(a) * radius);
         points[index++] = (float) (Math.cos(a) * radius);
         points[index++] = 0.0f;
      }

      // The top
      points[index++] = 0.0f;
      points[index++] = 0.0f;
      points[index++] = height;

      // The base central point
      points[index++] = 0.0f;
      points[index++] = 0.0f;
      points[index++] = 0.0f;

      return points;
   }

   static float[] generateTexCoords(int divisions)
   {
      float texCoords[] = new float[(divisions + 2) * 2];
      float dA = -1.0f / divisions;

      int index = 0;
      for (int i = 0; i < divisions; i++)
      {
         double a = (dA * i * 2) * Math.PI;
         texCoords[index++] = 0.5f + (float) (Math.cos(a) * 0.5f);
         texCoords[index++] = 0.5f + (float) (Math.sin(a) * 0.5f);
      }

      texCoords[index++] = 0.5f;
      texCoords[index++] = 0.5f;
      texCoords[index++] = 0.5f;
      texCoords[index++] = 0.5f;

      return texCoords;
   }

   static int[] generatreFaces(int divisions)
   {
      int faces[] = new int[divisions * 6 * 2];

      int fIndex = 0;

      int topIndex = divisions;
      int baseCenterIndex = divisions + 1;

      for (int p0 = 0; p0 < divisions; p0++)
      {
         int p1 = (p0 + 1) % divisions;

         faces[fIndex + 0] = baseCenterIndex;
         faces[fIndex + 2] = p1;
         faces[fIndex + 4] = p0;

         faces[fIndex + 1] = baseCenterIndex;
         faces[fIndex + 3] = p1;
         faces[fIndex + 5] = p0;
         fIndex += 6;

         faces[fIndex + 0] = p0;
         faces[fIndex + 2] = p1;
         faces[fIndex + 4] = topIndex;

         faces[fIndex + 1] = p0;
         faces[fIndex + 3] = p1;
         faces[fIndex + 5] = topIndex;
         fIndex += 6;

      }
      return faces;
   }

   static int[] generateFaceSmoothingGroups(int divisions)
   {
      int smoothing[] = new int[divisions * 2];
     for (int i = 0; i < divisions; ++i) 
         smoothing[i] = 1;
     for (int i = divisions; i < divisions * 2; ++i) 
         smoothing[i] = 1;
     return smoothing;
   }
}