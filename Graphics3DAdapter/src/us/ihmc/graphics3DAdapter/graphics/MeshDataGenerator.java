package us.ihmc.graphics3DAdapter.graphics;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.TexCoord2f;

import us.ihmc.robotics.geometry.ConvexPolygon2d;

public class MeshDataGenerator
{

   private MeshDataGenerator()
   {
      // Prevent an object being generated.
   }
   
   public static MeshDataHolder Sphere(double radius, int N, int M)
   {
      return Sphere((float) radius, N, M);
   }

   public static MeshDataHolder Sphere(float radius, int N, int M)
   {
      return Ellipsoid(radius, radius, radius, N, M);
   }

   public static MeshDataHolder Sphere(double xCenter, double yCenter, double zCenter, double radius, int N, int M)
   {
      return Ellipsoid(xCenter, yCenter, zCenter, radius, radius, radius, N, M);
   }

   public static MeshDataHolder Ellipsoid(double xRad, double yRad, double zRad, int N, int M)
   {
      return Ellipsoid(0.0f, 0.0f, 0.0f, (float) xRad, (float) yRad, (float) zRad, N, M);
   }

   public static MeshDataHolder Ellipsoid(double xCenter, double yCenter, double zCenter, double xRad, double yRad, double zRad, int N, int M)
   {
      return Ellipsoid((float) xCenter, (float) yCenter, (float) zCenter, (float) xRad, (float) yRad, (float) zRad, N, M);
   }

   public static MeshDataHolder Ellipsoid(float xCenter, float yCenter, float zCenter, float xRad, float yRad, float zRad, int N, int M)
   {
      float longitude;
      float ztheta;

      // FIXME: polygons at the opposite z-ends are done improperly, possibly in the cw order rather than ccw. This is not the case in the HemiEllipsoid, so maybe look for differences?
      
      // FIXME: Array should be smaller, or completely filled. Temporary solution uses smaller arrays.
//      Point3f points[] = new Point3f[(N + 1) * (M + 1)];
//      TexCoord2f textPoints[] = new TexCoord2f[(N + 1) * (M + 1)];
      Point3f points[] = new Point3f[(N + 1) * (M) + 1];
      TexCoord2f textPoints[] = new TexCoord2f[(N + 1) * (M) + 1];

      // for (int i=0;i<=N;i++)
      for (int j = 0; j <= M; j++)
      {
         // ztheta = (float) (-Math.PI/2.0f + Math.PI * ((float) i / (float) N));
         longitude = (float) (2.0 * Math.PI * ((float) j / (float) M));

         // for (int j=0;j<=M;j++)
         for (int i = 0; i <= N; i++)
         {
            // longitude = (float) (2.1*Math.PI*((float) j / (float) M));
            ztheta = (float) (-Math.PI / 2.0f + Math.PI * ((float) i / (float) N));

            points[i * M + j] = new Point3f((float) (xCenter + xRad * Math.cos(longitude) * Math.cos(ztheta)),
                                            (float) (yCenter + yRad * Math.sin(longitude) * Math.cos(ztheta)), (float) (zCenter + zRad * Math.sin(ztheta)));

            textPoints[i * M + j] = new TexCoord2f((float) (longitude / (2.0 * Math.PI)), (float) (0.5 * Math.sin(ztheta) + 0.5));
         }
      }

      int[] polygonIndices = new int[4* M * N];

      int index = 0;

      for (int i = 0; i < N; i++)
      {
         for (int j = 0; j < M; j++)
         {
            polygonIndices[index] = i * M + j;
            polygonIndices[index + 1] = i * M + j + 1;
            polygonIndices[index + 2] = (i + 1) * M + j + 1;
            polygonIndices[index + 3] = (i + 1) * M + j;

            index = index + 4;
         }
      }

      int[] pStripCounts = new int[(M) * (N)];
      for (int i = 0; i < (N) * (M); i++)
      {
         pStripCounts[i] = 4;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts); 
   }


   public static MeshDataHolder Polygon(ArrayList<Point3d> polygonPoints)
   {
      return Polygon(polygonPoints, polygonPoints.size());
   }


   public static MeshDataHolder Polygon(ArrayList<Point3d> polygonPoints, int numberOfVertices)
   {
      Point3f[] points = new Point3f[numberOfVertices];
      for (int i = 0; i < numberOfVertices; i++)
      {
         points[i] = new Point3f(polygonPoints.get(i));
      }

      return Polygon(points);
   }


   public static MeshDataHolder Polygon(ConvexPolygon2d convexPolygon)
   {
      Point3f[] points = new Point3f[convexPolygon.getNumberOfVertices()];
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         Point2d vertex = convexPolygon.getVertex(i);
         points[i] = new Point3f((float) vertex.x, (float) vertex.y, 0.0f);
      }

      return Polygon(points);
   }


   public static MeshDataHolder Polygon(Point3f[] polygonPoints)
   {
      // Assume convexity and ccw.
      int[] polygonIndices = new int[polygonPoints.length];
      int[] polygonStripCounts = new int[] {polygonPoints.length};
      
      for (int i = 0; i < polygonPoints.length; i++)
      {
         polygonIndices[i] = i;
      }

      return new MeshDataHolder(polygonPoints, generateInterpolatedTexturePoints(polygonPoints.length), polygonIndices, polygonStripCounts);
   }

   public static MeshDataHolder Polygon(Point3d[] polygonPoints)
   {
      Point3f[] points = makePoint3fArrayFromPoint3dArray(polygonPoints);

      return Polygon(points);
   }


   public static MeshDataHolder ExtrudedPolygon(ConvexPolygon2d convexPolygon2d, double extrusionHeight)
   {
      Point2d[] points = new Point2d[convexPolygon2d.getNumberOfVertices()];
      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         points[i] = new Point2d(convexPolygon2d.getVertex(i));
      }

      return ExtrudedPolygon(points, extrusionHeight);
   }

   public static MeshDataHolder ExtrudedPolygon(List<Point2d> polygonPoints, double extrusionHeight)
   {
      Point2d[] points = new Point2d[polygonPoints.size()];
      int i = 0;
      for (Point2d point2d : polygonPoints)
      {
         points[i++] = new Point2d(point2d);
      }

      return ExtrudedPolygon(points, extrusionHeight);
   }

   public static MeshDataHolder ExtrudedPolygon(Point2d[] polygonPoints, double extrusionHeight)
   {
      int numberOfPointsOnPolygon = polygonPoints.length;

      Point3f vertices[] = new Point3f[2 * numberOfPointsOnPolygon];
      TexCoord2f[] texturePoints = new TexCoord2f[vertices.length];  

      for (int i = 0; i < numberOfPointsOnPolygon; i++)
      {
         vertices[i] = new Point3f((float) (polygonPoints[i].getX()), (float) (polygonPoints[i].getY()), 0.0f);
         vertices[i + numberOfPointsOnPolygon] = new Point3f((float) (polygonPoints[i].getX()), (float) (polygonPoints[i].getY()), (float) extrusionHeight);
      
         //TODO: Figure out how to texture an extruded polygon!
         texturePoints[i] = new TexCoord2f();
         texturePoints[i + numberOfPointsOnPolygon] = new TexCoord2f();
      }
      
      int[] polygonIndices = new int[2 * numberOfPointsOnPolygon + 4 * numberOfPointsOnPolygon];
      int[] polygonStripCounts = new int[2 + numberOfPointsOnPolygon];

      // Bottom Polygon:
      int indexCount = 0;
      for (int i=0; i<numberOfPointsOnPolygon; i++)
      {
         polygonIndices[indexCount] = i;
         indexCount++;
      }
      polygonStripCounts[0] = numberOfPointsOnPolygon;
      
      // Top Polygon:
      for (int i=0; i<numberOfPointsOnPolygon; i++)
      {
         polygonIndices[indexCount] = numberOfPointsOnPolygon + i;
         indexCount++;
      }
      polygonStripCounts[1] = numberOfPointsOnPolygon;

      // Side rectangles:
      for (int i=0; i<numberOfPointsOnPolygon; i++)
      {
         polygonIndices[indexCount++] = i;
         polygonIndices[indexCount++] = (i + 1) % numberOfPointsOnPolygon;


         int index = numberOfPointsOnPolygon + i + 1;
         if (index >= 2.0 * numberOfPointsOnPolygon)
         {
            index = numberOfPointsOnPolygon;
         }
         polygonIndices[indexCount++] = index;

         polygonIndices[indexCount++] = numberOfPointsOnPolygon + i;
         
         polygonStripCounts[2+i] = 4;
      }
             
      return new MeshDataHolder(vertices, texturePoints, polygonIndices, polygonStripCounts);
   }



   public static MeshDataHolder HemiEllipsoid(double xRad, double yRad, double zRad, int N, int M)
   {
      return HemiEllipsoid((float) xRad, (float) yRad, (float) zRad, N, M);
   }

   public static MeshDataHolder HemiEllipsoid(float xRad, float yRad, float zRad, int N, int M)
   {
      float longitude;
      float ztheta;

      Point3f points[] = new Point3f[N * M];
      TexCoord2f[] textPoints = new TexCoord2f[N * M];

      for (int i = 0; i < N; i++)
      {
         ztheta = (float) (Math.PI / 2.0 * ((float) i / N));

         for (int j = 0; j < M; j++)
         {
            longitude = (float) (2.0 * Math.PI * j / M);

            points[i * M + j] = new Point3f((float) (xRad * Math.cos(longitude) * Math.cos(ztheta)), (float) (yRad * Math.sin(longitude) * Math.cos(ztheta)),
                                            (float) (zRad * Math.sin(ztheta)));

            textPoints[i * M + j] = new TexCoord2f((float) (longitude / (2.0 * Math.PI)), (float) (0.5 * Math.sin(ztheta) + 0.5));
         }
      }

      int[] polygonIndices = new int[4 * M * (N - 1) + 2 * M];

      int index = 0;

      for (int i = 0; i < N - 1; i++)
      {
         for (int j = 0; j < M - 1; j++)
         {
            polygonIndices[index] =     i * M + j;
            polygonIndices[index + 1] = i * M + j + 1;
            polygonIndices[index + 2] = (i + 1) * M + j + 1;
            polygonIndices[index + 3] = (i + 1) * M + j;
            index = index + 4;
         }

         polygonIndices[index] =     (i + 1) * M - 1;
         polygonIndices[index + 1] = i * M;
         polygonIndices[index + 2] = (i + 1) * M;
         polygonIndices[index + 3] = (i + 2) * M - 1;
         index = index + 4;
      }

      // Just cap both ends for now...
      // Need to figure out the right way to do spheres, etc...

      for (int j = 0; j < M; j++)
      {
         polygonIndices[index] =     M - j - 1;
         polygonIndices[index + M] = (N - 1) * M + j;
         index = index + 1;
      }

      index = index + M;

      int[] pStripCounts = new int[M * (N - 1) + 2];

      for (int i = 0; i < (N - 1) * M; i++)
      {
         pStripCounts[i] = 4;
      }

      pStripCounts[(N - 1) * M] = M;
      pStripCounts[(N - 1) * M + 1] = M;

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   public static MeshDataHolder Cylinder(double radius, double height, int N)
   {
      return Cylinder((float) radius, (float) height, N);
   }

   public static MeshDataHolder Cylinder(float radius, float height, int N)
   {
      Point3f points[] = new Point3f[2 * N];
      TexCoord2f textPoints[] = new TexCoord2f[2 * N];

      for (int i = 0; i < N; i++)
      {
         points[i] = new Point3f((float) (radius * Math.cos(i * 2.0 * Math.PI / N)), (float) (radius * Math.sin(i * 2.0 * Math.PI / N)), 0.0f);
         points[i + N] = new Point3f((float) (radius * Math.cos(i * 2.0 * Math.PI / N)), (float) (radius * Math.sin(i * 2.0 * Math.PI / N)), height);
         
         textPoints[i] = new TexCoord2f((float) (0.5f * Math.cos(i * 2.0 * Math.PI / N) + 0.5f), (float) (0.5f * Math.sin(i * 2.0 * Math.PI / N) + 0.5f));
         textPoints[i + N] = new TexCoord2f((float) (0.5f * Math.cos(i * 2.0 * Math.PI / N) + 0.5f), (float) (0.5f * Math.sin(i * 2.0 * Math.PI / N) + 0.5f));
      }

      int[] polygonIndices = new int[2 * N + 4 * N];

      int index = 0;

      for (int i = 0; i < N; i++)
      {
         polygonIndices[index] =     N - 1 - i;
         polygonIndices[index + N] = N + i;
         index = index + 1;
      }

      index = index + N;

      for (int i = 0; i < N - 1; i++)
      {
         polygonIndices[index] =     i;
         polygonIndices[index + 1] = i + 1;
         polygonIndices[index + 2] = N + i + 1;
         polygonIndices[index + 3] = N + i;
         index = index + 4;
      }

      polygonIndices[index] =     N - 1;
      polygonIndices[index + 1] = 0;
      polygonIndices[index + 2] = N;
      polygonIndices[index + 3] = 2 * N - 1;

      int[] pStripCounts = new int[2 + N];

      pStripCounts[0] = N;
      pStripCounts[1] = N;

      for (int i = 2; i < N + 2; i++)
      {
         pStripCounts[i] = 4;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   public static MeshDataHolder Cone(double height, double radius, int N)
   {
      return Cone((float) height, (float) radius, N);
   }

   public static MeshDataHolder Cone(float height, float radius, int N)
   {
      Point3f[] points = new Point3f[N + 1];
      TexCoord2f[] textPoints = new TexCoord2f[N + 1];

      for (int i = 0; i < N; i++)
      {
         points[i] = new Point3f((float) (radius * Math.cos(i * 2.0 * Math.PI / N)), (float) (radius * Math.sin(i * 2.0 * Math.PI / N)), 0.0f);
         textPoints[i] = new TexCoord2f((float) (0.5f * Math.cos(i * 2.0 * Math.PI / N) + 0.5f), (float) (0.5f * Math.sin(i * 2.0 * Math.PI / N) + 0.5f));
      }

      points[N] = new Point3f(0.0f, 0.0f, height);
      textPoints[N] = new TexCoord2f(0.5f, 0.5f);
      
      int[] polygonIndices = new int[N + 3 * N];

      int index = 0;

      for (int i = 0; i < N; i++)
      {
         polygonIndices[index] = N - 1 - i;
         index = index + 1;
      }

      for (int i = 0; i < N - 1; i++)
      {
         polygonIndices[index] = i;
         polygonIndices[index + 1] = i + 1;
         polygonIndices[index + 2] = N;
         index = index + 3;
      }

      polygonIndices[index] = N - 1;
      polygonIndices[index + 1] = 0;
      polygonIndices[index + 2] = N;

      int[] pStripCounts = new int[1 + N];

      pStripCounts[0] = N;

      for (int i = 1; i < N + 1; i++)
      {
         pStripCounts[i] = 3;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }


   public static MeshDataHolder GenTruncatedCone(double height, double bx, double by, double tx, double ty, int N)
   {
      return GenTruncatedCone((float) height, (float) bx, (float) by, (float) tx, (float) ty, N);
   }


   public static MeshDataHolder GenTruncatedCone(float height, float bx, float by, float tx, float ty, int N)
   {
      Point3f points[] = new Point3f[2 * N];
      TexCoord2f[] textPoints = new TexCoord2f[2 * N]; 

      for (int i = 0; i < N; i++)
      {
         points[i] = new Point3f((float) (bx * Math.cos(i * 2.0 * Math.PI / N)), (float) (by * Math.sin(i * 2.0 * Math.PI / N)), 0.0f);
         textPoints[i] = new TexCoord2f((float) (0.5f * Math.cos(i * 2.0 * Math.PI / N)+0.5f), (float) (0.5f * Math.sin(i * 2.0 * Math.PI / N) + 0.5f));
      }

      for (int i = 0; i < N; i++)
      {
         points[i + N] = new Point3f((float) (tx * Math.cos(i * 2.0 * Math.PI / N)), (float) (ty * Math.sin(i * 2.0 * Math.PI / N)), height);
         textPoints[i + N] = new TexCoord2f((float) (0.5f * Math.cos(i * 2.0 * Math.PI / N) + 0.5f), (float) (0.5f * Math.sin(i * 2.0 * Math.PI / N) + 0.5f));
      }

      int[] polygonIndices = new int[N + N + 4 * N];

      int index = 0;

      // Bottom
      for (int i = 0; i < N; i++)
      {
         polygonIndices[index] = N - 1 - i;
         index = index + 1;
      }

      // Top
      for (int i = 0; i < N; i++)
      {
         polygonIndices[index] = N + i;
         index = index + 1;
      }

      for (int i = 0; i < N - 1; i++)
      {
         polygonIndices[index] = i;
         polygonIndices[index + 1] = i + 1;
         polygonIndices[index + 2] = N + i + 1;
         polygonIndices[index + 3] = N + i;
         index = index + 4;
      }

      polygonIndices[index] =     N - 1;
      polygonIndices[index + 1] = 0;
      polygonIndices[index + 2] = N;
      polygonIndices[index + 3] = 2 * N - 1;

      int[] pStripCounts = new int[2 + N];

      pStripCounts[0] = N;
      pStripCounts[1] = N;

      for (int i = 2; i < N + 2; i++)
      {
         pStripCounts[i] = 4;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }


   public static MeshDataHolder ArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, int N)
   {
      return ArcTorus((float) startAngle, (float) endAngle, (float) majorRadius, (float) minorRadius, N);
   }

   public static MeshDataHolder ArcTorus(float startAngle, float endAngle, float majorRadius, float minorRadius, int N)
   {
      Point3f points[] = new Point3f[N * N];
      TexCoord2f[] textPoints = new TexCoord2f[N * N];
      double angle1, angle2;
      double cenX, cenY;
      double pX, pY, pZ;
      float texY, texX;

      for (int i = 0; i < N; i++)
      {
         angle1 = startAngle + i * (endAngle - startAngle) / (N - 1);
         cenX = majorRadius * Math.cos(angle1);
         cenY = majorRadius * Math.sin(angle1);
         
         texY = (float) i / (float) N;

         for (int j = 0; j < N; j++)
         {
            angle2 = j * 2.0 * Math.PI / N;
            pX = cenX + minorRadius * Math.cos(angle1) * Math.cos(angle2);
            pY = cenY + minorRadius * Math.sin(angle1) * Math.cos(angle2);
            pZ = minorRadius * Math.sin(angle2);
            points[i * N + j] = new Point3f((float) pX, (float) pY, (float) pZ);
            
            texX = (float) j / (float) N;
            textPoints[i * N + j] = new TexCoord2f(texX, texY);
         }
      }

      int[] polygonIndices = new int[4 * (N - 1) * N];

      int index = 0;

      // Bottom
      for (int i = 0; i < N - 1; i++)
      {
         for (int j = 0; j < N - 1; j++)
         {
            polygonIndices[index + 3] = (i * N) + j;
            polygonIndices[index + 2] = (i * N) + j + 1;
            polygonIndices[index + 1] = (i + 1) * N + j + 1;
            polygonIndices[index] =     (i + 1) * N + j;

            index = index + 4;
         }

         polygonIndices[index + 3] = (i * N) + N - 1;
         polygonIndices[index + 2] = (i * N);
         polygonIndices[index + 1] = (i + 1) * N;
         polygonIndices[index] =     (i + 1) * N + N - 1;
         index = index + 4;
      }


      int[] pStripCounts = new int[(N - 1) * N];

      for (int i = 0; i < (N - 1) * N; i++)
      {
         pStripCounts[i] = 4;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   public static MeshDataHolder Cube(double lx, double ly, double lz, boolean centered )
   {
      return (Cube((float) lx, (float) ly, (float) lz,centered));
   }

   public static MeshDataHolder Cube(float lx, float ly, float lz, boolean centered )
   {
      Point3f points[] = new Point3f[8];

      TexCoord2f textPoints[] = new TexCoord2f[8];

      float za = centered ? -lz/2f : 0;
      float zb = centered ? lz/2f : lz;

      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, za);
      points[1] = new Point3f(lx / 2.0f, -ly / 2.0f, za);
      points[2] = new Point3f(lx / 2.0f, ly / 2.0f, za);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, za);
      points[4] = new Point3f(-lx / 2.0f, -ly / 2.0f, zb);
      points[5] = new Point3f(lx / 2.0f, -ly / 2.0f, zb);
      points[6] = new Point3f(lx / 2.0f, ly / 2.0f, zb);
      points[7] = new Point3f(-lx / 2.0f, ly / 2.0f, zb);

      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);
      textPoints[4] = new TexCoord2f(0.0f, 0.0f);
      textPoints[5] = new TexCoord2f(1.0f, 0.0f);
      textPoints[6] = new TexCoord2f(1.0f, 1.0f);
      textPoints[7] = new TexCoord2f(0.0f, 1.0f);

      int[] polygonIndices = new int[6 * 4];

      int index = 0;

      for (int i = 0; i < 3; i++)
      {
         polygonIndices[index] =     i;
         polygonIndices[index + 1] = i + 1;
         polygonIndices[index + 2] = 4 + i + 1;
         polygonIndices[index + 3] = 4 + i;

         index = index + 4;
      }

      polygonIndices[index] = 3;
      polygonIndices[index + 1] = 0;
      polygonIndices[index + 2] = 4;
      polygonIndices[index + 3] = 7;

      index = index + 4;

      polygonIndices[index] =     3;
      polygonIndices[index + 1] = 2;
      polygonIndices[index + 2] = 1;
      polygonIndices[index + 3] = 0;

      index = index + 4;

      polygonIndices[index] =     4;
      polygonIndices[index + 1] = 5;
      polygonIndices[index + 2] = 6;
      polygonIndices[index + 3] = 7;

      index = index + 4;
      
      int[] polygonStripCounts = new int[6];
      for(int i = 0; i < polygonStripCounts.length; i++)
      {
         polygonStripCounts[i] = 4;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, polygonStripCounts);
   }

   public static MeshDataHolder FlatRectangle(double xMin, double yMin, double xMax, double yMax, double z)
   {
      return FlatRectangle((float) xMin, (float) yMin, (float) xMax, (float) yMax, (float) z);
   }

   public static MeshDataHolder FlatRectangle(float xMin, float yMin, float xMax, float yMax, float z)
   {
      Point3f points[] = new Point3f[4];
      TexCoord2f textPoints[] = new TexCoord2f[4];

      points[0] = new Point3f(xMin, yMin, z);
      points[1] = new Point3f(xMax, yMin, z);
      points[2] = new Point3f(xMax, yMax, z);
      points[3] = new Point3f(xMin, yMax, z);

      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);
      
      int[] polygonIndices = new int[4];
      for(int i = 0; i < polygonIndices.length; i++)
      {
         polygonIndices[i] = i;
      }
      
      int[] polygonStripCounts = {4};

      return new MeshDataHolder(points, textPoints, polygonIndices, polygonStripCounts);
   }

   public static MeshDataHolder Rectangle(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3,
                                    double z3)
   {
      return Rectangle((float) x0, (float) y0, (float) z0, (float) x1, (float) y1, (float) z1, (float) x2, (float) y2, (float) z2, (float) x3, (float) y3,
                       (float) z3);
   }

   public static MeshDataHolder Rectangle(float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3)
   {
      Point3f points[] = new Point3f[4];

      TexCoord2f textPoints[] = new TexCoord2f[4];

      points[0] = new Point3f(x0, y0, z0);
      points[1] = new Point3f(x1, y1, z1);
      points[2] = new Point3f(x2, y2, z2);
      points[3] = new Point3f(x3, y3, z3);

      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);

      int[] polygonIndices = new int[4];
      for(int i = 0; i < polygonIndices.length; i++)
      {
         polygonIndices[i] = i;
      }
      
      int[] polygonStripCounts = {4};

      return new MeshDataHolder(points, textPoints, polygonIndices, polygonStripCounts);
   }

   public static MeshDataHolder Wedge(double lx, double ly, double lz)
   {
      return (Wedge((float) lx, (float) ly, (float) lz));
   }


   public static MeshDataHolder Wedge(float lx, float ly, float lz)
   {
      Point3f points[] = new Point3f[6];

      TexCoord2f textPoints[] = new TexCoord2f[6];

      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      points[1] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      points[2] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);

      points[4] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      points[5] = new Point3f(lx / 2.0f, ly / 2.0f, lz);


      textPoints[0] = new TexCoord2f(0.0f, 0.0f);
      textPoints[1] = new TexCoord2f(1.0f, 0.0f);
      textPoints[2] = new TexCoord2f(1.0f, 1.0f);
      textPoints[3] = new TexCoord2f(0.0f, 1.0f);
      
      textPoints[4] = new TexCoord2f(0.0f, 1.0f);
      textPoints[5] = new TexCoord2f(1.0f, 1.0f);

      int[] polygonIndices = new int[3 * 4 + 2 * 3];

      int index = 0;

      polygonIndices[index] =     3;
      polygonIndices[index + 1] = 2;
      polygonIndices[index + 2] = 1;
      polygonIndices[index + 3] = 0;

      index = index + 4;

      polygonIndices[index] =     2;
      polygonIndices[index + 1] = 5;
      polygonIndices[index + 2] = 4;
      polygonIndices[index + 3] = 1;

      index = index + 4;

      polygonIndices[index] =     4;
      polygonIndices[index + 1] = 5;
      polygonIndices[index + 2] = 3;
      polygonIndices[index + 3] = 0;

      index = index + 4;

      polygonIndices[index] =     1;
      polygonIndices[index + 1] = 4;
      polygonIndices[index + 2] = 0;

      index = index + 3;

      polygonIndices[index] =     3;
      polygonIndices[index + 1] = 5;
      polygonIndices[index + 2] = 2;

      index = index + 3;

      int[] pStripCounts = new int[5];

      for (int i = 0; i < 3; i++)
      {
         pStripCounts[i] = 4;
      }

      for (int i = 3; i < 5; i++)
      {
         pStripCounts[i] = 3;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }


   public static MeshDataHolder PyramidCube(double lx, double ly, double lz, double lh)
   {
      return (PyramidCube((float) lx, (float) ly, (float) lz, (float) lh));
   }

   public static MeshDataHolder PyramidCube(float lx, float ly, float lz, float lh)
   {
      Point3f points[] = new Point3f[10];

      TexCoord2f textPoints[] = new TexCoord2f[10];

      points[0] = new Point3f(-lx / 2.0f, -ly / 2.0f, 0.0f);
      points[1] = new Point3f(lx / 2.0f, -ly / 2.0f, 0.0f);
      points[2] = new Point3f(lx / 2.0f, ly / 2.0f, 0.0f);
      points[3] = new Point3f(-lx / 2.0f, ly / 2.0f, 0.0f);
      points[4] = new Point3f(-lx / 2.0f, -ly / 2.0f, lz);
      points[5] = new Point3f(lx / 2.0f, -ly / 2.0f, lz);
      points[6] = new Point3f(lx / 2.0f, ly / 2.0f, lz);
      points[7] = new Point3f(-lx / 2.0f, ly / 2.0f, lz);

      points[8] = new Point3f(0.0f, 0.0f, lz + lh);
      points[9] = new Point3f(0.0f, 0.0f, -lh);

      textPoints[0] = new TexCoord2f(0.5f, 0.5f);
      textPoints[1] = new TexCoord2f(0.75f, 0.5f);
      textPoints[2] = new TexCoord2f(0.75f, 0.75f);
      textPoints[3] = new TexCoord2f(0.5f, 0.75f);
      textPoints[4] = new TexCoord2f(0.5f, 0.5f);
      textPoints[5] = new TexCoord2f(0.75f, 0.5f);
      textPoints[6] = new TexCoord2f(0.75f, 0.75f);
      textPoints[7] = new TexCoord2f(0.5f, 0.75f);
      
      textPoints[8] = new TexCoord2f(0.675f, 0.675f);
      textPoints[9] = new TexCoord2f(0.675f, 0.675f);

      int[] polygonIndices = new int[4 * 4 + 8 * 3];
      int index = 0;

      for (int i = 0; i <= 3; i++)
      {
         polygonIndices[index] =     i;
         polygonIndices[index + 1] = (i + 1) % 4;
         polygonIndices[index + 2] = 4 + (i + 1) % 4;
         polygonIndices[index + 3] = 4 + i;

         index = index + 4;
      }

      for (int i = 0; i <= 3; i++)
      {
         polygonIndices[index] = (i + 1) % 4;
         polygonIndices[index + 1] = i;
         polygonIndices[index + 2] = 9;

         index = index + 3;
      }

      for (int i = 0; i <= 3; i++)
      {
         polygonIndices[index] =     4 + i;
         polygonIndices[index + 1] = 4 + (i + 1) % 4;
         polygonIndices[index + 2] = 8;

         index = index + 3;
      }

      int[] pStripCounts = new int[12];

      for (int i = 0; i < 4; i++)
      {
         pStripCounts[i] = 4;
      }

      for (int i = 4; i < 12; i++)
      {
         pStripCounts[i] = 3;
      }

      return new MeshDataHolder(points, textPoints, polygonIndices, pStripCounts);
   }

   //TODO: Figure out what a Gridded Polytope is and figure out how to draw them.
//   public static MeshDataHolder griddedPolytope(Point3f[][] griddedPoints, double x_tiles, double y_tiles)
//   {
//      int firstSize = griddedPoints.length;
//      int secondSize = griddedPoints[0].length;
//
//      int totalN = firstSize * 2 * secondSize;
//
//      Point3f[] coords = new Point3f[totalN];
//
//      
//      int[] stripCounts = new int[firstSize];
//      TexCoord2f[] textPoints = new TexCoord2f[totalN];
//
//      int index = 0;
//      for (int i = 0; i < firstSize - 1; i++)
//      {
//         for (int j = 0; j < secondSize; j++)
//         {
//            coords[index] = new Point3f(griddedPoints[i + 1][j]);
//            textPoints[index] = new TexCoord2f((float) (x_tiles * ((float) i + 1) / (firstSize)),
//                                               (float) (y_tiles * (j) / (secondSize)));
//            
//            index++;
//
//            coords[index] = new Point3f(griddedPoints[i][j]);
//            textPoints[index] = new TexCoord2f((float) (x_tiles * (i) / (firstSize)), (float) (y_tiles * (j) / (secondSize)));
//            index++;
//         }
//      }
//      
//      for (int k = 0; k < secondSize; k++)
//      {
////       stripCounts[k] = 2*(xPointsPerSide+1);
//         stripCounts[k] = 2 * firstSize;
//      }
//
//      int[] polygonIndices = new int[coords.length];
//      for(int l = 0; l < firstSize; l++)
//      {
//         // TODO: Fill polygonIndices.
//      }
//      
//      return new MeshDataHolder(coords, textPoints, polygonIndices, stripCounts);
//   }
//   
   
   private static TexCoord2f[] generateInterpolatedTexturePoints(int numPoints)
   {
      TexCoord2f[] textPoints = new TexCoord2f[numPoints];
      
      double distanceBetweenPoints = 4.0/(numPoints); 
      float[] xSides = {0.0f, 0.0f, 1.0f, 1.0f};
      float[] ySides = {0.0f, 1.0f, 1.0f, 0.0f};
      float positionAlongPerimeter;
      float positionAlongSide;
      int side;
      int secondPoint;
      float texCoordX, texCoordY;
      for (int i = 0; i < textPoints.length; i++)
      {
         positionAlongPerimeter = (float) distanceBetweenPoints * i;
         positionAlongSide = (float) (positionAlongPerimeter - Math.floor(positionAlongPerimeter));
         side = ((int) Math.floor(positionAlongPerimeter)) % 4;
         secondPoint = (side + 1) % 4; 

         texCoordX = positionAlongSide * xSides[secondPoint] + (1.0f - positionAlongSide) * xSides[side];
         texCoordY = positionAlongSide * ySides[secondPoint] + (1.0f - positionAlongSide) * ySides[side];
         
         textPoints[i] = new TexCoord2f(texCoordX, texCoordY);
      }
      return textPoints;
   }
   
   private static Point3f[] makePoint3fArrayFromPoint3dArray(Point3d[] pPoints)
   {
      Point3f[] points3f = new Point3f[pPoints.length];
      int i = 0;
      for (Point3d point3d : pPoints)
      {
         points3f[i++] = new Point3f(point3d);
      }
      return points3f;
   }
}
