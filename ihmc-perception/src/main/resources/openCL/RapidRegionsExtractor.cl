#define FILTER_DISPARITY_THRESHOLD 0
#define MERGE_ANGULAR_THRESHOLD 1
#define MERGE_ORTHOGONAL_THRESHOLD 2
#define PATCH_HEIGHT 3
#define PATCH_WIDTH 4
#define SUB_H 5
#define SUB_W 6
#define DEPTH_FX 7
#define DEPTH_FY 8
#define DEPTH_CX 9
#define DEPTH_CY 10
#define FILTER_KERNEL_SIZE 11
#define FILTER_SUB_H 12
#define FILTER_SUB_W 13
#define INPUT_HEIGHT 14
#define INPUT_WIDTH 15
#define NORMAL_PACK_RANGE 16
#define CENTROID_PACK_RANGE 17
#define MERGE_RANGE 18
#define MERGE_DISTANCE_THRESHOLD 19
#define EXTRACTION_MODE 20

bool check_convergence(float3 va, float3 vb)
{
   float alen = length(va);
   float blen = length(vb);
   float r = min(alen, blen);

   float theta = acos(dot(va, vb) / (length(va) * length(vb)));
   float alpha = M_PI / 4;
   float distThreshold = r * sin(theta) / cos(theta / 2 + alpha);

   float3 diff = va - vb;
   float dist = length(diff);
   if (dist < distThreshold)
      return true;
   else
      return false;
}

float4 back_project_spherical(int2 pos, float depth, global float* params)
{
   float totalPitch = M_PI / 2;
   float totalYaw = 2 * M_PI;

   int x = pos.x;
   int y = pos.y;

   int xFromCenter = -x - (params[INPUT_WIDTH] / 2);
   int yFromCenter = -(y - (params[INPUT_HEIGHT] / 2));

   float yaw = xFromCenter / (float) params[INPUT_WIDTH] * totalYaw;
   float pitch = yFromCenter / (float) params[INPUT_HEIGHT] * totalPitch;

   float r = depth * cos(pitch);

   float px = r * cos(yaw);
   float py = r * sin(yaw);
   float pz = depth * sin(pitch);

   // printf("Projection: [%d,%d] (X:%.2lf,Y:%.2lf,Z:%.2lf,R:%.2lf,D:%.2lf)\n", x, y, px, py, pz, r, depth);

   float4 X = (float4) (px, py, pz, 0);
   return X;
}

float4 back_project_perspective(int2 pos, float Z, global float* params)
{
   float X = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
   float Y = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;

   float4 point = (float4) (Z, -X, -Y, 0);
   return point;
}

float3 estimate_perspective_normal(read_only image2d_t in, int rIndex, int cIndex, global float* params)
{
   float residual = 0;
   float radius = 0;
   int m = (int) params[NORMAL_PACK_RANGE];
   int count = 0;
   float4 normal = (float4) (0, 0, 0, 0);

   if (rIndex >= m && cIndex >= m && (rIndex * ((int) params[PATCH_HEIGHT]) + 2 * m) < ((int) params[INPUT_HEIGHT]) &&
       (cIndex * ((int) params[PATCH_WIDTH]) + 2 * m) < ((int) params[INPUT_WIDTH]))
   {
      for (int i = 0; i < (int) m; i++)
      {
         for (int j = 0; j < (int) m; j++)
         {
            count++;
            int grIndex = rIndex * (int) params[PATCH_HEIGHT] + i;
            int gcIndex = cIndex * (int) params[PATCH_WIDTH] + j;
            int2 pos = (int2) (gcIndex, grIndex);

            pos = (int2) (gcIndex, grIndex);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 va = back_project_perspective(pos, radius, params);

            pos = (int2) (gcIndex + m, grIndex);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vb = back_project_perspective(pos, radius, params);

            pos = (int2) (gcIndex + m, grIndex + m);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vc = back_project_perspective(pos, radius, params);

            pos = (int2) (gcIndex, grIndex + m);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vd = back_project_perspective(pos, radius, params);

            normal += cross((vc - vb), (vb - va));
            normal += cross((vd - vc), (vc - vb));
            normal += cross((va - vd), (vd - vc));
            normal += cross((vb - va), (va - vd));
         }
      }
   }
   return normalize((1 / (float) (count)) * normal.xyz);
}

float3 estimate_spherical_normal(read_only image2d_t in, int rIndex, int cIndex, global float* params)
{
   float residual = 0;
   float radius = 0;
   int m = (int) params[NORMAL_PACK_RANGE];
   int count = 0;
   float4 normal = (float4) (0, 0, 0, 0);

   if (rIndex >= m && cIndex >= m && (rIndex * ((int) params[PATCH_HEIGHT]) + 2 * m) < ((int) params[INPUT_HEIGHT]) &&
       (cIndex * ((int) params[PATCH_WIDTH]) + 2 * m) < ((int) params[INPUT_WIDTH]))
   {
      for (int i = 0; i < (int) m; i++)
      {
         for (int j = 0; j < (int) m; j++)
         {
            count++;
            int grIndex = rIndex * (int) params[PATCH_HEIGHT] + i;
            int gcIndex = cIndex * (int) params[PATCH_WIDTH] + j;
            int2 pos = (int2) (gcIndex, grIndex);

            pos = (int2) (gcIndex, grIndex);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 va = back_project_spherical(pos, radius, params);

            pos = (int2) (gcIndex + m, grIndex);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vb = back_project_spherical(pos, radius, params);

            pos = (int2) (gcIndex + m, grIndex + m);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vc = back_project_spherical(pos, radius, params);

            pos = (int2) (gcIndex, grIndex + m);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vd = back_project_spherical(pos, radius, params);

            normal += cross((vc - vb), (vb - va));
            normal += cross((vd - vc), (vc - vb));
            normal += cross((va - vd), (vd - vc));
            normal += cross((vb - va), (va - vd));
         }
      }
   }
   return normalize((1 / (float) (count)) * normal.xyz);
}

float3 estimate_perspective_centroid(read_only image2d_t in, int y, int x, global float* params)
{
   float Z = 0;
   int count = 0;
   float3 centroid = (float3) (0, 0, 0);
   if (y >= 0 && y < (int) params[SUB_H] && x >= 0 && x < (int) params[SUB_W])
   {
      for (int i = 0; i < (int) params[PATCH_HEIGHT]; i++)
      {
         for (int j = 0; j < (int) params[PATCH_WIDTH]; j++)
         {
            count++;
            int gx = x * (int) params[PATCH_HEIGHT] + i;
            int gy = y * (int) params[PATCH_WIDTH] + j;
            int2 pos = (int2) (gx, gy);
            Z = ((float) read_imageui(in, pos).x) / (float) 1000;
            if (Z > 0.1f)
            {
               float4 P = back_project_perspective(pos, Z, params);
               centroid += P.xyz;
            }
         }
      }
   }
   return (1 / (float) (count)) * centroid;
}

float3 estimate_spherical_centroid(read_only image2d_t in, int rIndex, int cIndex, global float* params)
{
   float radius = 0;
   int count = 0;
   float3 centroid = (float3) (0, 0, 0);
   // if(rIndex >= 0 && rIndex < (int)params[SUB_H] && cIndex >= 0 && cIndex < (int)params[SUB_W])
   {
      for (int i = 0; i < (int) params[PATCH_HEIGHT]; i++)
      {
         for (int j = 0; j < (int) params[PATCH_WIDTH]; j++)
         {
            count++;
            int grIndex = rIndex * (int) params[PATCH_HEIGHT] + i;
            int gcIndex = cIndex * (int) params[PATCH_WIDTH] + j;
            int2 pos = (int2) (gcIndex, grIndex);
            radius = ((float) read_imageui(in, pos).x) / (float) 1000;
            if (radius > 0.1f)
            {
               float4 P = back_project_spherical(pos, radius, params);
               centroid += P.xyz;
            }
         }
      }
   }
   return (1 / (float) (count)) * centroid;
}

bool isConnected(float3 ag, float3 an, float3 bg, float3 bn, global float* params)
{

   float lenA = length(ag);
   float lenB = length(bg);

   float3 vMin = (lenB < lenA) ? bg : ag;
   float3 vMax = (lenB < lenA) ? ag : bg;

   float3 vec = vMax - vMin;

   float distanceToNearPoint = min(lenA, lenB);
   float distanceToFarPoint = max(lenA, lenB);

   float dist = length(vec);
   float sim = fabs(dot(an, bn));

   float perpDist = fabs(dot(ag - bg, bn)) + fabs(dot(bg - ag, an));

   float phi = acos(fabs(dot(vMin, vec) / (length(vMin) * length(vec))));
   float theta = acos(dot(ag, bg) / (length(ag) * length(bg)));

   float distThreshold = distanceToFarPoint * fabs(sin(theta) / sin(phi));

   if (perpDist < params[MERGE_ORTHOGONAL_THRESHOLD] && (dist < distThreshold * params[MERGE_DISTANCE_THRESHOLD]) && sim > params[MERGE_ANGULAR_THRESHOLD] &&
       distanceToNearPoint > 0.8f)
   {
      return true;
   }
   else
   {
      return false;
   }
}

/*
 * Replace non-reading (0s) pixels with an approximate nearby value, so they don't
 * upset the rest of the algorithm.
 */
void fillDeadPixels(read_only image2d_t inputImage, int x, int y, write_only image2d_t out0, global float* params)
{
   uint Z = 0;
   int count = 0;
   int xs[8], ys[8], values[8];
   int total_unique = 0;

   /* Find all non-zero unique depth values available in this patch. */
   for (int i = 0; i < (int) params[FILTER_KERNEL_SIZE]; i++)
   {
      for (int j = 0; j < (int) params[FILTER_KERNEL_SIZE]; j++)
      {
         count++;
         int gx = x * ((int) params[FILTER_KERNEL_SIZE]) + i;
         int gy = y * ((int) params[FILTER_KERNEL_SIZE]) + j;

         int2 pos = (int2) (gx, gy);
         Z = read_imageui(inputImage, pos).x;

         /* For every unique non-zero depth value, insert x,y,z into xs,ys and values, respectively. */
         if (Z != 0)
         {
            /* For the first non-zero depth, simply insert.*/
            if (total_unique == 0)
            {
               xs[total_unique] = gx;
               ys[total_unique] = gy;
               values[total_unique] = Z;
               total_unique++;
            }
            else
            {
               /* For all other non-zero values, check against all previously visited non-zero unique depth values. */
               int unique = 0;
               for (int k = 0; k < total_unique; k++)
               {
                  if ((Z - values[k]) * (Z - values[k]) > params[FILTER_DISPARITY_THRESHOLD])
                  {
                     // if (x==0 && y==0)printf("Compare(%d,%d,%d)(%d,%d)\n", Z, abs(Z - values[k]), values[k], unique, total_unique);
                     unique++;
                  }
               }
               /* After checking to see if the non-zero value is truly unique against stored unique values, insert.*/
               if (unique == total_unique)
               {
                  // if (x==0 && y==0)printf("Insert(%d,%d)(%d,%d)\n", gx, gy, unique, total_unique);
                  xs[total_unique] = gx;
                  ys[total_unique] = gy;
                  values[total_unique] = Z;
                  total_unique++;
               }
            }
         }
      }
   }

   /* Fill all dead pixels in the patch with the nearest neighboring non-zero unique depth value. */
   for (int i = 0; i < (int) params[FILTER_KERNEL_SIZE]; i++)
   {
      for (int j = 0; j < (int) params[FILTER_KERNEL_SIZE]; j++)
      {
         count++;
         int gx = x * (int) params[FILTER_KERNEL_SIZE] + i;
         int gy = y * (int) params[FILTER_KERNEL_SIZE] + j;

         int2 pos = (int2) (gx, gy);
         Z = read_imageui(inputImage, pos).x;

         if (Z == 0)
         {
            float minDist = 10000000;
            float curDist = 0;
            int minIndex = 0;

            /* Find the nearest non-zero-unique neighbor for this dead pixel. */
            for (int m = 0; m < total_unique; m++)
            {
               curDist = length((float2) (gx, gy) - (float2) (xs[m], ys[m]));
               // if (x==0 && y==0) printf("Filling(%d,%d)-(%d,%d):%.2lf\n", gx,gy,xs[m],ys[m],curDist);
               if (curDist < minDist)
               {
                  minDist = curDist;
                  minIndex = m;
               }
            }
            Z = values[minIndex];
            // if (x==0 && y==0) printf("%d\t",Z);
         }

         write_imageui(out0, pos, (uint4) (Z, 0, 0, 0));
      }
   }
}

void smooth_non_boundary(read_only image2d_t in, int x, int y, write_only image2d_t out0, global float* params)
{
   uint Z = 0;
   int m = 5;
   int left = 0;
   int right = 0;
   for (int i = 0; i < (int) params[FILTER_KERNEL_SIZE]; i++)
   {
      for (int j = 0; j < (int) params[FILTER_KERNEL_SIZE]; j++)
      {
         int gx = x * ((int) params[FILTER_KERNEL_SIZE]) + i;
         int gy = y * ((int) params[FILTER_KERNEL_SIZE]) + j;

         if (gx - m >= 0 && gx + m < params[INPUT_HEIGHT])
         {
            int2 pos = (int2) (gx, gy);
            Z = read_imageui(in, pos).x;

            pos = (int2) (gx - m, gy);
            left = read_imageui(in, pos).x;

            pos = (int2) (gx + m, gy);
            right = read_imageui(in, pos).x;

            if (abs((Z - left) - (right - Z)) < 50)
            {
               Z = (left + right) / 2;
               write_imageui(out0, pos, (uint4) (Z, 0, 0, 0));
            }
         }
      }
   }
}

float4 transform4(float4 point, float4 r1, float4 r2, float4 r3, float4 t)
{
   return (float4) (dot(r1, point) + t.x, dot(r2, point) + t.y, dot(r3, point) + t.z, 0);
}

float3 calculateNormal(float3 p1, float3 p2, float3 p3)
{
   float3 v12 = p2 - p1;
   float3 v23 = p3 - p2;
   return normalize(cross(v12, v23));
}

/*
 * Filters all pixels within a patch on depth map. Removes outliers, flying points, dead pixels and measurement
 * noise.
 */
void kernel filterKernel(read_only image2d_t inputImage, write_only image2d_t filteredImage, write_only image2d_t nxImage, global float* parameters)
{
   int y = get_global_id(0);
   int x = get_global_id(1);
   int filterPatchImageHeight = parameters[12];
   int filterPatchImageWidth = parameters[13];

   if (y >= 0 && y < filterPatchImageHeight && x >= 0 && x < filterPatchImageWidth)
   {
      fillDeadPixels(inputImage, x, y, filteredImage, parameters);
      // mark_boundary_patches(inputImage, x, y, nxImage, parameters);
      // smooth_non_boundary(inputImage, x, y, filteredImage, parameters);
   }
}

/* Pack Kernel: Generates the patches by packing the centroid, surface normal and metadata related to
 * patches on a sub-sampled grid of the depth map. The following intrinsic parameters are used to convert to Point Cloud.
 * K: [459.97265625, 0.0, 341.83984375, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 1.0]
 * */
void kernel packKernel(read_only image2d_t in,
                       write_only image2d_t out0,
                       write_only image2d_t out1,
                       write_only image2d_t out2, /* float3 maps for normal*/
                       write_only image2d_t out3,
                       write_only image2d_t out4,
                       write_only image2d_t out5, /* float3 maps for centroids */
                       global float* params
                       // write_only image2d_t debug

)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   float3 normal;
   float3 centroid;

   //    printf("[r:%d,c:%d] -> PackKernel:(%d,%d,%d,%d,%d,%d,%d,%.2lf,%.2lf)\n", rIndex, cIndex,
   //                            (int)params[INPUT_HEIGHT],
   //                            (int)params[INPUT_WIDTH],
   //                            (int)params[SUB_H],
   //                            (int)params[SUB_W],
   //                            (int)params[PATCH_HEIGHT],
   //                            (int)params[PATCH_WIDTH],
   //                            (int)params[FILTER_DISPARITY_THRESHOLD],
   //                            params[MERGE_ANGULAR_THRESHOLD],
   //                            params[MERGE_ORTHOGONAL_THRESHOLD]);

   // if(cIndex >= 0 && cIndex < (int)params[SUB_H] && rIndex >= 0 && rIndex < (int)params[SUB_W])
   {
      if (params[EXTRACTION_MODE] > 0.5f)
      {
         normal = estimate_spherical_normal(in, rIndex, cIndex, params);
         centroid = estimate_spherical_centroid(in, rIndex, cIndex, params);
      }
      else
      {
         normal = estimate_perspective_normal(in, rIndex, cIndex, params);
         centroid = estimate_perspective_centroid(in, rIndex, cIndex, params);
      }

      write_imagef(out0, (int2) (cIndex, rIndex), (float4) (normal.x, 0, 0, 0));
      write_imagef(out1, (int2) (cIndex, rIndex), (float4) (normal.y, 0, 0, 0));
      write_imagef(out2, (int2) (cIndex, rIndex), (float4) (normal.z, 0, 0, 0));
      write_imagef(out3, (int2) (cIndex, rIndex), (float4) (centroid.x, 0, 0, 0));
      write_imagef(out4, (int2) (cIndex, rIndex), (float4) (centroid.y, 0, 0, 0));
      write_imagef(out5, (int2) (cIndex, rIndex), (float4) (centroid.z, 0, 0, 0));

      //      printf("PackKernel[%d,%d]\t Centroid:(%.4lf, %.4lf, %.4lf)\t Normal:(%.4lf,%.4lf,%.4lf)\n", rIndex, cIndex, centroid.x, centroid.y, centroid.z,
      //      normal.x, normal.y, normal.z);
   }
}

/* Merge Kernel: Creates the graph-based structure by adding connections between the neighboring
 * patches based on similarity.
 */
void kernel mergeKernel(read_only image2d_t in,
                        read_only image2d_t out0,
                        read_only image2d_t out1,
                        read_only image2d_t out2, /* float3 maps for normal*/
                        read_only image2d_t out3,
                        read_only image2d_t out4,
                        read_only image2d_t out5,  /* float3 maps for centroids */
                        write_only image2d_t out6, /* uint8 map for patch metadata*/
                        global float* params       /* All parameters */
)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   int m = (int) params[MERGE_RANGE];

   //    if(cIndex == 0 && rIndex == 0) printf("MergeKernel:(%d)\n", params[MERGE_RANGE]);

   if (rIndex >= m && rIndex < (int) params[SUB_H] - m && cIndex >= m && cIndex < (int) params[SUB_W] - m)
   {
      float n1_a = read_imagef(out0, (int2) (cIndex, rIndex)).x;
      float n2_a = read_imagef(out1, (int2) (cIndex, rIndex)).x;
      float n3_a = read_imagef(out2, (int2) (cIndex, rIndex)).x;
      float g1_a = read_imagef(out3, (int2) (cIndex, rIndex)).x;
      float g2_a = read_imagef(out4, (int2) (cIndex, rIndex)).x;
      float g3_a = read_imagef(out5, (int2) (cIndex, rIndex)).x;

      float3 g_a = (float3) (g1_a, g2_a, g3_a);
      float3 n_a = (float3) (n1_a, n2_a, n3_a);

      uint boundaryConnectionsEncodedAsOnes = (uint) (0);

      int count = 0;
      for (int i = -m; i < m + 1; i += m)
      {
         for (int j = -m; j < m + 1; j += m)
         {
            if (!(j == 0 && i == 0))
            {
               float n1_b = read_imagef(out0, (int2) (cIndex + j, rIndex + i)).x;
               float n2_b = read_imagef(out1, (int2) (cIndex + j, rIndex + i)).x;
               float n3_b = read_imagef(out2, (int2) (cIndex + j, rIndex + i)).x;
               float g1_b = read_imagef(out3, (int2) (cIndex + j, rIndex + i)).x;
               float g2_b = read_imagef(out4, (int2) (cIndex + j, rIndex + i)).x;
               float g3_b = read_imagef(out5, (int2) (cIndex + j, rIndex + i)).x;

               float3 g_b = (float3) (g1_b, g2_b, g3_b);
               float3 n_b = (float3) (n1_b, n2_b, n3_b);

               if (isConnected(g_a, normalize(n_a), g_b, normalize(n_b), params))
               {
                  boundaryConnectionsEncodedAsOnes = (1 << count) | boundaryConnectionsEncodedAsOnes;
               }
               count++;
            }
         }
      }
      write_imageui(out6, (int2) (cIndex, rIndex), (uint4) (boundaryConnectionsEncodedAsOnes, 0, 0, 0));

      //      printf("MergeKernel[%d,%d] -> (%d)\n", rIndex, cIndex, boundaryConnectionsEncodedAsOnes);
   }
}

/*
 * Perspective Back-Projection Kernel for Structured Light And Perspective Time-of-Flight Sensors
 * */
void kernel perspectiveBackProjectionKernel(read_only image2d_t in, global float* cloud, global float* params)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   // if(rIndex >= 0 && rIndex < (int)params[INPUT_HEIGHT] && cIndex >= 0 && cIndex < (int)params[INPUT_WIDTH])
   {
      int2 pos = (int2) (cIndex, rIndex);

      float scaleToMeters = 0.001f;
      float radius = ((float) read_imageui(in, pos).x) * scaleToMeters;

      if (radius > 0.3f)
      {
         float4 point = back_project_perspective(pos, radius, params);

         int index = ((rIndex * params[INPUT_WIDTH]) + cIndex) * 3;

         if (length(point.xyz) > 1.5f)
         {
            cloud[index] = point.x;
            cloud[index + 1] = point.y;
            cloud[index + 2] = point.z;
         }
         else
         {
            cloud[index] = 0;
            cloud[index + 1] = 0;
            cloud[index + 2] = 0;
         }

         // printf("[%d] Spherical(%d,%d):\t Radius: %.3lf, Point:(%.4lf, %.4lf, %.4lf)\n", index, rIndex, cIndex, radius, cloud[index], cloud[index+1],
         // cloud[index+2]);
      }
   }
}

/*
 * Spherical Back-Projection Kernel for Rotating LIDARs.
 * */
void kernel sphericalBackProjectionKernel(read_only image2d_t in, global float* cloud, global float* params)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   // if(rIndex >= 0 && rIndex < (int)params[INPUT_HEIGHT] && cIndex >= 0 && cIndex < (int)params[INPUT_WIDTH])
   {
      int2 pos = (int2) (cIndex, rIndex);

      float scaleToMeters = 0.001f;
      float radius = ((float) read_imageui(in, pos).x) * scaleToMeters;

      if (radius > 0.1f)
      {
         float4 point = back_project_spherical(pos, radius, params);

         int index = ((rIndex * params[INPUT_WIDTH]) + cIndex) * 3;

         cloud[index] = point.x;
         cloud[index + 1] = point.y;
         cloud[index + 2] = point.z;

         // printf("[%d] Spherical(%d,%d):\t Radius: %.3lf, Point:(%.4lf, %.4lf, %.4lf)\n", index, rIndex, cIndex, radius, cloud[index], cloud[index+1],
         // cloud[index+2]);
      }
   }
}

/*
 * Copy kernel to move feature grid map into cache buffer
 */
void kernel copyKernel(read_only image2d_t in0,
                       read_only image2d_t in1,
                       read_only image2d_t in2,
                       read_only image2d_t in3,
                       read_only image2d_t in4,
                       read_only image2d_t in5,
                       write_only image2d_t out0,
                       write_only image2d_t out1,
                       write_only image2d_t out2,
                       write_only image2d_t out3,
                       write_only image2d_t out4,
                       write_only image2d_t out5,
                       global float* params)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   float n1 = read_imagef(in0, (int2) (cIndex, rIndex)).x;
   float n2 = read_imagef(in1, (int2) (cIndex, rIndex)).x;
   float n3 = read_imagef(in2, (int2) (cIndex, rIndex)).x;
   float g1 = read_imagef(in3, (int2) (cIndex, rIndex)).x;
   float g2 = read_imagef(in4, (int2) (cIndex, rIndex)).x;
   float g3 = read_imagef(in5, (int2) (cIndex, rIndex)).x;

   write_imagef(out0, (int2) (cIndex, rIndex), (float4) (n1, 0, 0, 0));
   write_imagef(out1, (int2) (cIndex, rIndex), (float4) (n2, 0, 0, 0));
   write_imagef(out2, (int2) (cIndex, rIndex), (float4) (n3, 0, 0, 0));
   write_imagef(out3, (int2) (cIndex, rIndex), (float4) (g1, 0, 0, 0));
   write_imagef(out4, (int2) (cIndex, rIndex), (float4) (g2, 0, 0, 0));
   write_imagef(out5, (int2) (cIndex, rIndex), (float4) (g3, 0, 0, 0));

   //      printf("CopyKernel[%d,%d] -> (%.3lf, %.3lf, %.3lf) (%.3lf, %.3lf, %.3lf)\n", rIndex, cIndex, n1, n2, n3, g1, g2, g3);
}

/*
 * Correspondence Kernel for Iterative Closest Point
 */
void kernel correspondenceKernel(read_only image2d_t one0,
                                 read_only image2d_t one1,
                                 read_only image2d_t one2,
                                 read_only image2d_t one3,
                                 read_only image2d_t one4,
                                 read_only image2d_t one5,
                                 read_only image2d_t two0,
                                 read_only image2d_t two1,
                                 read_only image2d_t two2,
                                 read_only image2d_t two3,
                                 read_only image2d_t two4,
                                 read_only image2d_t two5,
                                 write_only image2d_t matchRow,
                                 write_only image2d_t matchColumn,
                                 global float* params)
{
   int cIndex = get_global_id(0);
   int rIndex = get_global_id(1);

   int totalRows = params[0];
   int totalColumns = params[1];

   float4 pointOne = (float4) (0, 0, 0, 0);
   float4 pointTwo = (float4) (0, 0, 0, 0);
   float4 normalOne = (float4) (0, 0, 0, 0);
   float4 normalTwo = (float4) (0, 0, 0, 0);

   //    if(rIndex == 0 && cIndex == 0) printf("Correspondence Kernel\n");

   float minLength = 10000000;
   float distance = 0;

   uint minRowIndex = 0;
   uint minColumnIndex = 0;

   float nx1 = read_imagef(one0, (int2) (cIndex, rIndex)).x;
   float ny1 = read_imagef(one1, (int2) (cIndex, rIndex)).x;
   float nz1 = read_imagef(one2, (int2) (cIndex, rIndex)).x;
   float gx1 = read_imagef(one3, (int2) (cIndex, rIndex)).x;
   float gy1 = read_imagef(one4, (int2) (cIndex, rIndex)).x;
   float gz1 = read_imagef(one5, (int2) (cIndex, rIndex)).x;

   pointOne = (float4) (gx1, gy1, gz1, 0);
   normalOne = (float4) (nx1, ny1, nz1, 0);

   int rowOne = rIndex;
   int columnOne = cIndex;

   float radius = length(pointOne.xyz);
   if (radius < 0.3f || radius > 100.0f)
   {
      write_imageui(matchRow, (int2) (cIndex, rIndex), (uint4) (0, 0, 0, 0));
      write_imageui(matchColumn, (int2) (cIndex, rIndex), (uint4) (0, 0, 0, 0));
      return;
   }

   //    printf("CorrespondenceKernel: (%d,%d) -> (%.2lf, %.2lf, %.2lf)\n", rIndex, cIndex, pointOne.x, pointOne.y, pointOne.z);

   for (int i = 0; i < 4; i++)
   {
      for (int j = 0; j < 4; j++)
      {
         int rowTwo = rIndex + i;
         int columnTwo = cIndex + j;

         if (rowTwo >= 0 && rowTwo < totalRows && columnTwo >= 0 && columnTwo < totalColumns)
         {
            float nx2 = read_imagef(two0, (int2) (columnTwo, rowTwo)).x;
            float ny2 = read_imagef(two1, (int2) (columnTwo, rowTwo)).x;
            float nz2 = read_imagef(two2, (int2) (columnTwo, rowTwo)).x;
            float gx2 = read_imagef(two3, (int2) (columnTwo, rowTwo)).x;
            float gy2 = read_imagef(two4, (int2) (columnTwo, rowTwo)).x;
            float gz2 = read_imagef(two5, (int2) (columnTwo, rowTwo)).x;

            pointTwo = (float4) (gx2, gy2, gz2, 0);
            normalTwo = (float4) (nx2, ny2, nz2, 0);

            float radiusTwo = length(pointTwo.xyz);
            if (radiusTwo < 0.3f || radiusTwo > 100.0f)
            {
               continue;
            }

            // pointTwo = (float4)(cloudTwo[j*3+0], cloudTwo[j*3+1], cloudTwo[j*3+2], 0);
            // pointTwo = transform(pointTwo, (float4)(transformTwo[0], transformTwo[1], transformTwo[2], 0),
            //(float4)(transformTwo[3], transformTwo[4], transformTwo[5], 0),
            //(float4)(transformTwo[6], transformTwo[7], transformTwo[8], 0),
            //(float4)(transformTwo[9], transformTwo[10], transformTwo[11], 0));

            distance = length(pointTwo - pointOne);

            if (distance < minLength)
            {
               //                    printf("Match: (%.2lf, %.2lf, %.2lf)[%.2lf] : (%.2lf, %.2lf, %.2lf) -> [%.2lf]\n", pointOne.x, pointOne.y, pointOne.z,
               //                    radiusTwo,
               //                                                                pointTwo.x, pointTwo.y, pointTwo.z, distance);
               minRowIndex = (uint) (rowTwo);
               minColumnIndex = (uint) (columnTwo);
               minLength = distance;
            }
         }
      }
   }

   //    if(minColumnIndex != 0 && minRowIndex != 0)
   //    {
   //        printf("Match: (%d, %d) -> [%d, %d] : {%.2lf}\n", rowOne, columnOne, minRowIndex, minColumnIndex, minLength);
   //    }

   write_imageui(matchRow, (int2) (cIndex, rIndex), (uint4) (minRowIndex, 0, 0, 0));
   write_imageui(matchColumn, (int2) (cIndex, rIndex), (uint4) (minColumnIndex, 0, 0, 0));
}

/*
 * Centroid Calculation Kernel for Iterative Closest Point
 * */
void kernel centroidReduceKernel(read_only image2d_t one0,
                                 read_only image2d_t one1,
                                 read_only image2d_t one2,
                                 read_only image2d_t one3,
                                 read_only image2d_t one4,
                                 read_only image2d_t one5,
                                 read_only image2d_t two0,
                                 read_only image2d_t two1,
                                 read_only image2d_t two2,
                                 read_only image2d_t two3,
                                 read_only image2d_t two4,
                                 read_only image2d_t two5,
                                 read_only image2d_t matchRowImage,
                                 read_only image2d_t matchColumnImage,
                                 global float* mean,
                                 global float* params)
{
   int cIndex = get_global_id(0);

   float4 pointOne = (float4) (0, 0, 0, 0);
   float4 pointTwo = (float4) (0, 0, 0, 0);
   float columnMeanVec[6];
   int countOne = 0;
   int countTwo = 0;

   for (int k = 0; k < 6; k++)
   {
      columnMeanVec[k] = 0;
      mean[cIndex * 6 + k] = 0;
   }

   for (int rIndex = 0; rIndex < params[0]; rIndex++)
   {
      //        if(rIndex == 0 && cIndex == 0) printf("Centroid Reduce Kernel\n");

      int2 pos = (int2) (cIndex, rIndex);

      uint matchRow = (uint) read_imageui(matchRowImage, pos).x;
      uint matchColumn = (uint) read_imageui(matchColumnImage, pos).x;
      int2 matchPos = (int2) (matchColumn, matchRow);

      if (matchRow != 0 && matchColumn != 0)
      {
         countOne += 1;
         float cx1 = read_imagef(one3, pos).x;
         float cy1 = read_imagef(one4, pos).x;
         float cz1 = read_imagef(one5, pos).x;
         pointOne = (float4) (cx1, cy1, cz1, 0);
         columnMeanVec[0] += pointOne.x;
         columnMeanVec[1] += pointOne.y;
         columnMeanVec[2] += pointOne.z;

         countTwo += 1;
         float cx2 = read_imagef(two3, matchPos).x;
         float cy2 = read_imagef(two4, matchPos).x;
         float cz2 = read_imagef(two5, matchPos).x;
         pointTwo = (float4) (cx2, cy2, cz2, 0);
         columnMeanVec[3] += pointTwo.x;
         columnMeanVec[4] += pointTwo.y;
         columnMeanVec[5] += pointTwo.z;
      }
   }

   mean[cIndex * 6 + 0] = (float) columnMeanVec[0] / (float) countOne;
   mean[cIndex * 6 + 1] = (float) columnMeanVec[1] / (float) countOne;
   mean[cIndex * 6 + 2] = (float) columnMeanVec[2] / (float) countOne;

   mean[cIndex * 6 + 3] = (float) columnMeanVec[3] / (float) countTwo;
   mean[cIndex * 6 + 4] = (float) columnMeanVec[4] / (float) countTwo;
   mean[cIndex * 6 + 5] = (float) columnMeanVec[5] / (float) countTwo;

   //   printf("Mean(%d) Count(%d): (%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf)\n", cIndex, count,
   //          mean[cIndex*6], mean[cIndex*6 + 1], mean[cIndex*6 + 2], mean[cIndex*6 + 3], mean[cIndex*6 + 4], mean[cIndex*6 + 5]);
}

/*
 * ICP Kernel for Iterative Closest Point
 * */
void kernel correlReduceKernel(read_only image2d_t one0,
                               read_only image2d_t one1,
                               read_only image2d_t one2,
                               read_only image2d_t one3,
                               read_only image2d_t one4,
                               read_only image2d_t one5,
                               read_only image2d_t two0,
                               read_only image2d_t two1,
                               read_only image2d_t two2,
                               read_only image2d_t two3,
                               read_only image2d_t two4,
                               read_only image2d_t two5,
                               read_only image2d_t matchRowImage,
                               read_only image2d_t matchColumnImage,
                               global float* correlation,
                               global float* params)
{
   int cIndex = get_global_id(0);

   float correl[9];
   float4 pointOne = (float4) (0, 0, 0, 0);
   float4 pointTwo = (float4) (0, 0, 0, 0);
   float columnMeanVec[6];
   int countOne = 0;
   int countTwo = 0;

   for (int k = 0; k < 9; k++)
   {
      correl[k] = 0;
      correlation[cIndex * 9 + k] = 0;
   }

   for (int rIndex = 0; rIndex < params[0]; rIndex++)
   {
      //        if(rIndex == 0 && cIndex == 0) printf("Correlation Reduce Kernel\n");

      int2 pos = (int2) (cIndex, rIndex);

      uint matchRow = (uint) read_imageui(matchRowImage, pos).x;
      uint matchColumn = (uint) read_imageui(matchColumnImage, pos).x;
      int2 matchPos = (int2) (matchColumn, matchRow);

      if (matchRow != 0 && matchColumn != 0)
      {
         countOne += 1;
         float cx1 = read_imagef(one3, pos).x;
         float cy1 = read_imagef(one4, pos).x;
         float cz1 = read_imagef(one5, pos).x;
         pointOne = (float4) (cx1 - params[4], cy1 - params[5], cz1 - params[6], 0);

         countTwo += 1;
         float cx2 = read_imagef(two3, matchPos).x;
         float cy2 = read_imagef(two4, matchPos).x;
         float cz2 = read_imagef(two5, matchPos).x;
         pointTwo = (float4) (cx2 - params[7], cy2 - params[8], cz2 - params[9], 0);

         // float weight = 1.0/(float)(length(pointOne - pointTwo));

         // Add 9x1 correlation vector into "correl" array
         correl[0] += pointOne.x * pointTwo.x;
         correl[1] += pointOne.x * pointTwo.y;
         correl[2] += pointOne.x * pointTwo.z;
         correl[3] += pointOne.y * pointTwo.x;
         correl[4] += pointOne.y * pointTwo.y;
         correl[5] += pointOne.y * pointTwo.z;
         correl[6] += pointOne.z * pointTwo.x;
         correl[7] += pointOne.z * pointTwo.y;
         correl[8] += pointOne.z * pointTwo.z;
      }
   }

   // Store final 9x1 "correl" into cIndex'th block in "correlation"
   for (int k = 0; k < 9; k++)
   {
      correlation[cIndex * 9 + k] = correl[k];
   }
}
