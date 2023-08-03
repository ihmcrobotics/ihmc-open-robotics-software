#define FILTER_DISPARITY_THRESHOLD 0
#define MERGE_ANGULAR_THRESHOLD 1
#define MERGE_DISTANCE_THRESHOLD 2
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

float4 back_project(int2 pos, float Z, global float* params)
{
   float px = (pos.x - params[DEPTH_CX]) / params[DEPTH_FX] * Z;
   float py = (pos.y - params[DEPTH_CY]) / params[DEPTH_FY] * Z;
   float4 X = (float4) (px, py, Z, 0);
   return X;
}

float3 estimate_normal(read_only image2d_t in, int x, int y, global float* params)
{
   float residual = 0;
   float Z = 0;
   int m = 1;
   int count = 0;
   float4 normal = (float4) (0, 0, 0, 0);
   if (y >= 0 && y < (int) params[SUB_H] && x >= 0 && x < (int) params[SUB_W])
   {
      for (int i = 0; i < (int) params[PATCH_HEIGHT] - m; i++)
      {
         for (int j = 0; j < (int) params[PATCH_WIDTH] - m; j++)
         {
            count++;
            int gx = x * (int) params[PATCH_HEIGHT] + i;
            int gy = y * (int) params[PATCH_WIDTH] + j;
            int2 pos = (int2) (gx, gy);

            pos = (int2) (gx, gy);
            Z = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 va = back_project(pos, Z, params);

            pos = (int2) (gx + m, gy);
            Z = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vb = back_project(pos, Z, params);

            pos = (int2) (gx + m, gy + m);
            Z = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vc = back_project(pos, Z, params);

            pos = (int2) (gx, gy + m);
            Z = ((float) read_imageui(in, pos).x) / (float) 1000;
            float4 vd = back_project(pos, Z, params);

            normal += cross((vc - vb), (vb - va));
            normal += cross((vd - vc), (vc - vb));
            normal += cross((va - vd), (vd - vc));
            normal += cross((vb - va), (va - vd));
         }
      }
   }
   return (1 / (float) (count)) * normal.xyz;
}

float3 estimate_centroid(read_only image2d_t in, int x, int y, global float* params)
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
               float4 P = back_project(pos, Z, params);
               centroid += P.xyz;
            }
         }
      }
   }
   return (1 / (float) (count)) * centroid;
}

bool isConnected(float3 ag, float3 an, float3 bg, float3 bn, global float* params)
{
   float3 vec = ag - bg;
   float dist = length(vec);
   float sim = fabs(dot(an, bn));
   float perpDist = fabs(dot(ag - bg, bn)) + fabs(dot(bg - ag, an));
   if (perpDist < params[MERGE_DISTANCE_THRESHOLD] * dist * 40 && sim > params[MERGE_ANGULAR_THRESHOLD])
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

// This isn't used and isn't documented
float4 transform_unused(float4 point, float4 r1, float4 r2, float4 r3, float4 t)
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
   int y = get_global_id(0);
   int x = get_global_id(1);

   //    if(x==0 && y==0) printf("PackKernel:(%d,%d,%d,%d,%d,%.2lf,%.2lf)\n",
   //                            (int)params[SUB_H],
   //                            (int)params[SUB_W],
   //                            (int)params[PATCH_HEIGHT],
   //                            (int)params[PATCH_WIDTH],
   //                            (int)params[FILTER_DISPARITY_THRESHOLD],
   //                            params[MERGE_ANGULAR_THRESHOLD],
   //                            params[MERGE_DISTANCE_THRESHOLD]);

   if (y >= 0 && y < (int) params[SUB_H] && x >= 0 && x < (int) params[SUB_W])
   {
      float3 normal = estimate_normal(in, x, y, params);
      float3 centroid = estimate_centroid(in, x, y, params);

      //        if(x==24 && y==50) printf("PackKernel Normal:(%.4lf, %.4lf, %.4lf)\n", normal.x, normal.y, normal.z);
      //        if(x==26 && y==7) printf("PackKernel Centroid:(%.4lf, %.4lf, %.4lf)\n", centroid.x, centroid.y, centroid.z);
      //        printf("PackKernel Centroid:(%.4lf, %.4lf, %.4lf)\n", centroid.x, centroid.y, centroid.z);

      write_imagef(out0, (int2) (x, y), (float4) (normal.x, 0, 0, 0));
      write_imagef(out1, (int2) (x, y), (float4) (normal.y, 0, 0, 0));
      write_imagef(out2, (int2) (x, y), (float4) (normal.z, 0, 0, 0));
      write_imagef(out3, (int2) (x, y), (float4) (centroid.x, 0, 0, 0));
      write_imagef(out4, (int2) (x, y), (float4) (centroid.y, 0, 0, 0));
      write_imagef(out5, (int2) (x, y), (float4) (centroid.z, 0, 0, 0));
   }
}

/* Merge Kernel: Creates the graph-based structure by adding connections between the neighboring
 * patches based on similarity.
 */
void kernel mergeKernel(read_only image2d_t out0,
                        read_only image2d_t out1,
                        read_only image2d_t out2, /* float3 maps for normal*/
                        read_only image2d_t out3,
                        read_only image2d_t out4,
                        read_only image2d_t out5,  /* float3 maps for centroids */
                        write_only image2d_t out6, /* uint8 map for patch metadata*/
                        global float* params       /* All parameters */
                                                   // write_only image2d_t debug
)
{
   int y = get_global_id(0);
   int x = get_global_id(1);

   int m = 2;

   //     if(x==0 && y==0) printf("MergeKernel:(%d,%d)\n", (int)params[SUB_H], (int)params[SUB_W]);
   //    printf("MergeKernel: subHeight: %d, subWidth: %d, x: %d, y: %d \n", (int) params[SUB_H], (int) params[SUB_W], x, y);
   if (y >= m && y < (int) params[SUB_H] - m && x >= m && x < (int) params[SUB_W] - m)
   {

      float n1_a = read_imagef(out0, (int2) (x, y)).x;
      float n2_a = read_imagef(out1, (int2) (x, y)).x;
      float n3_a = read_imagef(out2, (int2) (x, y)).x;
      float g1_a = read_imagef(out3, (int2) (x, y)).x;
      float g2_a = read_imagef(out4, (int2) (x, y)).x;
      float g3_a = read_imagef(out5, (int2) (x, y)).x;

      //        if (n1_a + n2_a + n3_a + g1_a + g2_a + g3_a > 0 || n1_a + n2_a + n3_a + g1_a + g2_a + g3_a < 0)
      //            printf("MergeKernel: n1_a: %f, n2_a: %f, n3_a: %f, g1_a: %f, g2_a: %f, g3_a: %f \n", n1_a, n2_a, n3_a, g1_a, g2_a, g3_a);

      float3 g_a = (float3) (g1_a, g2_a, g3_a);
      float3 n_a = (float3) (n1_a, n2_a, n3_a);

      uint boundaryConnectionsEncodedAsOnes = (uint) (0);

      int count = 0;
      for (int i = -m; i < m + 1; i += m)
      {
         for (int j = -m; j < m + 1; j += m)
         {
            // printf("MergeKernel:(%d,%d)\n", i, j);
            if (!(j == 0 && i == 0))
            {
               float n1_b = read_imagef(out0, (int2) (x + i, y + j)).x;
               float n2_b = read_imagef(out1, (int2) (x + i, y + j)).x;
               float n3_b = read_imagef(out2, (int2) (x + i, y + j)).x;
               float g1_b = read_imagef(out3, (int2) (x + i, y + j)).x;
               float g2_b = read_imagef(out4, (int2) (x + i, y + j)).x;
               float g3_b = read_imagef(out5, (int2) (x + i, y + j)).x;

               float3 g_b = (float3) (g1_b, g2_b, g3_b);
               float3 n_b = (float3) (n1_b, n2_b, n3_b);

               if (isConnected(g_a, normalize(n_a), g_b, normalize(n_b), params))
               {
                  // printf("Connected: (%d,%d)\n",x+i, y+j);
                  boundaryConnectionsEncodedAsOnes = (1 << count) | boundaryConnectionsEncodedAsOnes;
               }
               count++;
            }
         }
      }
      write_imageui(out6, (int2) (x, y), (uint4) (boundaryConnectionsEncodedAsOnes, 0, 0, 0));
   }
}
