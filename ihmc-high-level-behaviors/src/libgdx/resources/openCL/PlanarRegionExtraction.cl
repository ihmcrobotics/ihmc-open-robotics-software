
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

float4 back_project(int2 pos, float Z, global float* params){
    float px = (pos.x - params[DEPTH_CX])/(params[DEPTH_FX]) * Z;
    float py = (pos.y - params[DEPTH_CY])/(params[DEPTH_FY]) * Z;
    float4 X = (float4)(px, py, Z, 0);
    return X;
}

 float3 estimate_normal(read_only image2d_t in, int x, int y, global float* params){
    float residual = 0;
    float Z = 0;
    int m = 1;
    int count = 0;
    float4 normal = (float4)(0,0,0,0);
    if(y >= 0 && y < (int)params[SUB_H] && x >= 0 && x < (int)params[SUB_W]){

        for(int i = 0; i<(int)params[PATCH_HEIGHT]-m; i++){
            for(int j = 0; j<(int)params[PATCH_WIDTH]-m; j++){
                count++;
                int gx = x*(int)params[PATCH_HEIGHT] + i;
                int gy = y*(int)params[PATCH_WIDTH] + j;
                int2 pos = (int2)(gx,gy);

                pos = (int2)(gx,gy);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                float4 va = back_project(pos, Z, params);

                pos = (int2)(gx + m, gy);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                float4 vb = back_project(pos, Z, params);

                pos = (int2)(gx + m, gy + m);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                float4 vc = back_project(pos, Z, params);

                pos = (int2)(gx,gy + m);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                float4 vd = back_project(pos, Z, params);

                normal += cross((vc-vb),(vb-va));
                normal += cross((vd-vc),(vc-vb));
                normal += cross((va-vd),(vd-vc));
                normal += cross((vb-va),(va-vd));

            }
        }
    }
    return (1/(float)(count)) * normal.xyz;
}

 float3 estimate_centroid(read_only image2d_t in, int x, int y, global float* params){
     float Z = 0;
     int count = 0;
     float3 centroid = (float3)(0,0,0);
    if(y >= 0 && y < (int)params[SUB_H] && x >= 0 && x < (int)params[SUB_W]){
        for(int i = 0; i<(int)params[PATCH_HEIGHT]; i++){
            for(int j = 0; j<(int)params[PATCH_WIDTH]; j++){
                count++;
                int gx = x*(int)params[PATCH_HEIGHT] + i;
                int gy = y*(int)params[PATCH_WIDTH] + j;
                int2 pos = (int2)(gx,gy);
                Z = ((float)read_imageui(in, pos).x)/(float)1000;
                if(Z > 0.1f){
                    float4 P = back_project(pos, Z, params);
                    centroid += P.xyz;
                }

            }
        }
    }
    return (1/(float)(count)) * centroid;
 }

bool isConnected(float3 ag, float3 an, float3 bg, float3 bn, global float* params){
    float3 vec = ag - bg;
    float dist = length(vec);
    float sim = fabs(dot(an, bn));
    float perpDist = fabs(dot(ag-bg, bn)) + fabs(dot(bg-ag, an));
    if (perpDist < params[MERGE_DISTANCE_THRESHOLD] * dist * 40 && sim > params[MERGE_ANGULAR_THRESHOLD]){
        return true;
    }else {
        return false;
    }
}

void fill_dead_pixels(read_only image2d_t in, int x, int y, write_only image2d_t out0, global float* params){
    uint Z = 0;
    int count = 0;
    int xs[8], ys[8], values[8];
    int total_unique = 0;

    /* Find all non-zero unique depth values available in this patch. */
    for(int i = 0; i<(int)params[FILTER_KERNEL_SIZE]; i++){
        for(int j = 0; j<(int)params[FILTER_KERNEL_SIZE]; j++){
            count++;
            int gx = x * ((int)params[FILTER_KERNEL_SIZE]) + i;
            int gy = y * ((int)params[FILTER_KERNEL_SIZE]) + j;

            int2 pos = (int2)(gx,gy);
            Z = read_imageui(in, pos).x;

            /* For every unique non-zero depth value, insert x,y,z into xs,ys and values, respectively. */
            if (Z != 0){
                /* For the first non-zero depth, simply insert.*/
                if (total_unique == 0){
                    xs[total_unique] = gx;
                    ys[total_unique] = gy;
                    values[total_unique] = Z;
                    total_unique++;
                }else{
                    /* For all other non-zero values, check against all previously visited non-zero unique depth values. */
                    int unique = 0;
                    for(int k = 0; k<total_unique; k++){

                        if ((Z - values[k])*(Z - values[k]) > params[FILTER_DISPARITY_THRESHOLD]){
                            // if (x==0 && y==0)printf("Compare(%d,%d,%d)(%d,%d)\n", Z, abs(Z - values[k]), values[k], unique, total_unique);
                            unique++;

                        }
                    }
                    /* After checking to see if the non-zero value is truly unique against stored unique values, insert.*/
                    if (unique == total_unique){
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
    for(int i = 0; i<(int)params[FILTER_KERNEL_SIZE]; i++){
        for(int j = 0; j<(int)params[FILTER_KERNEL_SIZE]; j++){
            count++;
            int gx = x*(int)params[FILTER_KERNEL_SIZE] + i;
            int gy = y*(int)params[FILTER_KERNEL_SIZE] + j;

            int2 pos = (int2)(gx,gy);
            Z = read_imageui(in, pos).x;

            if (Z == 0){
                float minDist = 10000000;
                float curDist = 0;
                int minIndex = 0;

                /* Find the nearest non-zero-unique neighbor for this dead pixel. */
                for(int m = 0; m<total_unique; m++){
                    curDist = length((float2)(gx,gy) - (float2)(xs[m], ys[m]));
                    // if (x==0 && y==0) printf("Filling(%d,%d)-(%d,%d):%.2lf\n", gx,gy,xs[m],ys[m],curDist);
                    if (curDist < minDist){
                        minDist = curDist;
                        minIndex = m;
                    }
                }
                Z = values[minIndex];
                // if (x==0 && y==0) printf("%d\t",Z);
            }

            write_imageui(out0, pos, (uint4)(Z,0,0,0));
        }
    }
}

void smooth_non_boundary(read_only image2d_t in, int x, int y, write_only image2d_t out0, global float* params){
   uint Z = 0;
   int m = 5;
   int left = 0;
   int right = 0;
   for(int i = 0; i<(int)params[FILTER_KERNEL_SIZE]; i++)
   {
      for (int j = 0; j < (int) params[FILTER_KERNEL_SIZE]; j++)
      {
         int gx = x * ((int) params[FILTER_KERNEL_SIZE]) + i;
         int gy = y * ((int) params[FILTER_KERNEL_SIZE]) + j;


         if(gx-m >= 0 && gx+m < params[INPUT_HEIGHT]){
            int2 pos = (int2)(gx, gy);
            Z = read_imageui(in, pos).x;

            pos = (int2)(gx-m, gy);
            left = read_imageui(in, pos).x;

            pos = (int2)(gx+m, gy);
            right = read_imageui(in, pos).x;

            if( abs((Z - left) - (right - Z)) < 50){
               Z = (left + right) / 2;
               write_imageui(out0, pos, (uint4)(Z,0,0,0));
            }

         }

      }
   }
}

float4 transform(float4 point, float4 r1, float4 r2, float4 r3, float4 t)
{
   return (float4)(dot(r1, point) + t.x, dot(r2, point) + t.y, dot(r3, point) + t.z, 0);
}

float3 calculateNormal(float3 p1, float3 p2, float3 p3)
{
   float3 v12 = p2 - p1;
   float3 v23 = p3 - p2;
   return normalize(cross(v12, v23));
}

/* ++++++++++++++++++++++++++++++++++++++++++ OPEN_CL KERNELS BEGIN HERE ++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* OpenCL Kernels Begin Here. All utilities above this point. */


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  GPU Planar Region Kernels Here
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

/* Filter Kernel: Filters all pixels within a patch on depth map. Removes outliers, flying points, dead pixels and measurement
 * noise. */

 void kernel filterKernel(read_only image2d_t in, write_only image2d_t filteredDepth, write_only image2d_t buffer_nx, global float* params){
    int y = get_global_id(0);
    int x = get_global_id(1);

//    if(x==0 && y==0) printf("FilterKernel\n");
    if(y >= 0 && y < (int)params[FILTER_SUB_H] && x >= 0 && x < (int)params[FILTER_SUB_W]){

        fill_dead_pixels(in, x, y, filteredDepth, params);
//        mark_boundary_patches(in, x, y, buffer_nx, params);
//        smooth_non_boundary(in, x, y, filteredDepth, params);
    }
}

/* Pack Kernel: Generates the patches by packing the centroid, surface normal and metadata related to
 * patches on a sub-sampled grid of the depth map. The following intrinsic parameters are used to convert to Point Cloud.
 * K: [459.97265625, 0.0, 341.83984375, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 1.0]
 * */
void kernel packKernel(  read_only image2d_t in,
	                        write_only image2d_t out0, write_only image2d_t out1, write_only image2d_t out2, /* float3 maps for normal*/
                            write_only image2d_t out3, write_only image2d_t out4, write_only image2d_t out5, /* float3 maps for centroids */
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

    if(y >= 0 && y < (int)params[SUB_H] && x >= 0 && x < (int)params[SUB_W]){
        float3 normal = estimate_normal(in, x, y, params);
        float3 centroid = estimate_centroid(in, x, y, params);

//        if(x==24 && y==50) printf("PackKernel Normal:(%.4lf, %.4lf, %.4lf)\n", normal.x, normal.y, normal.z);

        write_imagef(out0, (int2)(x,y), (float4)(normal.x,0,0,0));
        write_imagef(out1, (int2)(x,y), (float4)(normal.y,0,0,0));
        write_imagef(out2, (int2)(x,y), (float4)(normal.z,0,0,0));
        write_imagef(out3, (int2)(x,y), (float4)(centroid.x,0,0,0));
        write_imagef(out4, (int2)(x,y), (float4)(centroid.y,0,0,0));
        write_imagef(out5, (int2)(x,y), (float4)(centroid.z,0,0,0));
    }
}


/* Merge Kernel: Creates the graph-based structure by adding connections between the neighboring
 * patches based on similarity.
 */
void kernel mergeKernel( write_only image2d_t out0, write_only image2d_t out1, write_only image2d_t out2, /* float3 maps for normal*/
                          write_only image2d_t out3, write_only image2d_t out4, write_only image2d_t out5, /* float3 maps for centroids */
                          write_only image2d_t out6, /* uint8 map for patch metadata*/
                          global float* params /* All parameters */
                          // write_only image2d_t debug
){
     int y = get_global_id(0);
     int x = get_global_id(1);

     int m = 2;

//     if(x==0 && y==0) printf("MergeKernel:(%d,%d)\n", (int)params[SUB_H], (int)params[SUB_W]);
     if(y >= m && y < (int)params[SUB_H]-m && x >= m && x < (int)params[SUB_W]-m){

        float n1_a = read_imagef(out0, (int2)(x,y)).x;
        float n2_a = read_imagef(out1, (int2)(x,y)).x;
        float n3_a = read_imagef(out2, (int2)(x,y)).x;
        float g1_a = read_imagef(out3, (int2)(x,y)).x;
        float g2_a = read_imagef(out4, (int2)(x,y)).x;
        float g3_a = read_imagef(out5, (int2)(x,y)).x;

        float3 g_a = (float3)(g1_a,g2_a,g3_a);
        float3 n_a = (float3)(n1_a,n2_a,n3_a);

        uint patch = (uint)(0);

        int count = 0;
        for(int i = -m; i<m+1; i+=m){
            for(int j = -m; j<m+1; j+=m){
                if (!(j==0 && i==0)){

                     float n1_b = read_imagef(out0, (int2)(x+i,y+j)).x;
                     float n2_b = read_imagef(out1, (int2)(x+i,y+j)).x;
                     float n3_b = read_imagef(out2, (int2)(x+i,y+j)).x;
                     float g1_b = read_imagef(out3, (int2)(x+i,y+j)).x;
                     float g2_b = read_imagef(out4, (int2)(x+i,y+j)).x;
                     float g3_b = read_imagef(out5, (int2)(x+i,y+j)).x;

                     float3 g_b = (float3)(g1_b,g2_b,g3_b);
                     float3 n_b = (float3)(n1_b,n2_b,n3_b);

                     if(isConnected(g_a, normalize(n_a), g_b, normalize(n_b), params)){
                         // printf("Connected: (%d,%d)\n",x+i, y+j);
                         patch = (1 << count) | patch;
                     }
                     count++;
                }
            }
        }
        write_imageui(out6, (int2)(x,y), (uint4)(patch, 0, 0, 0));

    }
}

/*
 * Segmentation Kernel for generating patch graph for colored images.
 * */
void kernel segmentKernel(read_only image2d_t color, write_only image2d_t filteredImage, write_only image2d_t graph, global float* params){
   int y = get_global_id(0);
   int x = get_global_id(1);

   //    if(x==0 && y==0) printf("SegmentKernel\n");
   if(y >= 0 && y < (int)params[FILTER_SUB_H] && x >= 0 && x < (int)params[FILTER_SUB_W]){
      fill_dead_pixels(color, x, y, filteredImage, params);
   }
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  ICP Kernels Here
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * */

/*
 * Correspondence Kernel for Iterative Closest Point
 * */
void kernel correspondenceKernel(global float* cloudOne, global float* transformOne, global float* cloudTwo, global float* transformTwo,
                                 global int* matches, int sizeOne, int sizeTwo, int threads)
{
   int gid = get_global_id(0);
   float4 pointOne = (float4)(0,0,0,0);
   float4 pointTwo = (float4)(0,0,0,0);

//   if(gid==0) printf("CorrespondenceKernel\n");

   float minLength = 10000000;
   float distance = 0;
   int minIndex = -1;
   pointOne = (float4)(cloudOne[gid*3+0], cloudOne[gid*3+1], cloudOne[gid*3+2], 0);
   pointOne = transform(pointOne, (float4)(transformOne[0], transformOne[1], transformOne[2], 0),
                                 (float4)(transformOne[3], transformOne[4], transformOne[5], 0),
                                 (float4)(transformOne[6], transformOne[7], transformOne[8], 0),
                                 (float4)(transformOne[9], transformOne[10], transformOne[11], 0));

   for(int j = 0; j<sizeTwo/3; j+=16)
   {
      pointTwo = (float4)(cloudTwo[j*3+0], cloudTwo[j*3+1], cloudTwo[j*3+2], 0);
      pointTwo = transform(pointTwo, (float4)(transformTwo[0], transformTwo[1], transformTwo[2], 0),
                     (float4)(transformTwo[3], transformTwo[4], transformTwo[5], 0),
                     (float4)(transformTwo[6], transformTwo[7], transformTwo[8], 0),
                     (float4)(transformTwo[9], transformTwo[10], transformTwo[11], 0));

      distance = length(pointTwo - pointOne);
      if(distance < minLength){
         minIndex = j;
         minLength = distance;
      }
   }

//   printf("Match(%d,%d) Dist:%.3lf\n", gid, minIndex, minLength);
//   if(minLength < 1.0) {
      matches[gid] = minIndex;
//   }
}

/*
 * Correlation Matrix Calculation Kernel for Iterative Closest Point
 * */
void kernel correlationKernel(global float* cloudOne,global float* cloudTwo,
      global float* mean, global int* matches, global float* correlation, int sizeOne, int sizeTwo, int threads)
{
   int gid = get_global_id(0);
//   if(gid==0) printf("CorrelationKernel() Works!\n");

   float4 pointOne = (float4)(0,0,0,0);
   float4 pointTwo = (float4)(0,0,0,0);
   int totalPoints = sizeOne/3;
   int blockSize = totalPoints/threads;
   int startPoint = gid * blockSize;
   int endPoint = startPoint + blockSize;
   float correl[9];

   int count = 0;

//   if(gid == 0) printf("Mean: %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf\n", mean[0],mean[1],mean[2],mean[3],mean[4],mean[5]);

   for(int k = 0; k<9; k++)
   {
      correl[k] = 0;
      correlation[gid*9 + k] = 0;
   }

   // For each point in block of points
   for(int i = startPoint; i<endPoint; i++)
   {
      if(matches[i] != -1 && matches[i] != 0)
      {
         count += 1;
         // Calculate correlation 3x3 -> 9x1 for point with matching point.
         pointOne = (float4)(cloudOne[i*3+0] - mean[0], cloudOne[i*3+1] - mean[1], cloudOne[i*3+2] - mean[2], 0);
         pointTwo = (float4)(cloudTwo[matches[i]*3+0] - mean[3], cloudTwo[matches[i]*3+1] - mean[4], cloudTwo[matches[i]*3+2] - mean[5], 0);

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
   // Store final 9x1 "correl" into gid'th block in "correlation"
   for(int k = 0; k<9; k++)
   {
      correlation[gid*9 + k] = correl[k];
   }
//   printf("CountCorrelation: (%d,%d) %d - %d\n", startPoint, endPoint, gid, count);
}


/*
 * Centroid Calculation Kernel for Iterative Closest Point
 * */
void kernel centroidKernel(global float* cloudOne, global float* cloudTwo,
      global float* mean, global int* matches, int sizeOne, int sizeTwo, int threads)
{
   int gid = get_global_id(0);
//   if(gid==0) printf("CentroidKernel() Works!\n");

   float4 pointOne = (float4)(0,0,0,0);
   float4 pointTwo = (float4)(0,0,0,0);
   int totalPoints = sizeOne/3;
   int blockSize = totalPoints/threads;
   int startPoint = gid * blockSize;
   int endPoint = startPoint + blockSize;
   float meanVec[6];

   int count = 0;

   for(int k = 0; k<6; k++)
   {
      meanVec[k] = 0;
      mean[gid*6 + k] = 0;
   }

   // For each point in block of points
   for(int i = startPoint; i<endPoint; i++)
   {
      if(matches[i] != -1 && matches[i] != 0)
      {
         count += 1;
         // Calculate correlation 3x3 -> 6x1 for point with matching point.
         pointOne = (float4)(cloudOne[i*3+0] - mean[0], cloudOne[i*3+1] - mean[1], cloudOne[i*3+2] - mean[2], 0);
         pointTwo = (float4)(cloudTwo[matches[i]*3+0] - mean[3], cloudTwo[matches[i]*3+1] - mean[4], cloudTwo[matches[i]*3+2] - mean[5], 0);

         // Add 6x1 correlation vector into "correl" array
         meanVec[0] += pointOne.x;
         meanVec[1] += pointOne.y;
         meanVec[2] += pointOne.z;
         meanVec[3] += pointTwo.x;
         meanVec[4] += pointTwo.y;
         meanVec[5] += pointTwo.z;
      }
   }
   // Store final 6x1 "correl" into gid'th block in "correlation"
   for(int k = 0; k<6; k++)
   {
      mean[gid*6 + k] = (float)meanVec[k] / (float)count;
   }
//   printf("Mean(%d) Count(%d): (%.3lf, %.3lf, %.3lf, %.3lf, %.3lf, %.3lf)\n", gid, count,
//          mean[gid*6], mean[gid*6 + 1], mean[gid*6 + 2], mean[gid*6 + 3], mean[gid*6 + 4], mean[gid*6 + 5]);
}


/*
 * Cylinder Kernel for Iterative Closest Point
 * */
void kernel cylinderKernel(global float* cloud, global int* cylinderIndices, int size, int numParts)
{
   int partId = get_global_id(0);
   int partMaxPoints = size/numParts;
   int blockStart = partId * partMaxPoints;
   int outputCounter = 0;
   float4 pointOne;

   //      if(gid==0)printf("GID: %d, i:%d, length:%.3lf\n", gid, i, s);
   float partYaw = 360 / numParts;
   float yaw = 0;
   int yawCount = -1;

   // Initialize part IDs for all points to -1
   for(int i = blockStart; i<blockStart + partMaxPoints; i++)
   {
      cylinderIndices[i] = -1;
   }

   // Go through all points and check if their yaw falls within the current partId yaw region.
   for(int i = 0; i<size; i++)
   {
      pointOne = (float4)(cloud[i*3], cloud[i*3+1], cloud[i*3+2], 1);
      float s = length(pointOne);

      // Check whether the point falls inside the 'partId'.
      yaw = degrees(atan2(pointOne.z, pointOne.x));
      yawCount = (int)(yaw / partYaw) + numParts/2;

//      printf("PartId: %d, i:%d, Yaw:%.3lf, YawCount:%d\n", partId, outputCounter, yaw, yawCount);
      if(outputCounter < partMaxPoints && yawCount == partId)
      {
         // Store the point index in cylinderIndices at 'outputCounter'
         cylinderIndices[blockStart + outputCounter] = i;
         outputCounter++;
      }
   }
}

/*
 * Planes Kernel for calculating planar surfaces and normals within a cylinder section.
 */
void kernel planesKernel(global float* cloud, global int* cylinderIndices, global int* blockIds,
                         global int* blockPointCount, int size, int numParts, int numBlocks)
{
   int partId = get_global_id(0);

   if(partId == 0) printf("PlanesKernel\n");

   int partMaxPoints = size / numParts;
   int blockMaxPoints = partMaxPoints / numBlocks;
   int blockStart = partId * partMaxPoints;
   int blockCountStart = partId * numBlocks;
   float4 point = (float4)(0,0,0,1);
   float blockPitch = 90 / numBlocks;
   float radius = 0.0f;
   float pitch = 0.0f;
   int blockCount = -1;

   // Identify the pointIds that fall on each of the blocks.
   for(int i = blockStart; i<blockStart + partMaxPoints; i++)
   {
      point = (float4)(cloud[3*cylinderIndices[i]+0], cloud[3*cylinderIndices[i]+1], cloud[3*cylinderIndices[i]+2], 1);
      radius = sqrt(point.x * point.x + point.z * point.z);
      pitch = degrees(atan2(point.y, radius));
      blockCount = (int) (pitch / blockPitch) + numBlocks/2 - 1;

      if(blockPointCount[blockCountStart + blockCount] < blockMaxPoints)
      {
         blockPointCount[blockCountStart + blockCount]++;
         blockIds[blockStart + blockCount * blockMaxPoints + blockPointCount[blockCountStart + blockCount]] = cylinderIndices[i];
      }
//      if(partId == 1 && blockCount == 2) printf("%.3lf, %.3lf, %.3lf\n", point.x, point.y, point.z);
//      if(partId == 36 && blockCount == 7) printf("%.3lf, %.3lf, %.3lf\n", point.x, point.y, point.z);
   }

//   for(int i = 0; i<numBlocks * numParts; i++)
//   {
//      if(partId == 0)printf("%d, %d -> %d\n", i/numBlocks, i%numBlocks, blockPointCount[i]);
//   }

}

void kernel normalsKernel(global float* cloud, global int* cylinderIndices, global int* blockIds,
      global float* normals, int size, int numParts, int numBlocks)
{
   int gid = get_global_id(0);
   int blockId = gid % numBlocks;
   int partId = gid / numBlocks;
   int partMaxPoints = size / numParts;
   int blockMaxPoints = partMaxPoints / numBlocks;
   int blockStart = (partId * partMaxPoints) + (blockId * blockMaxPoints);
   float3 p1 = (float3)(0,0,0);
   float3 p2 = (float3)(0,0,0);
   float3 p3 = (float3)(0,0,0);
   float3 normal = (float3)(0,0,0);
   float3 centroid = (float3)(0,0,0);
   int pointId = 0;

   int centroidCount = 0;
   for(int i = 0; i<blockMaxPoints; i++)
   {
      pointId = blockStart + i;
      p1 = (float3)(cloud[blockIds[pointId] * 3], cloud[blockIds[pointId] * 3 + 1], cloud[blockIds[pointId] * 3 + 2]);
//      if(partId == 1 && blockId == 2) printf("%.3lf, %.3lf, %.3lf\n", p1.x, p1.y, p1.z);

      if(blockIds[pointId] > 0)
      {
         centroid += p1;
         centroidCount += 1;
      }
   }
   if(centroidCount != 0) centroid = centroid / centroidCount;
   normals[gid * 6 + 3] = centroid.x;
   normals[gid * 6 + 4] = centroid.y;
   normals[gid * 6 + 5] = centroid.z;

//   printf("%.3lf, %.3lf, %.3lf\n", centroid.x, centroid.y, centroid.z);

//   for(int i = 0; i<blockMaxPoints - 40; i++)
//   {
//      pointId = blockStart + i;
//      p1 = (float3)(cloud[blockIds[pointId] * 3], cloud[blockIds[pointId] * 3 + 1], cloud[blockIds[pointId] * 3 + 2]);
//      p2 = (float3)(cloud[blockIds[pointId + 15] * 3], cloud[blockIds[pointId + 15] * 3 + 1], cloud[blockIds[pointId + 15] * 3 + 2]);
//      p3 = (float3)(cloud[blockIds[pointId + 30] * 3], cloud[blockIds[pointId + 30] * 3 + 1], cloud[blockIds[pointId + 30] * 3 + 2]);
//
//
//
//      //      printf("%d, %d, %d\n", i, pointId, blockIds[pointId]);
//   }


}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
