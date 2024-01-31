package us.ihmc.perception.odometry;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.rapidRegions.PatchFeatureGrid;

import java.nio.FloatBuffer;
import java.util.ArrayList;

/**
 * This class is used to compute the transform between two PatchFeatureGrids using Iterative Closest Point (ICP) on the GPU.
 * It has its own associated OpenCL kernel to perform matching, similarity and centroid calculations necessary for ICP.
 *
 * {@link PatchFeatureGrid} is a class that stores the class that manages OpenCL handles for centroid and surface normals
 * computed on a depth map or height map or polar grid map.
 *
 * @author Bhavyansh Mishra
 */
public class RapidPatchesBasedICP
{
   private int patchColumns;
   private int patchRows;

   private boolean initialized = false;

   private OpenCLManager openCLManager;
   private _cl_kernel correspondenceKernel;
   private _cl_kernel centroidReduceKernel;
   private _cl_kernel correlReduceKernel;
//   private _cl_kernel addReduceKernel;

   private OpenCLFloatBuffer parametersBuffer;
   private OpenCLFloatBuffer centroidBuffer;
   private OpenCLFloatBuffer correlBuffer;

   private BytedecoImage rowMatchIndexImage;
   private BytedecoImage columnMatchIndexImage;

   private final Point3D centroidPrevious = new Point3D();
   private final Point3D centroidCurrent = new Point3D();

   private final RigidBodyTransform transformToPrevious = new RigidBodyTransform();

   private final SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);
   private final DMatrixRMaj svdU = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj svdVt = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj correl = new DMatrixRMaj(3, 3);

   /**
    * Sets up the kernels and buffers required for ICP on the GPU.
    *
    * @param openCLManager OpenCLManager passed in from the calling class
    * @param program OpenCL program passed in from the calling class
    * @param patchRows Number of rows in feature grid map
    * @param patchColumns Number of columns in feature grid map
    */
   public void create(OpenCLManager openCLManager, _cl_program program, int patchRows, int patchColumns)
   {
      this.openCLManager = openCLManager;
      this.correspondenceKernel = openCLManager.createKernel(program, "correspondenceKernel");
      this.centroidReduceKernel = openCLManager.createKernel(program, "centroidReduceKernel");
      this.correlReduceKernel = openCLManager.createKernel(program, "correlReduceKernel");
//      this.addReduceKernel = openCLManager.createKernel(program, "addReduceKernel");

      this.patchRows = patchRows;
      this.patchColumns = patchColumns;

      parametersBuffer = new OpenCLFloatBuffer(10);
      centroidBuffer = new OpenCLFloatBuffer(patchColumns * 6);
      correlBuffer = new OpenCLFloatBuffer(patchColumns * 9);

      rowMatchIndexImage = new BytedecoImage(patchColumns, patchRows, opencv_core.CV_16UC1);
      columnMatchIndexImage = new BytedecoImage(patchColumns, patchRows, opencv_core.CV_16UC1);

   }

   /**
    * Computes the transform between two PatchFeatureGrids using ICP on the GPU. The final result is stored in transformToPrevious.
    *
    * @param previousFeatureGrid PatchFeatureGrid from previous frame
    * @param currentFeatureGrid PatchFeatureGrid from current frame
    */
   public void update(PatchFeatureGrid previousFeatureGrid, PatchFeatureGrid currentFeatureGrid)
   {
      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) currentFeatureGrid.getRows());
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) currentFeatureGrid.getColumns());
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, 0.3f);
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, 100.0f);

      if (!initialized)
      {
         parametersBuffer.createOpenCLBufferObject(openCLManager);
         centroidBuffer.createOpenCLBufferObject(openCLManager);
         correlBuffer.createOpenCLBufferObject(openCLManager);

         rowMatchIndexImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         columnMatchIndexImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

         initialized = true;
      }
      else
      {
         parametersBuffer.writeOpenCLBufferObject(openCLManager);

         setFeatureGridKernelArguments(correspondenceKernel, previousFeatureGrid, currentFeatureGrid);
         openCLManager.setKernelArgument(correspondenceKernel, 12, rowMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(correspondenceKernel, 13, columnMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(correspondenceKernel, 14, parametersBuffer.getOpenCLBufferObject());
         openCLManager.execute2D(correspondenceKernel, patchColumns, patchRows);

         setFeatureGridKernelArguments(centroidReduceKernel, previousFeatureGrid, currentFeatureGrid);
         openCLManager.setKernelArgument(centroidReduceKernel, 12, rowMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 13, columnMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 14, centroidBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 15, parametersBuffer.getOpenCLBufferObject());
         openCLManager.execute1D(centroidReduceKernel, patchColumns);
         centroidBuffer.readOpenCLBufferObject(openCLManager);

         centroidPrevious.setToZero();
         centroidCurrent.setToZero();

         LogTools.debug("Before -> Centroid One: " + centroidPrevious + ", Centroid Two: " + centroidCurrent);

         collectCentroid(centroidBuffer.getBackingDirectFloatBuffer(), centroidPrevious, centroidCurrent);

         LogTools.debug("After -> Centroid One: " + centroidPrevious + ", Centroid Two: " + centroidCurrent);

         setFeatureGridKernelArguments(correlReduceKernel, previousFeatureGrid, currentFeatureGrid);
         openCLManager.setKernelArgument(correlReduceKernel, 12, rowMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(correlReduceKernel, 13, columnMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(correlReduceKernel, 14, correlBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(correlReduceKernel, 15, parametersBuffer.getOpenCLBufferObject());
         openCLManager.execute1D(correlReduceKernel, patchColumns);
         correlBuffer.readOpenCLBufferObject(openCLManager);

         correl.zero();
         collectCorrelation(correlBuffer.getBackingDirectFloatBuffer(), correl);

         LogTools.debug("Correlation Matrix: " + correl);

         computeTransform(centroidPrevious, centroidCurrent, correl, transformToPrevious);
         LogTools.debug("Transform: \n" + transformToPrevious);
      }
   }

   /**
    * Performs reduction across all columns of the correlation matrix
    *
    * @param correlBuffer DirectFloatBuffer containing the tensor of correlation matrices for each column
    * @param correl DMatrixRMaj to store the final correlation matrix
    */
   private void collectCorrelation(FloatBuffer correlBuffer, DMatrixRMaj correl)
   {
      correl.zero();
      for (int i = 0; i < patchColumns; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            for (int k = 0; k < 3; k++)
            {
               correl.add(j, k, correlBuffer.get(i * 9 + j * 3 + k));
            }
         }
      }
   }

   /**
    * Performs reduction across all columns of the centroid vectors
    *
    * @param centroidBuffer DirectFloatBuffer containing the matrix of centroid vectors for each column
    * @param centroidOneToPack Vector3D to store the final centroid vector for the first feature grid
    * @param centroidTwoToPack Vector3D to store the final centroid vector for the second feature grid
    */
   private void collectCentroid(FloatBuffer centroidBuffer, Point3D centroidOneToPack, Point3D centroidTwoToPack)
   {
      int countOne = 0;
      int countTwo = 0;
      Point3D centroidOne = new Point3D();
      Point3D centroidTwo = new Point3D();
      for (int i = 0; i < patchColumns; i++)
      {
         centroidOne.set(centroidBuffer.get(i * 6), centroidBuffer.get(i * 6 + 1), centroidBuffer.get(i * 6 + 2));
         centroidTwo.set(centroidBuffer.get(i * 6 + 3), centroidBuffer.get(i * 6 + 4), centroidBuffer.get(i * 6 + 5));

         if (centroidOne.norm() > 0.3 && centroidOne.norm() < 100)
         {
            countOne++;
            centroidOneToPack.add(centroidOne);
         }

         if (centroidTwo.norm() > 0.3 && centroidTwo.norm() < 100)
         {
            countTwo++;
            centroidTwoToPack.add(centroidTwo);
         }
      }
      LogTools.debug("Unscaled -> Centroid One: " + centroidOneToPack + ", Centroid Two: " + centroidTwoToPack);
      centroidOneToPack.scale(1.0 / countOne);
      centroidTwoToPack.scale(1.0 / countTwo);
   }

   /**
    * Computes the transform from the second feature grid to the first feature grid
    *
    * @param centroidPrevious Centroid vector for the previous feature grid
    * @param centroidCurrent Centroid vector for the current feature grid
    * @param correl Correlation matrix
    * @param transformToPack Transform from the second feature grid to the first feature grid
    */
   private void computeTransform(Point3D centroidPrevious, Point3D centroidCurrent, DMatrixRMaj correl, RigidBodyTransform transformToPack)
   {
      if (svd.decompose(correl))
      {
         svd.getU(svdU, false);
         svd.getV(svdVt, true);

         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.mult(svdU, svdVt, rotationMatrix);

         transformToPack.setRotationAndZeroTranslation(rotationMatrix);
      }
      else
      {
         transformToPack.setToZero();
      }

      Point3D translation = new Point3D(centroidPrevious);
      translation.sub(centroidCurrent);
      transformToPack.appendTranslation(translation);
   }

   private void setFeatureGridKernelArguments(_cl_kernel kernel, PatchFeatureGrid previousFeatureGrid, PatchFeatureGrid currentFeatureGrid)
   {
      openCLManager.setKernelArgument(kernel, 0, previousFeatureGrid.getNxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 1, previousFeatureGrid.getNyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 2, previousFeatureGrid.getNzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 3, previousFeatureGrid.getCxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 4, previousFeatureGrid.getCyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 5, previousFeatureGrid.getCzImage().getOpenCLImageObject());

      openCLManager.setKernelArgument(kernel, 6, currentFeatureGrid.getNxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 7, currentFeatureGrid.getNyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 8, currentFeatureGrid.getNzImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 9, currentFeatureGrid.getCxImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 10, currentFeatureGrid.getCyImage().getOpenCLImageObject());
      openCLManager.setKernelArgument(kernel, 11, currentFeatureGrid.getCzImage().getOpenCLImageObject());
   }

   public void testAlignmentICP()
   {
      ArrayList<Point3D> cube = new ArrayList<>();
      cube.add(new Point3D(1.0, 1.0, 1.0));
      cube.add(new Point3D(1.0, -1.0, 1.0));
      cube.add(new Point3D(-1.0, -1.0, 1.0));
      cube.add(new Point3D(-1.0, 1.0, 1.0));
      cube.add(new Point3D(1.0, 1.0, -1.0));
      cube.add(new Point3D(1.0, -1.0, -1.0));
      cube.add(new Point3D(-1.0, -1.0, -1.0));
      cube.add(new Point3D(-1.0, 1.0, -1.0));

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendPitchRotation(0.123);
      transform.appendYawRotation(0.231);
      //transform.appendTranslation(0.2, 0.3, 0.4);

      ArrayList<Point3D> transformedCube = new ArrayList<>();
      for (Point3D point3d : cube)
      {
         Point3D transformedPoint = new Point3D();
         transform.transform(point3d, transformedPoint);
         transformedCube.add(transformedPoint);
      }

      SvdImplicitQrDecompose_DDRM svd = new SvdImplicitQrDecompose_DDRM(false, true, true, true);
      DMatrixRMaj svdU = new DMatrixRMaj(3, 3);
      DMatrixRMaj svdVt = new DMatrixRMaj(3, 3);
      DMatrixRMaj patchMatrix = new DMatrixRMaj(3, 3);

      DMatrixRMaj matrixOne = new DMatrixRMaj(3, cube.size());
      DMatrixRMaj matrixTwo = new DMatrixRMaj(3, transformedCube.size());

      for (int i = 0; i < cube.size(); i++)
      {
         Point3D pointOne = cube.get(i);
         matrixOne.set(0, i, pointOne.getX());
         matrixOne.set(1, i, pointOne.getY());
         matrixOne.set(2, i, pointOne.getZ());

         Point3D pointTwo = transformedCube.get(i);
         matrixTwo.set(0, i, pointTwo.getX());
         matrixTwo.set(1, i, pointTwo.getY());
         matrixTwo.set(2, i, pointTwo.getZ());

         LogTools.debug("Matrix1: {}", matrixOne);
         LogTools.debug("Matrix2: {}", matrixTwo);

         CommonOps_DDRM.multAddTransB(matrixOne, matrixTwo, patchMatrix);
      }

      if (svd.decompose(patchMatrix))
      {
         svd.getU(svdU, false);
         svd.getV(svdVt, true);

         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.mult(svdU, svdVt, rotationMatrix);

         LogTools.debug("Rotation Matrix: " + rotationMatrix);

         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setRotationAndZeroTranslation(rotationMatrix);

         LogTools.debug("Transform: \n{}", rigidBodyTransform);

         Point3D angles = new Point3D();
         rigidBodyTransform.getRotation().getEuler(angles);

         LogTools.debug("Angles: {}", angles);
      }
   }
}
