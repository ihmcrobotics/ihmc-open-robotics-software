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
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.rapidRegions.PatchFeatureGrid;

import java.nio.FloatBuffer;
import java.util.ArrayList;

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


   private final Point3D centroidOne = new Point3D();
   private final Point3D centroidTwo = new Point3D();


   public void create(OpenCLManager openCLManager, _cl_program program, int patchRows, int patchColumns)
   {
      this.openCLManager = openCLManager;
      this.correspondenceKernel = openCLManager.createKernel(program, "correspondenceKernel");
      this.centroidReduceKernel = openCLManager.createKernel(program, "centroidReduceKernel");
      this.correlReduceKernel = openCLManager.createKernel(program, "correlReduceKernel");
//      this.addReduceKernel = openCLManager.createKernel(program, "addReduceKernel");

      this.patchRows = patchRows;
      this.patchColumns = patchColumns;

      parametersBuffer = new OpenCLFloatBuffer(4);
      centroidBuffer = new OpenCLFloatBuffer(patchColumns * 6);
      correlBuffer = new OpenCLFloatBuffer(patchRows * patchColumns * 9);

      rowMatchIndexImage = new BytedecoImage(patchColumns, patchRows, opencv_core.CV_16UC1);
      columnMatchIndexImage = new BytedecoImage(patchColumns, patchRows, opencv_core.CV_16UC1);

   }

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

         LogTools.info("Before Correspondence Kernel");
         openCLManager.execute2D(correspondenceKernel, patchColumns, patchRows);
         LogTools.info("After Correspondence Kernel");

         setFeatureGridKernelArguments(centroidReduceKernel, previousFeatureGrid, currentFeatureGrid);
         openCLManager.setKernelArgument(centroidReduceKernel, 12, rowMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 13, columnMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 14, centroidBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 15, parametersBuffer.getOpenCLBufferObject());

         LogTools.info("Before Centroid Kernel");
         openCLManager.execute1D(centroidReduceKernel, patchColumns);
         LogTools.info("After Centroid Kernel");

         setFeatureGridKernelArguments(correlReduceKernel, previousFeatureGrid, currentFeatureGrid);
         openCLManager.setKernelArgument(centroidReduceKernel, 12, rowMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 13, columnMatchIndexImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 14, correlBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(centroidReduceKernel, 15, parametersBuffer.getOpenCLBufferObject());

         LogTools.info("Before Correlation Kernel");
         openCLManager.execute2D(correlReduceKernel, patchColumns, patchRows);
         LogTools.info("After Correlation Kernel");



//         setFeatureGridKernelArguments(addReduceKernel, previousFeatureGrid, currentFeatureGrid);
//         openCLManager.setKernelArgument(addReduceKernel, 12, correlBuffer.getOpenCLBufferObject());
//         openCLManager.setKernelArgument(addReduceKernel, 13, parametersBuffer.getOpenCLBufferObject());

         openCLManager.finish();

//         finalCentroidReduce(centroidBuffer.getBackingDirectFloatBuffer(), centroidPrevious, centroidCurrent);

//         centroidBuffer.readOpenCLBufferObject(openCLManager);
      }
   }

   private void finalCentroidReduce(FloatBuffer buffer, Point3D centroidOneToPack, Point3D centroidTwoToPack)
   {
      int countOne = 0;
      int countTwo = 0;
      centroidOne.set(0,0,0);
      centroidTwo.set(0,0,0);
      for (int i = 0; i < patchColumns; i++)
      {
         centroidOne.set(buffer.get(i * 6), buffer.get(i * 6 + 1), buffer.get(i * 6 + 2));
         centroidTwo.set(buffer.get(i * 6 + 3), buffer.get(i * 6 + 4), buffer.get(i * 6 + 5));

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

      centroidOneToPack.scale(1.0 / countOne);
      centroidTwoToPack.scale(1.0 / countTwo);
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

         LogTools.info("Matrix1: {}", matrixOne);
         LogTools.info("Matrix2: {}", matrixTwo);

         CommonOps_DDRM.multAddTransB(matrixOne, matrixTwo, patchMatrix);
      }

      if (svd.decompose(patchMatrix))
      {
         svd.getU(svdU, false);
         svd.getV(svdVt, true);

         DMatrixRMaj rotationMatrix = new DMatrixRMaj(3, 3);
         CommonOps_DDRM.mult(svdU, svdVt, rotationMatrix);

         LogTools.info("Rotation Matrix: " + rotationMatrix);

         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         rigidBodyTransform.setRotationAndZeroTranslation(rotationMatrix);

         LogTools.info("Transform: \n{}", rigidBodyTransform);

         Point3D angles = new Point3D();
         rigidBodyTransform.getRotation().getEuler(angles);

         LogTools.info("Angles: {}", angles);
      }
   }
}
