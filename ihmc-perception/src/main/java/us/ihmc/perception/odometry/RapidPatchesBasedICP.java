package us.ihmc.perception.odometry;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.svd.SvdImplicitQrDecompose_DDRM;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

import java.util.ArrayList;

public class RapidPatchesBasedICP
{

   private Mat currentFeatureMat;
   private Mat previousFeatureMat;

   private int patchColumns;
   private int patchRows;

   private boolean initialized = false;

   private OpenCLManager openCLManager;
   private _cl_kernel icpKernel;
   private _cl_program icpProgram;

   private OpenCLFloatBuffer parametersBuffer;
   private OpenCLFloatBuffer previousFeatureBuffer;
   private OpenCLFloatBuffer currentFeatureBuffer;


   public void create(int patchRows, int patchColumns)
   {
      openCLManager = new OpenCLManager();
      openCLManager.create();

      icpProgram = openCLManager.loadProgram("RapidPatchesBasedICP");
      icpKernel = openCLManager.createKernel(icpProgram, "icpKernel");

      this.patchRows = patchRows;
      this.patchColumns = patchColumns;

      this.currentFeatureMat = new Mat(patchRows, patchColumns, opencv_core.CV_32FC(6));
      this.previousFeatureMat = new Mat(patchRows, patchColumns, opencv_core.CV_32FC(6));

      parametersBuffer = new OpenCLFloatBuffer(3);
      previousFeatureBuffer = new OpenCLFloatBuffer(patchRows * patchColumns * 6, previousFeatureMat.data().asBuffer().asFloatBuffer());
      currentFeatureBuffer = new OpenCLFloatBuffer(patchRows * patchColumns * 6, currentFeatureMat.data().asBuffer().asFloatBuffer());
   }

   public void update(Mat featureMap)
   {
      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) 0.1f);
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) 0.2f);
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, (float) 0.3f);

      this.currentFeatureMat.put(featureMap);

      if (!initialized)
      {
         parametersBuffer.createOpenCLBufferObject(openCLManager);
         previousFeatureBuffer.createOpenCLBufferObject(openCLManager);
         currentFeatureBuffer.createOpenCLBufferObject(openCLManager);
      }
      else
      {
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
         previousFeatureBuffer.writeOpenCLBufferObject(openCLManager);
         currentFeatureBuffer.writeOpenCLBufferObject(openCLManager);

         openCLManager.setKernelArgument(icpKernel, 0, previousFeatureBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(icpKernel, 1, currentFeatureBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(icpKernel, 2, parametersBuffer.getOpenCLBufferObject());

         openCLManager.execute2D(icpKernel, patchRows, patchColumns);

         openCLManager.finish();
      }

      previousFeatureMat = currentFeatureMat.clone();
   }

   public void testInitialization()
   {
      Mat ones = new Mat(patchRows, patchColumns, opencv_core.CV_32FC1, new Scalar(1.0));
      Mat twos = new Mat(patchRows, patchColumns, opencv_core.CV_32FC1, new Scalar(2.0));
      Mat threes = new Mat(patchRows, patchColumns, opencv_core.CV_32FC1, new Scalar(3.0));
      Mat fours = new Mat(patchRows, patchColumns, opencv_core.CV_32FC1, new Scalar(4.0));
      Mat fives = new Mat(patchRows, patchColumns, opencv_core.CV_32FC1, new Scalar(5.0));
      Mat sixes = new Mat(patchRows, patchColumns, opencv_core.CV_32FC1, new Scalar(6.0));

      MatVector finalMats = new MatVector();
      finalMats.push_back(ones);
      finalMats.push_back(twos);
      finalMats.push_back(threes);
      finalMats.push_back(fours);
      finalMats.push_back(fives);
      finalMats.push_back(sixes);

      opencv_core.merge(finalMats, previousFeatureMat);

      finalMats.clear();
      finalMats.push_back(sixes);
      finalMats.push_back(fives);
      finalMats.push_back(fours);
      finalMats.push_back(threes);
      finalMats.push_back(twos);
      finalMats.push_back(ones);

      opencv_core.merge(finalMats, currentFeatureMat);

      LogTools.info("First");
      update(currentFeatureMat);

      LogTools.info("Second");
      update(currentFeatureMat);

      LogTools.info("Third");
      update(currentFeatureMat);
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



   public static void main(String[] args)
   {
      RapidPatchesBasedICP rapidPatchesBasedICP = new RapidPatchesBasedICP();
      rapidPatchesBasedICP.create(10,10);
      //rapidPatchesBasedICP.testInitialization();
      //rapidPatchesBasedICP.testAlignmentICP();

   }
}
