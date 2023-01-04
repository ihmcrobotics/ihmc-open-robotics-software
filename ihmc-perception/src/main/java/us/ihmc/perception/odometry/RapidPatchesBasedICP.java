package us.ihmc.perception.odometry;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

import static org.bytedeco.opencv.global.opencv_core.CV_32F;
import static org.bytedeco.opencv.global.opencv_core.CV_8UC2;

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

   public RapidPatchesBasedICP(int patchRows, int patchColumns)
   {
      openCLManager = new OpenCLManager();
      openCLManager.create();

      icpProgram = openCLManager.loadProgram("RapidPatchesBasedICP");
      icpKernel = openCLManager.createKernel(icpProgram, "icpKernel");

      this.patchRows = patchRows;
      this.patchColumns = patchColumns;

      this.currentFeatureMat = new Mat(patchRows, patchColumns, opencv_core.CV_32FC(6));
      this.previousFeatureMat = new Mat(patchRows, patchColumns, opencv_core.CV_32FC(6));

      parametersBuffer = new OpenCLFloatBuffer( 3);
      previousFeatureBuffer = new OpenCLFloatBuffer( patchRows * patchColumns * 6, previousFeatureMat.data().asBuffer().asFloatBuffer());
      currentFeatureBuffer = new OpenCLFloatBuffer( patchRows * patchColumns * 6, currentFeatureMat.data().asBuffer().asFloatBuffer());
   }

   public void update(Mat featureMap)
   {
      if (!initialized)
      {
      }

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

      previousFeatureBuffer.createOpenCLBufferObject(openCLManager);
      currentFeatureBuffer.createOpenCLBufferObject(openCLManager);

      //previousFeatureBuffer.writeOpenCLBufferObject(openCLManager);
      //currentFeatureBuffer.writeOpenCLBufferObject(openCLManager);

      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) 0.1f);
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) 0.2f);
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, (float) 0.3f);

      parametersBuffer.createOpenCLBufferObject(openCLManager);
      previousFeatureBuffer.createOpenCLBufferObject(openCLManager);
      currentFeatureBuffer.createOpenCLBufferObject(openCLManager);

      openCLManager.setKernelArgument(icpKernel, 0, previousFeatureBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(icpKernel, 1, currentFeatureBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(icpKernel, 2, parametersBuffer.getOpenCLBufferObject());

      openCLManager.execute2D(icpKernel, patchRows, patchColumns);

      openCLManager.finish();
   }

   public static void main(String[] args)
   {
      RapidPatchesBasedICP rapidPatchesBasedICP = new RapidPatchesBasedICP(10, 10);
      rapidPatchesBasedICP.testInitialization();
   }
}
