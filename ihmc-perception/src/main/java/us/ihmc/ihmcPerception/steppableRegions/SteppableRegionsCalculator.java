package us.ihmc.ihmcPerception.steppableRegions;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class SteppableRegionsCalculator
{
   private static final float distanceFromCliffTops = 0.02f;
   private static final float distanceFromCliffBottoms = 0.05f;
   private static final int yawDiscretizations = 5;
   private static final float footWidth = 0.12f;
   private static final float footLength = 0.22f;

   private static final int defaultCells = 50;
   private final OpenCLManager openCLManager;
   private _cl_program steppableRegionsProgram;
   private _cl_kernel computeSteppabilityKernel;

   private final OpenCLFloatParameters steppableParameters = new OpenCLFloatParameters();
   private final BytedecoImage heightMapImage = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);

   private int cellsPerSide;

   public SteppableRegionsCalculator()
   {
      // Load all the native data on the thread. This effectively just loads all the bytedeco stuff to be used by the planner
      var nativeLoader = BytedecoTools.loadNativesOnAThread();
      boolean doneLoading = false;

      openCLManager = new OpenCLManager();

      while (!doneLoading)
      {
         if (nativeLoader.poll())
         {
            if (nativeLoader.isNewlyActivated())
            {
               createOpenCLStuff(defaultCells);
               doneLoading = true;
            }
         }
      }
   }

   /**
    * This creates all the open cl managers, programs, and kernels used in the planner
    */
   private void createOpenCLStuff(int cellsPerSide)
   {
      this.cellsPerSide = cellsPerSide;

      openCLManager.create();

      steppableRegionsProgram = openCLManager.loadProgram("SteppableRegions");
      computeSteppabilityKernel = openCLManager.createKernel(steppableRegionsProgram, "computeSteppability");
   }

   public void compute(HeightMapData heightMapData)
   {
      resize(heightMapData.getCellsPerAxis());

      populateSteppabilityParameters(heightMapData);
      populateHeightMapImage(heightMapData);

      openCLManager.setKernelArgument(computeSteppabilityKernel, 0, steppableParameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 1, heightMapImage.getOpenCLImageObject());

      openCLManager.execute3D(computeSteppabilityKernel, cellsPerSide, cellsPerSide, yawDiscretizations);

      openCLManager.finish();
   }

   private void resize(int cellsPerSide)
   {
      if (this.cellsPerSide == cellsPerSide)
         return;

      this.cellsPerSide = cellsPerSide;

      heightMapImage.resize(cellsPerSide, cellsPerSide, openCLManager, null);
   }

   private void populateSteppabilityParameters(HeightMapData heightMapData)
   {
      steppableParameters.setParameter(heightMapData.getCenterIndex());
      steppableParameters.setParameter((float) heightMapData.getGridResolutionXY());
      steppableParameters.setParameter(distanceFromCliffTops);
      steppableParameters.setParameter(distanceFromCliffBottoms);
      steppableParameters.setParameter(yawDiscretizations);
      steppableParameters.setParameter(footWidth);
      steppableParameters.setParameter(footLength);

      steppableParameters.writeOpenCLBufferObject(openCLManager);
   }

   private void populateHeightMapImage(HeightMapData heightMapData)
   {
      for (int xIndex = 0; xIndex < heightMapData.getCellsPerAxis(); xIndex++)
      {
         for (int yIndex = 0; yIndex < heightMapData.getCellsPerAxis(); yIndex++)
         {
            // the x and y values are switched between the world coordinates and the image coordinates
            heightMapImage.setFloat(yIndex, xIndex, (float) heightMapData.getHeightAt(xIndex, yIndex));
         }
      }
      heightMapImage.writeOpenCLImage(openCLManager);
   }

   public static void main(String[] args)
   {
      new SteppableRegionsCalculator();
   }

}
