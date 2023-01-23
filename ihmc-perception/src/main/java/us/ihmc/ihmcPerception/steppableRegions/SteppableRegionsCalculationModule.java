package us.ihmc.ihmcPerception.steppableRegions;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.List;

public class SteppableRegionsCalculationModule
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
   private _cl_kernel computeSteppabilityConnectionsKernel;

   private final OpenCLFloatParameters steppableParameters = new OpenCLFloatParameters();
   private final BytedecoImage heightMapImage = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityImage0 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityImage1 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityImage2 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityImage3 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityImage4 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityImage5 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityConnections0 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityConnections1 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityConnections2 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityConnections3 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityConnections4 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);
   private final BytedecoImage steppabilityConnections5 = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);

   private List<SteppableRegion> regions0;
   private List<SteppableRegion> regions1;
   private List<SteppableRegion> regions2;
   private List<SteppableRegion> regions3;
   private List<SteppableRegion> regions4;
   private List<SteppableRegion> regions5;

   private int cellsPerSide;

   public SteppableRegionsCalculationModule()
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

   public List<SteppableRegion> getConvexSteppableRegionsYaw0()
   {
      return regions0;
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
      computeSteppabilityConnectionsKernel = openCLManager.createKernel(steppableRegionsProgram, "computeSteppabilityConnections");
   }

   public void compute(HeightMapData heightMapData)
   {
      resize(heightMapData.getCellsPerAxis());

      populateSteppabilityParameters(heightMapData);
      populateHeightMapImage(heightMapData);

      openCLManager.setKernelArgument(computeSteppabilityKernel, 0, steppableParameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 1, heightMapImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 2, steppabilityImage0.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 3, steppabilityImage1.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 4, steppabilityImage2.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 5, steppabilityImage3.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 6, steppabilityImage4.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityKernel, 7, steppabilityImage5.getOpenCLImageObject());

      openCLManager.execute3D(computeSteppabilityKernel, cellsPerSide, cellsPerSide, yawDiscretizations);

      computeConnectionsOnTheGPU(steppabilityImage0, steppabilityConnections0);
      computeConnectionsOnTheGPU(steppabilityImage1, steppabilityConnections1);
      computeConnectionsOnTheGPU(steppabilityImage2, steppabilityConnections2);
      computeConnectionsOnTheGPU(steppabilityImage3, steppabilityConnections3);
      computeConnectionsOnTheGPU(steppabilityImage4, steppabilityConnections4);
      computeConnectionsOnTheGPU(steppabilityImage5, steppabilityConnections5);

      openCLManager.finish();

      createRegionsFromResults(heightMapData);
   }

   private void computeConnectionsOnTheGPU(BytedecoImage steppability, BytedecoImage connections)
   {
      openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 0, steppableParameters.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 1, steppability.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 2, connections.getOpenCLImageObject());

      openCLManager.execute2D(computeSteppabilityConnectionsKernel, cellsPerSide, cellsPerSide);

      steppability.readOpenCLImage(openCLManager);
      connections.readOpenCLImage(openCLManager);
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

   private void createRegionsFromResults(HeightMapData heightMapData)
   {
      regions0 = createSteppableRegions(heightMapData, steppabilityImage0, steppabilityConnections0);
      regions1 = createSteppableRegions(heightMapData, steppabilityImage1, steppabilityConnections1);
      regions2 = createSteppableRegions(heightMapData, steppabilityImage2, steppabilityConnections2);
      regions3 = createSteppableRegions(heightMapData, steppabilityImage3, steppabilityConnections3);
      regions4 = createSteppableRegions(heightMapData, steppabilityImage4, steppabilityConnections4);
      regions5 = createSteppableRegions(heightMapData, steppabilityImage5, steppabilityConnections5);
   }

   private static List<SteppableRegion> createSteppableRegions(HeightMapData heightMapData, BytedecoImage steppability, BytedecoImage steppabilityConnections)
   {
      SteppableRegionsCalculator.SteppableRegionsEnvironmentModel environment0 = SteppableRegionsCalculator.mergeCellsIntoSteppableRegionEnvironment(
            steppability,
            steppabilityConnections);
      return SteppableRegionsCalculator.createSteppableRegions(environment0, heightMapData);
   }

   public static void main(String[] args)
   {
      new SteppableRegionsCalculationModule();
   }
}
