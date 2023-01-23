package us.ihmc.ihmcPerception.steppableRegions;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

public class SteppableRegionsCalculationModule
{
   private static final float distanceFromCliffTops = 0.02f;
   private static final float distanceFromCliffBottoms = 0.05f;
   static final int yawDiscretizations = 5;
   static final float footWidth = 0.12f;
   static final float footLength = 0.22f;

   private static final int defaultCells = 50;
   private final OpenCLManager openCLManager;
   private _cl_program steppableRegionsProgram;
   private _cl_kernel computeSteppabilityKernel;
   private _cl_kernel computeSteppabilityConnectionsKernel;

   ConcaveHullFactoryParameters concaveHullParameters = new ConcaveHullFactoryParameters();
   PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   private final OpenCLFloatParameters steppableParameters = new OpenCLFloatParameters();
   private final OpenCLFloatParameters yaw = new OpenCLFloatParameters();
   private final BytedecoImage heightMapImage = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);

   private final List<BytedecoImage> steppabilityImages = new ArrayList<>();
   private final List<BytedecoImage> steppabilityConnections = new ArrayList<>();

   private final List<List<SteppableRegion>> regions = new ArrayList<>();

   private int cellsPerSide;

   public SteppableRegionsCalculationModule()
   {
      // Load all the native data on the thread. This effectively just loads all the bytedeco stuff to be used by the planner
      var nativeLoader = BytedecoTools.loadNativesOnAThread();
      boolean doneLoading = false;

      openCLManager = new OpenCLManager();

      for (int i = 0; i < yawDiscretizations; i++)
      {
         steppabilityImages.add(new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1));
         steppabilityConnections.add(new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1));
      }

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

   public List<List<SteppableRegion>> getSteppableRegions()
   {
      return regions;
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

      heightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      for (int i = 0; i < yawDiscretizations; i++)
      {
         steppabilityImages.get(i).createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         steppabilityConnections.get(i).createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
      }
   }

   public void compute(HeightMapData heightMapData)
   {
      resize(heightMapData.getCellsPerAxis());

      populateSteppabilityParameters(heightMapData);
      populateHeightMapImage(heightMapData);

      Stopwatch timer = new Stopwatch();

      regions.clear();
      for (int yawValue = 0; yawValue < yawDiscretizations; yawValue++)
      {
         yaw.setParameter((float) yawValue);
         yaw.writeOpenCLBufferObject(openCLManager);
         openCLManager.setKernelArgument(computeSteppabilityKernel, 0, steppableParameters.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 1, heightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 2, yaw.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 3, steppabilityImages.get(yawValue).getOpenCLImageObject());

         openCLManager.execute2D(computeSteppabilityKernel, cellsPerSide, cellsPerSide);

         openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 0, steppableParameters.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 1, steppabilityImages.get(yawValue).getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 2, steppabilityConnections.get(yawValue).getOpenCLImageObject());

         openCLManager.execute2D(computeSteppabilityConnectionsKernel, cellsPerSide, cellsPerSide);

         steppabilityImages.get(yawValue).readOpenCLImage(openCLManager);
         steppabilityConnections.get(yawValue).readOpenCLImage(openCLManager);

         openCLManager.finish();

         timer.start();
         regions.add(createSteppableRegions(concaveHullParameters, polygonizerParameters, heightMapData, steppabilityImages.get(yawValue), steppabilityConnections.get(yawValue)));
         LogTools.info("time = " + timer.lapElapsed());
         timer.lap();
      }
   }

   private void resize(int cellsPerSide)
   {
      if (this.cellsPerSide == cellsPerSide)
         return;

      this.cellsPerSide = cellsPerSide;

      heightMapImage.resize(cellsPerSide, cellsPerSide, openCLManager, null);
      for (int i = 0; i < yawDiscretizations; i++)
      {
         steppabilityImages.get(i).resize(cellsPerSide, cellsPerSide, openCLManager, null);
         steppabilityConnections.get(i).resize(cellsPerSide, cellsPerSide, openCLManager, null);
      }
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

   private static List<SteppableRegion> createSteppableRegions(ConcaveHullFactoryParameters concaveHullFactoryParameters,
                                                               PolygonizerParameters polygonizerParameters,
                                                               HeightMapData heightMapData, BytedecoImage steppability, BytedecoImage steppabilityConnections)
   {
      long startTIme = System.nanoTime();
      SteppableRegionsCalculator.SteppableRegionsEnvironmentModel environment0 = SteppableRegionsCalculator.mergeCellsIntoSteppableRegionEnvironment(
            steppability,
            steppabilityConnections);
      LogTools.info("Merge duration " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTIme));
      startTIme = System.nanoTime();
      List<SteppableRegion> regions = SteppableRegionsCalculator.createSteppableRegions(concaveHullFactoryParameters, polygonizerParameters, environment0, heightMapData);
      LogTools.info("Create duration " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTIme));

      return regions;
   }

   public static void main(String[] args)
   {
      new SteppableRegionsCalculationModule();
   }
}
