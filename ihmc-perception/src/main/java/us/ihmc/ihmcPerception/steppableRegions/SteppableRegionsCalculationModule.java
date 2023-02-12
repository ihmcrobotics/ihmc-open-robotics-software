package us.ihmc.ihmcPerception.steppableRegions;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;

public class SteppableRegionsCalculationModule
{
   private static final float distanceFromCliffTops = 0.02f;
   private static final float distanceFromCliffBottoms = 0.05f;
   public static final int yawDiscretizations = 5;
   public static final float footWidth = 0.12f;
   public static final float footLength = 0.22f;
   public static final float cliffStartHeightToAvoid = 0.1f;
   public static final float cliffEndHeightToAvoid = 1.2f;

   private static final int defaultCells = 50;
   private final OpenCLManager openCLManager;
   private _cl_program steppableRegionsProgram;
   private _cl_kernel computeSteppabilityKernel;
   private _cl_kernel computeSteppabilityConnectionsKernel;

   private final ConcaveHullFactoryParameters concaveHullParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   private final SteppableRegionCalculatorParameters parameters = new SteppableRegionCalculatorParameters();
   private final OpenCLFloatParameters steppableParameters = new OpenCLFloatParameters();
   private final OpenCLFloatParameters yaw = new OpenCLFloatParameters();
   private final BytedecoImage heightMapImage = new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1);

   private final List<BytedecoImage> steppabilityImages = new ArrayList<>();
   private final List<BytedecoImage> snapHeightImages = new ArrayList<>();
   private final List<BytedecoImage> snapNormalXImages = new ArrayList<>();
   private final List<BytedecoImage> snapNormalYImages = new ArrayList<>();
   private final List<BytedecoImage> snapNormalZImages = new ArrayList<>();
   private final List<BytedecoImage> steppabilityConnections = new ArrayList<>();

   private final List<List<SteppableRegion>> regions = new ArrayList<>();
   private final List<SteppableRegionsCalculator.SteppableRegionsEnvironmentModel> regionEnvironments = new ArrayList<>();

   private int cellsPerSide;

   public SteppableRegionsCalculationModule()
   {
      // Load all the native data on the thread. This effectively just loads all the bytedeco stuff to be used by the planner
      var nativeLoader = BytedecoTools.loadNativesOnAThread();
      boolean doneLoading = false;

      openCLManager = new OpenCLManager();
      concaveHullParameters.setTriangulationTolerance(5e-3);
      concaveHullParameters.setMaxNumberOfIterations(1000);

      for (int i = 0; i < yawDiscretizations; i++)
      {
         steppabilityImages.add(new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1));
         snapHeightImages.add(new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1));
         snapNormalXImages.add(new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1));
         snapNormalYImages.add(new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1));
         snapNormalZImages.add(new BytedecoImage(defaultCells, defaultCells, opencv_core.CV_32FC1));
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

   public void setSteppableRegionsCalculatorParameters(SteppableRegionCalculatorParametersReadOnly parameters)
   {
      this.parameters.set(parameters);
   }

   public List<SteppableRegionsCalculator.SteppableRegionsEnvironmentModel> getRegionEnvironments()
   {
      return regionEnvironments;
   }

   public List<BytedecoImage> getSteppableImage()
   {
      return steppabilityImages;
   }

   public OpenCLManager getOpenCLManager()
   {
      return openCLManager;
   }

   /**
    * This creates all the open cl managers, programs, and kernels used in the planner
    */
   private void createOpenCLStuff(int cellsPerSide)
   {
      this.cellsPerSide = cellsPerSide;

      openCLManager.create();

      steppableRegionsProgram = openCLManager.loadProgram("SteppableRegions", "HeightMapUtils.cl");
      computeSteppabilityKernel = openCLManager.createKernel(steppableRegionsProgram, "computeSteppability");
      computeSteppabilityConnectionsKernel = openCLManager.createKernel(steppableRegionsProgram, "computeSteppabilityConnections");

      heightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
      for (int i = 0; i < yawDiscretizations; i++)
      {
         steppabilityImages.get(i).createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         snapHeightImages.get(i).createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         snapNormalXImages.get(i).createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         snapNormalYImages.get(i).createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         snapNormalZImages.get(i).createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
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
      regionEnvironments.clear();
      for (int yawValue = 0; yawValue < yawDiscretizations; yawValue++)
      {
         yaw.setParameter((float) yawValue);
         yaw.writeOpenCLBufferObject(openCLManager);
         openCLManager.setKernelArgument(computeSteppabilityKernel, 0, steppableParameters.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 1, heightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 2, yaw.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 3, steppabilityImages.get(yawValue).getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 4, snapHeightImages.get(yawValue).getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 5, snapNormalXImages.get(yawValue).getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 6, snapNormalYImages.get(yawValue).getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityKernel, 7, snapNormalZImages.get(yawValue).getOpenCLImageObject());

         openCLManager.execute2D(computeSteppabilityKernel, cellsPerSide, cellsPerSide);

         openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 0, steppableParameters.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 1, steppabilityImages.get(yawValue).getOpenCLImageObject());
         openCLManager.setKernelArgument(computeSteppabilityConnectionsKernel, 2, steppabilityConnections.get(yawValue).getOpenCLImageObject());

         openCLManager.execute2D(computeSteppabilityConnectionsKernel, cellsPerSide, cellsPerSide);

         steppabilityImages.get(yawValue).readOpenCLImage(openCLManager);
         snapHeightImages.get(yawValue).readOpenCLImage(openCLManager);
         snapNormalXImages.get(yawValue).readOpenCLImage(openCLManager);
         snapNormalYImages.get(yawValue).readOpenCLImage(openCLManager);
         snapNormalZImages.get(yawValue).readOpenCLImage(openCLManager);
         steppabilityConnections.get(yawValue).readOpenCLImage(openCLManager);

         openCLManager.finish();

         timer.start();

         SteppableRegionsCalculator.SteppableRegionsEnvironmentModel environment = SteppableRegionsCalculator.mergeCellsIntoSteppableRegionEnvironment(
               steppabilityImages.get(yawValue),
               snapHeightImages.get(yawValue),
               snapNormalXImages.get(yawValue),
               snapNormalYImages.get(yawValue),
               snapNormalZImages.get(yawValue),
               steppabilityConnections.get(yawValue));
         polygonizerParameters.setLengthThreshold(0.4 * heightMapData.getGridResolutionXY()); // this is critical to prevent it from filtering small regions
         List<SteppableRegion> regions = SteppableRegionsCalculator.createSteppableRegions(concaveHullParameters,
                                                                                           polygonizerParameters,
                                                                                           environment,
                                                                                           heightMapData);

         this.regionEnvironments.add(environment);
         this.regions.add(regions);

         LogTools.info("time = " + timer.lapElapsed());
         timer.lap();
      }
   }

   public void destroy()
   {
      openCLManager.destroy();
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
         snapHeightImages.get(i).resize(cellsPerSide, cellsPerSide, openCLManager, null);
         snapNormalXImages.get(i).resize(cellsPerSide, cellsPerSide, openCLManager, null);
         snapNormalYImages.get(i).resize(cellsPerSide, cellsPerSide, openCLManager, null);
         snapNormalZImages.get(i).resize(cellsPerSide, cellsPerSide, openCLManager, null);
      }
   }

   private void populateSteppabilityParameters(HeightMapData heightMapData)
   {
      steppableParameters.setParameter(heightMapData.getCenterIndex());
      steppableParameters.setParameter((float) heightMapData.getGridResolutionXY());
      steppableParameters.setParameter((float) heightMapData.getGridCenter().getX());
      steppableParameters.setParameter((float) heightMapData.getGridCenter().getY());
      steppableParameters.setParameter((float) parameters.getDistanceFromCliffTops());
      steppableParameters.setParameter((float) parameters.getDistanceFromCliffBottoms());
      steppableParameters.setParameter(parameters.getYawDiscretizations());
      steppableParameters.setParameter((float) parameters.getFootWidth());
      steppableParameters.setParameter((float) parameters.getFootLength());
      steppableParameters.setParameter((float) parameters.getCliffStartHeightToAvoid());
      steppableParameters.setParameter((float) parameters.getCliffEndHeightToAvoid());

      steppableParameters.writeOpenCLBufferObject(openCLManager);
   }

   private void populateHeightMapImage(HeightMapData heightMapData)
   {
      int size = heightMapData.getCellsPerAxis();
      for (int imageRow = 0; imageRow < size; imageRow++)
      {
         for (int imageColumn = 0; imageColumn < size; imageColumn++)
         {
            int x = size - imageRow;
            int y = size - imageColumn;
            // the x and y values are switched between the world coordinates and the image coordinates
            heightMapImage.setFloat(imageRow, imageColumn, (float) heightMapData.getHeightAt(x, y));
         }
      }
      heightMapImage.writeOpenCLImage(openCLManager);
   }

   public static void main(String[] args)
   {
      new SteppableRegionsCalculationModule();
   }
}
