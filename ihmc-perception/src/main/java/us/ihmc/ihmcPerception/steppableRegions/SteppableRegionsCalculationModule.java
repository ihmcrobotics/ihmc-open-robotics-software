package us.ihmc.ihmcPerception.steppableRegions;

import io.netty.buffer.ByteBuf;
import javafx.scene.paint.Color;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SteppableRegionDebugImageMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.memory.NativeMemoryTools;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParametersReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

public class SteppableRegionsCalculationModule
{
   public static final int yawDiscretizations = 5;
   public static final float footWidth = 0.12f;
   public static final float footLength = 0.22f;

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

   private final SteppableRegionsListCollection regionCollection = new SteppableRegionsListCollection(yawDiscretizations);
   private final List<SteppableRegionsCalculator.SteppableRegionsEnvironmentModel> regionEnvironments = new ArrayList<>();

   private int cellsPerSide;

   private final IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);

   private final List<Consumer<SteppableRegionsListCollection>> steppableRegionListOutputConsumers = new ArrayList<>();
   private final List<Consumer<SteppableRegionDebugImagesMessage>> steppableRegionDebugConsumers = new ArrayList<>();

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

   public SteppableRegionsListCollection getSteppableRegionsListCollection()
   {
      return regionCollection;
   }

   public void addSteppableRegionListCollectionOutputConsumer(Consumer<SteppableRegionsListCollection> outputConsumer)
   {
      this.steppableRegionListOutputConsumers.add(outputConsumer);
   }

   public void addSteppableRegionDebugConsumer(Consumer<SteppableRegionDebugImagesMessage> outputConsumer)
   {
      this.steppableRegionDebugConsumers.add(outputConsumer);
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
      timer.start();

      regionCollection.clear();
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

         double yawAngle = ((double) yawValue) / (yawDiscretizations - 1) * Math.PI;

         SteppableRegionsCalculator.SteppableRegionsEnvironmentModel environment = SteppableRegionsCalculator.mergeCellsIntoSteppableRegionEnvironment(
               steppabilityImages.get(yawValue),
               snapHeightImages.get(yawValue),
               snapNormalXImages.get(yawValue),
               snapNormalYImages.get(yawValue),
               snapNormalZImages.get(yawValue),
               steppabilityConnections.get(yawValue));
         polygonizerParameters.setLengthThreshold(0.4 * heightMapData.getGridResolutionXY()); // this is critical to prevent it from filtering small regions
         SteppableRegionsList regions = SteppableRegionsCalculator.createSteppableRegions(concaveHullParameters,
                                                                                          polygonizerParameters,
                                                                                          environment,
                                                                                          heightMapData,
                                                                                          yawAngle,
                                                                                          footLength,
                                                                                          footWidth);

         this.regionEnvironments.add(environment);
         this.regionCollection.setSteppableRegions(yawValue, regions);
      }

      LogTools.info("time = " + timer.lapElapsed());
      timer.suspend();

      SteppableRegionDebugImagesMessage debugImagesMessage = new SteppableRegionDebugImagesMessage();
      for (int i = 0; i < yawDiscretizations; i++)
      {
         generateSteppableRegionDebugImage(i, debugImagesMessage.getRegionImages().add());
         generateSteppabilityDebugImage(i, debugImagesMessage.getSteppabilityImages().add());
      }

      for (Consumer<SteppableRegionsListCollection> outputConsumer : steppableRegionListOutputConsumers)
         outputConsumer.accept(regionCollection);
      for (Consumer<SteppableRegionDebugImagesMessage> debugConsumer : steppableRegionDebugConsumers)
         debugConsumer.accept(debugImagesMessage);
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

   private void generateSteppabilityDebugImage(int yawIndex, SteppableRegionDebugImageMessage messageToPack)
   {
      int totalSize = 3 * cellsPerSide * cellsPerSide;
      ByteBuffer uncompressedByteBuffer = NativeMemoryTools.allocate(totalSize);
      uncompressedByteBuffer.order(ByteOrder.nativeOrder());

      for (int row = 0; row < cellsPerSide; row++)
      {
         for (int col = 0; col < cellsPerSide; col++)
         {
            Color color;
            int status = steppabilityImages.get(yawIndex).getInt(row, col);
            if (status == 0)
               color = Color.WHITE; // valid
            else if (status == 1)
               color = Color.BLACK; // cliff top
            else if (status == 3)
               color = Color.BLUE; // bad snap
            else
               color = Color.GRAY; // cliff bottom

            uncompressedByteBuffer.put((byte) (color.getRed() * 255.0));
            uncompressedByteBuffer.put((byte) (color.getGreen() * 255.0));
            uncompressedByteBuffer.put((byte) (color.getBlue() * 255.0));
         }
      }

      messageToPack.getData().reset();
      uncompressedByteBuffer.rewind();
      for (int i = 0; i < totalSize; i++)
         messageToPack.getData().add(uncompressedByteBuffer.get());
      messageToPack.setImageHeight(cellsPerSide);
      messageToPack.setImageWidth(cellsPerSide);
   }

   private void generateSteppableRegionDebugImage(int yawIndex, SteppableRegionDebugImageMessage messageToPack)
   {
      SteppableRegionsCalculator.SteppableRegionsEnvironmentModel environmentModel = regionEnvironments.get(yawIndex);
      int totalSize = 3 * cellsPerSide * cellsPerSide;
      ByteBuffer uncompressedByteBuffer = NativeMemoryTools.allocate(totalSize);
      uncompressedByteBuffer.order(ByteOrder.nativeOrder());

      // fill with black
      uncompressedByteBuffer.rewind();
      for (int x = 0; x < cellsPerSide; x++)
      {
         for (int y = 0; y < cellsPerSide; y++)
         {
            uncompressedByteBuffer.put((byte) 0);
            uncompressedByteBuffer.put((byte) 0);
            uncompressedByteBuffer.put((byte) 0);
         }
      }

      for (SteppableRegionsCalculator.SteppableRegionDataHolder region : environmentModel.getRegions())
      {
         for (SteppableRegionsCalculator.SteppableCell cell : region.getCells())
         {
            int x = cell.getX();
            int y = cell.getY();

            int row = cellsPerSide - x - 1;
            int column = cellsPerSide - y - 1;
            int index = row * cellsPerSide + column;
            int start = 3 * index;

            int r = (region.regionNumber + 1) * 312 % 255;
            int g = (region.regionNumber + 1) * 123 % 255;
            int b = (region.regionNumber + 1) * 231 % 255;
            uncompressedByteBuffer.put(start, (byte) r);
            uncompressedByteBuffer.put(start + 1, (byte) g);
            uncompressedByteBuffer.put(start + 2, (byte) b);
         }
      }


      messageToPack.getData().reset();
      uncompressedByteBuffer.rewind();
      for (int i = 0; i < totalSize; i++)
         messageToPack.getData().add(uncompressedByteBuffer.get());
      messageToPack.setImageHeight(cellsPerSide);
      messageToPack.setImageWidth(cellsPerSide);
   }

   public static void main(String[] args)
   {
      new SteppableRegionsCalculationModule();
   }
}
