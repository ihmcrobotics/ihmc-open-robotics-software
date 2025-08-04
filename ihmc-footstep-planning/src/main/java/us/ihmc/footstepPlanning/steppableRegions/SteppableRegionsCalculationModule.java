package us.ihmc.footstepPlanning.steppableRegions;

import javafx.scene.paint.Color;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.SteppableRegionDebugImageMessage;
import perception_msgs.msg.dds.SteppableRegionDebugImagesMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.footstepPlanning.steppableRegions.data.SteppableCell;
import us.ihmc.footstepPlanning.steppableRegions.data.SteppableRegionDataHolder;
import us.ihmc.footstepPlanning.steppableRegions.data.SteppableRegionsEnvironmentModel;
import us.ihmc.log.LogTools;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class SteppableRegionsCalculationModule
{
   public static final float footWidth = 0.12f;
   public static final float footLength = 0.22f;

   private final ConcaveHullFactoryParameters concaveHullParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();

   private final SteppableRegionCalculatorParameters parameters;

   private final SteppableRegionsListCollection regionCollection;
   private final List<SteppableRegionsEnvironmentModel> regionEnvironments = new ArrayList<>();

   private final int cellsPerSide;

   private final List<Consumer<SteppableRegionsListCollectionMessage>> steppableRegionListOutputConsumers = new ArrayList<>();
   private final List<Consumer<SteppableRegionDebugImagesMessage>> steppableRegionDebugConsumers = new ArrayList<>();
   private Mat steppabilityMat;

   public SteppableRegionsCalculationModule()
   {
      this(new SteppableRegionCalculatorParameters(), new HeightMapParameters());
   }

   public SteppableRegionsCalculationModule(SteppableRegionCalculatorParametersReadOnly defaultParameters, HeightMapParameters heightMapParameters)
   {
      this.parameters = new SteppableRegionCalculatorParameters(defaultParameters);
      this.cellsPerSide = (HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), heightMapParameters.getGridResolutionXY()) * 2) + 1;

      concaveHullParameters.setEdgeLengthThreshold(1.0);

      regionCollection = new SteppableRegionsListCollection(parameters.getYawDiscretizations());
   }

   public void addSteppableRegionListCollectionOutputConsumer(Consumer<SteppableRegionsListCollectionMessage> outputConsumer)
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

   public void compute(TerrainMapData terrainMapData)
   {
      steppabilityMat = terrainMapData.getSteppabilityMat();
      Stopwatch timer = new Stopwatch();
      timer.start();

      // it's highly likely we have a region that's the ground, so we need to set the length limit to that size.
      concaveHullParameters.setEdgeLengthThreshold(parameters.getEdgeLengthThreshold());
      // this is critical to prevent it from filtering small regions
      polygonizerParameters.setLengthThreshold(0.4 * terrainMapData.getCenterIndex());

      regionCollection.clear();
      regionCollection.resizeCollection(parameters.getYawDiscretizations());

      regionEnvironments.clear();
      for (int yawValue = 0; yawValue < parameters.getYawDiscretizations(); yawValue++)
      {
         double yawAngle = 0.0;
         if (parameters.getYawDiscretizations() > 1)
            yawAngle = ((double) yawValue) / (parameters.getYawDiscretizations() - 1) * Math.PI;

         SteppableRegionsEnvironmentModel environment = SteppableRegionsCalculator.createEnvironmentByMergingCellsIntoRegions(terrainMapData.getSteppabilityMat(),
                                                                                                                              terrainMapData.getSnapHeightMat(),
                                                                                                                              terrainMapData.getSnapNormalXMat(),
                                                                                                                              terrainMapData.getSnapNormalYMat(),
                                                                                                                              terrainMapData.getSnapNormalZMat(),
                                                                                                                              terrainMapData.getSteppabilityConnectionsMat(),
                                                                                                                              parameters,
                                                                                                                              terrainMapData);

         SteppableRegionsList regions = SteppableRegionsCalculator.createSteppableRegions(concaveHullParameters,
                                                                                          polygonizerParameters,
                                                                                          parameters,
                                                                                          environment,
                                                                                          terrainMapData,
                                                                                          yawAngle);

         this.regionEnvironments.add(environment);
         this.regionCollection.setSteppableRegions(yawValue, regions);
      }

      LogTools.info("time = " + timer.lapElapsed());
      timer.suspend();

      SteppableRegionDebugImagesMessage debugImagesMessage = new SteppableRegionDebugImagesMessage();
      for (int i = 0; i < parameters.getYawDiscretizations(); i++)
      {
//         generateSteppableRegionDebugImage(i, debugImagesMessage.getRegionImages().add());
//         generateSteppabilityDebugImage(debugImagesMessage.getSteppabilityImages().add());
      }

      SteppableRegionsListCollectionMessage message = SteppableRegionMessageConverter.convertToSteppableRegionsListCollectionMessage(regionCollection);

      for (Consumer<SteppableRegionsListCollectionMessage> outputConsumer : steppableRegionListOutputConsumers)
         outputConsumer.accept(message);
      for (Consumer<SteppableRegionDebugImagesMessage> debugConsumer : steppableRegionDebugConsumers)
         debugConsumer.accept(debugImagesMessage);
   }

   public int getYawDiscretizations()
   {
      return parameters.getYawDiscretizations();
   }

   private static double computeMaximumWIdthInWindow(SteppableRegionCalculatorParametersReadOnly parameters)
   {
      // we know when snapping, the foot can be off axis, so what's the maximum off axis it can be?
      double rotationForPseudoWidth = (Math.PI / parameters.getYawDiscretizations()) / 2.0;
      Vector2D footVector = new Vector2D(parameters.getFootLength() / 2.0, parameters.getFootWidth() / 2.0);
      Vector2D rotatedFootVector = new Vector2D();
      // apply yaw rotation to the vector
      RotationMatrixTools.applyYawRotation(rotationForPseudoWidth, footVector, rotatedFootVector);

      return rotatedFootVector.getY() * 2.0;
   }

   private void generateSteppabilityDebugImage(SteppableRegionDebugImageMessage messageToPack)
   {
      int totalSize = 3 * cellsPerSide * cellsPerSide;
      ByteBuffer uncompressedByteBuffer = NativeMemoryTools.allocate(totalSize);
      uncompressedByteBuffer.order(ByteOrder.nativeOrder());

      for (int row = 0; row < cellsPerSide; row++)
      {
         for (int col = 0; col < cellsPerSide; col++)
         {
            Color color;
            int status = steppabilityMat.ptr(row, col).get();
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

      messageToPack.getData().resetQuick();
      uncompressedByteBuffer.rewind();
      for (int i = 0; i < totalSize; i++)
         messageToPack.getData().add(uncompressedByteBuffer.get());
      messageToPack.setImageHeight(cellsPerSide);
      messageToPack.setImageWidth(cellsPerSide);
   }

   private void generateSteppableRegionDebugImage(int yawIndex, SteppableRegionDebugImageMessage messageToPack)
   {
      SteppableRegionsEnvironmentModel environmentModel = regionEnvironments.get(yawIndex);
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

      for (SteppableRegionDataHolder region : environmentModel.getRegions())
      {
         for (SteppableCell cell : region.getCells())
         {
            int x = cell.getXIndex();
            int y = cell.getYIndex();

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

      messageToPack.getData().resetQuick();
      uncompressedByteBuffer.rewind();
      for (int i = 0; i < totalSize; i++)
         messageToPack.getData().add(uncompressedByteBuffer.get());
      messageToPack.setImageHeight(cellsPerSide);
      messageToPack.setImageWidth(cellsPerSide);
   }

   public SteppableRegionsListCollection getSteppableRegionsListCollection()
   {
      return regionCollection;
   }


   public static void main(String[] args)
   {
      new SteppableRegionsCalculationModule();
   }
}
