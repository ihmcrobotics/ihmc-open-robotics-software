package us.ihmc.perception;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMap;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapUpdater;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class GPUHeightMapVisualizer
{
   public GPUHeightMapVisualizer(SimpleGPUHeightMap heightMap)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(buildHeightMapGraphics(heightMap));

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private static Graphics3DObject buildHeightMapGraphics(SimpleGPUHeightMap heightMapData)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();

      graphics3DObject.translate(heightMapData.getCenter().getX(), heightMapData.getCenter().getY(), 0.0);
      graphics3DObject.addCoordinateSystem(0.3);



      int mapElements = heightMapData.getCellsPerSide() * heightMapData.getCellsPerSide();
      for (int key = 0; key < mapElements; key++)
      {
         Point2D cellPosition = new Point2D(heightMapData.getCellX(key), heightMapData.getCellY(key));
         double height = heightMapData.getCellZ(key);
         double variance = heightMapData.getVariance(key);

         double alpha = variance / 20.0;


         graphics3DObject.identity();
         graphics3DObject.translate(cellPosition.getX(), cellPosition.getY(), 0.5 * height);

         Color blue = Color.BLUE;
         Color red = Color.red;

         AppearanceDefinition color = interpolateColor(blue, red, alpha);

         graphics3DObject.addCube(heightMapData.getResolution(), heightMapData.getResolution(), height, true, color);
      }

      return graphics3DObject;
   }

   private static AppearanceDefinition interpolateColor(Color a, Color b, double alpha)
   {
      float[] aColor = a.getRGBComponents(null);
      float[] bColor = b.getRGBComponents(null);
      float r = (float) InterpolationTools.linearInterpolate(aColor[0], bColor[0], alpha);
      float g =  (float) InterpolationTools.linearInterpolate(aColor[1], bColor[1], alpha);
      float d =  (float) InterpolationTools.linearInterpolate(aColor[2], bColor[2], alpha);

      return YoAppearance.RGBColor(r, g, d);
   }

   public static void main(String[] args)
   {
      SimpleGPUHeightMapParameters parameters = new SimpleGPUHeightMapParameters();
      parameters.mapLength = 2.0;
      parameters.resolution = 0.04;
      SimpleGPUHeightMapUpdater gpuHeightMap = new SimpleGPUHeightMapUpdater(parameters);

      Random random = new Random(1738L);
      List<Point3D> pointsToAdd = new ArrayList<>();
      for (int i = 0; i < 3000; i++)
      {
         Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 0.5 * parameters.mapLength);
         point.setZ(RandomNumbers.nextDouble(random, 0.1));
         pointsToAdd.add(point);
      }

      gpuHeightMap.inputFromPointCloud(pointsToAdd, new RigidBodyTransform());

      new GPUHeightMapVisualizer(gpuHeightMap.getHeightMap());
   }
}
