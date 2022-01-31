package us.ihmc.footstepPlanning.bodyPath;

import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.HeightMapMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.idl.IDLSequence;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class AStarBodyPathSmootherTest
{
   private boolean visualize = true;
   private AStarBodyPathSmoother smoother;
   private SimulationConstructionSet scs;

   public void setup(HeightMapData heightMapData)
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Dummy"));
         YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
         smoother = new AStarBodyPathSmoother(scs, graphicsListRegistry, scs.getRootRegistry());

         if (heightMapData != null)
         {
            scs.addStaticLinkGraphics(buildHeightMapGraphics(heightMapData));
            scs.setGroundVisible(false);
         }

         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.startOnAThread();
      }
      else
      {
         smoother = new AStarBodyPathSmoother();
      }
   }

   @AfterEach
   public void after()
   {
      if (visualize)
      {
//         scs.cropBuffer();
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testSimpleSmoothing0()
   {
      List<BodyPathLatticePoint> bodyPathLatticePoints = new ArrayList<>();
      bodyPathLatticePoints.add(new BodyPathLatticePoint(0, 0));
      bodyPathLatticePoints.add(new BodyPathLatticePoint(1, 0));
      bodyPathLatticePoints.add(new BodyPathLatticePoint(2, -1));
      bodyPathLatticePoints.add(new BodyPathLatticePoint(3, -1));

      List<Point3D> bodyPath = bodyPathLatticePoints.stream().map(p -> new Point3D(p.getX(), p.getY(), 0.0)).collect(Collectors.toList());
      setup(null);

      smoother.doSmoothing(bodyPath, null);
   }

   @Test
   public void testSimpleSmoothing1()
   {
      List<BodyPathLatticePoint> bodyPathLatticePoints = new ArrayList<>();
      for (int i = 0; i < 11; i++)
      {
         bodyPathLatticePoints.add(new BodyPathLatticePoint(i, 0));
      }

      double heightMapGridResolution = 0.025;
      double heightMapGridCenterX = 0.0;
      double heightMapGridCenterY = 0.0;

      double estimatedGroundPlane = 0.0;
      HeightMapData heightMapData = new HeightMapData(heightMapGridResolution, 5.0, heightMapGridCenterX, heightMapGridCenterY);
      heightMapData.setEstimatedGroundHeight(estimatedGroundPlane);

      double obstacleHeight = 0.5;
      double obstacleMinY = 0.25;
      double obstacleWidth = 0.2;
      double obstacleThickness = 0.1;

      double midX = bodyPathLatticePoints.get(5).getX();
      double minX = midX - 0.5 * obstacleThickness;
      for (double x = minX; x < minX + obstacleThickness; x += heightMapGridResolution)
      {
         for (double y = obstacleMinY; y < obstacleMinY + obstacleWidth; y += heightMapGridResolution)
         {
            heightMapData.setHeightAt(x, y, obstacleHeight);
         }
      }

      setup(heightMapData);
      List<Point3D> bodyPath = bodyPathLatticePoints.stream().map(p -> new Point3D(p.getX(), p.getY(), 0.0)).collect(Collectors.toList());
      smoother.doSmoothing(bodyPath, heightMapData);
   }

   @Test
   public void testSimpleSmoothing2()
   {
      List<BodyPathLatticePoint> bodyPathLatticePoints = new ArrayList<>();
      for (int i = 0; i < 11; i++)
      {
         bodyPathLatticePoints.add(new BodyPathLatticePoint(0, i));
      }

      double heightMapGridResolution = 0.025;
      double heightMapGridCenterX = 0.0;
      double heightMapGridCenterY = 0.0;

      double estimatedGroundPlane = 0.0;
      HeightMapData heightMapData = new HeightMapData(heightMapGridResolution, 5.0, heightMapGridCenterX, heightMapGridCenterY);
      heightMapData.setEstimatedGroundHeight(estimatedGroundPlane);

      double obstacleHeight = 0.5;
      double obstacleMinX = 0.25;
      double obstacleWidth = 0.2;
      double obstacleThickness = 0.1;

      double midY = bodyPathLatticePoints.get(5).getY();
      double minY = midY - 0.5 * obstacleThickness;
      for (double x = obstacleMinX; x < obstacleMinX + obstacleWidth; x += heightMapGridResolution)
      {
         for (double y = minY; y < minY + obstacleThickness; y += heightMapGridResolution)
         {
            heightMapData.setHeightAt(x, y, obstacleHeight);
         }
      }

      setup(heightMapData);
      List<Point3D> bodyPath = bodyPathLatticePoints.stream().map(p -> new Point3D(p.getX(), p.getY(), 0.0)).collect(Collectors.toList());
      smoother.doSmoothing(bodyPath, heightMapData);
   }

   @Test
   public void testSimpleSmoothing3()
   {
      List<BodyPathLatticePoint> bodyPathLatticePoints = new ArrayList<>();
      for (int i = 0; i < 11; i++)
      {
         bodyPathLatticePoints.add(new BodyPathLatticePoint(i, i));
      }

      double heightMapGridResolution = 0.025;
      double heightMapGridCenterX = 0.0;
      double heightMapGridCenterY = 0.0;

      double estimatedGroundPlane = 0.0;
      HeightMapData heightMapData = new HeightMapData(heightMapGridResolution, 5.0, heightMapGridCenterX, heightMapGridCenterY);
      heightMapData.setEstimatedGroundHeight(estimatedGroundPlane);

      double obstacleHeight = 0.5;
      double obstacleMinX = bodyPathLatticePoints.get(5).getX() + 0.2;
      double obstacleWidth = 0.2;
      double obstacleThickness = 0.1;

      double midY = bodyPathLatticePoints.get(5).getY();
      double minY = midY - 0.5 * obstacleThickness;
      for (double x = obstacleMinX; x < obstacleMinX + obstacleWidth; x += heightMapGridResolution)
      {
         for (double y = minY; y < minY + obstacleThickness; y += heightMapGridResolution)
         {
            heightMapData.setHeightAt(x, y, obstacleHeight);
         }
      }

      setup(heightMapData);
      List<Point3D> bodyPath = bodyPathLatticePoints.stream().map(p -> new Point3D(p.getX(), p.getY(), 0.0)).collect(Collectors.toList());
      smoother.doSmoothing(bodyPath, heightMapData);
   }

   @Test
   public void testDataset0()
   {
      runDataset(0);
   }

   @Test
   public void testDataset1()
   {
      runDataset(1);
   }

   @Test
   public void testDataset2()
   {
      runDataset(2);
   }

   private void runDataset(int datasetIndex)
   {
      FootstepPlannerLogLoader loader = new FootstepPlannerLogLoader();

      File file = new File(System.getProperty("user.home") + File.separator + "heightMapDatasets" + File.separator + "dataset" + datasetIndex);
      loader.load(file);
      FootstepPlannerLog log = loader.getLog();

      FootstepPlanningToolboxOutputStatus statusPacket = log.getStatusPacket();
      List<Pose3D> bodyPathPoseWaypoints = statusPacket.getBodyPath();
      HeightMapMessage heightMapMessage = log.getRequestPacket().getHeightMapMessage();

      HeightMapData heightMapData = new HeightMapData(heightMapMessage);
      List<Point3D> bodyPath = bodyPathPoseWaypoints.stream().map(Pose3D::getPosition).collect(Collectors.toList());

      setup(heightMapData);
      smoother.doSmoothing(bodyPath, heightMapData);
   }

   private static Graphics3DObject buildHeightMapGraphics(HeightMapData heightMapData)
   {
      Graphics3DObject graphics3DObject = new Graphics3DObject();

      double groundPlaneThickness = 0.01;
      graphics3DObject.translate(heightMapData.getGridCenter().getX(), heightMapData.getGridCenter().getY(), heightMapData.getEstimatedGroundHeight());
      graphics3DObject.translate(0.0, 0.0, - groundPlaneThickness);
      graphics3DObject.addCube(heightMapData.getGridSizeXY(), heightMapData.getGridSizeXY(), groundPlaneThickness, YoAppearance.Blue());
      graphics3DObject.addCoordinateSystem(0.3);

      double groundPlaneHeight = heightMapData.getEstimatedGroundHeight();

      int gridWidth = 2 * heightMapData.getCenterIndex() + 1;
      for (int key = 0; key < gridWidth * gridWidth; key++)
      {
         Point2D cellPosition = new Point2D(HeightMapTools.keyToXCoordinate(key,
                                                                            heightMapData.getGridCenter().getX(),
                                                                            heightMapData.getGridResolutionXY(),
                                                                            heightMapData.getCenterIndex()),
                                            HeightMapTools.keyToYCoordinate(key,
                                                                            heightMapData.getGridCenter().getY(),
                                                                            heightMapData.getGridResolutionXY(),
                                                                            heightMapData.getCenterIndex()));
         double height = heightMapData.getHeightAt(HeightMapTools.keyToXIndex(key, heightMapData.getCenterIndex()), HeightMapTools.keyToYIndex(key, heightMapData.getCenterIndex()));

         double renderedHeight = height - groundPlaneHeight;
         graphics3DObject.identity();
         graphics3DObject.translate(cellPosition.getX(), cellPosition.getY(), groundPlaneHeight + 0.5 * renderedHeight);

         if (height > groundPlaneHeight + 1e-5)
         {
            AppearanceDefinition color = YoAppearance.Olive();
            graphics3DObject.addCube(heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), renderedHeight, true, color);
         }
      }

      return graphics3DObject;
   }
}
