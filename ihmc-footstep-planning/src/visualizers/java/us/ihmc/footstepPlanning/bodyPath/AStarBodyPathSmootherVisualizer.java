package us.ihmc.footstepPlanning.bodyPath;

import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.HeightMapDataSetName;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.perception.gpuHeightMap.HeightMapTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import static us.ihmc.pathPlanning.HeightMapDataSetName.*;

public class AStarBodyPathSmootherVisualizer
{
   private AStarBodyPathSmoother smoother;
   private SimulationConstructionSet scs;

   public void setup(HeightMapData heightMapData)
   {
      scs = new SimulationConstructionSet(new Robot("Dummy"));
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      smoother = new AStarBodyPathSmoother(new AStarBodyPathPlannerParameters(), scs, graphicsListRegistry, scs.getRootRegistry());

      if (heightMapData != null)
      {
         scs.addStaticLinkGraphics(buildHeightMapGraphics(heightMapData));
         scs.setGroundVisible(false);
      }

      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();
   }

   public void visualizeSimpleSmoothing0()
   {
      List<BodyPathLatticePoint> bodyPathLatticePoints = new ArrayList<>();
      bodyPathLatticePoints.add(new BodyPathLatticePoint(0, 0));
      bodyPathLatticePoints.add(new BodyPathLatticePoint(1, 0));
      bodyPathLatticePoints.add(new BodyPathLatticePoint(2, -1));
      bodyPathLatticePoints.add(new BodyPathLatticePoint(3, -1));

      List<Point3D> bodyPath = bodyPathLatticePoints.stream().map(p -> new Point3D(p.getX(), p.getY(), 0.0)).collect(Collectors.toList());
      setup(null);

      smoother.doSmoothing(bodyPath, null);
      scs.cropBuffer();
   }

   public void visualizeSimpleSmoothing1()
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
      scs.cropBuffer();
   }

   public void visualizeSimpleSmoothing2()
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
      scs.cropBuffer();
   }

   public void visualizeSimpleSmoothing3()
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
      scs.cropBuffer();
   }

   public void visualizeCinders1()
   {
      runDataset("cinders_1");
   }

   public void visualizeCinders2()
   {
      runDataset("cinders_2");
   }

   public void visualizeObstacles1()
   {
      runDataset("obstacles_1");
   }

   public void visualizeObstacles2()
   {
      runDataset("obstacles_2");
   }

   public void visualizeRamp()
   {
      runDataset("ramp");
   }

   public void visualizeRamp2()
   {
      runDataset("ramp_2");
   }

   public void visualizeRampAtAnAngle()
   {
      runDataset("ramp_at_an_angle");
   }

   public void visualizeStairs1()
   {
      runDataset("stairs_1");
   }

   public void visualizeStairs2()
   {
      runDataset("stairs_2");
   }

   public void visualizeStairsHuggingEdge()
   {
      runDataset("stairs_hugging_edge");
   }

   public void visualizeSteppingStones3()
   {
      runDataset("stepping_stones_3");
   }

   public void visualizeSteppingStones4()
   {
      runDataset("stepping_stones_4");
   }

   private void runDataset(String name)
   {
      FootstepPlannerLogLoader loader = new FootstepPlannerLogLoader();

      File file = new File(System.getProperty("user.home") + File.separator + "heightMapPaper" + File.separator + name);
      loader.load(file);
      FootstepPlannerLog log = loader.getLog();

      FootstepPlanningToolboxOutputStatus statusPacket = log.getStatusPacket();
      List<Pose3D> bodyPathPoseWaypoints = statusPacket.getBodyPath();
      HeightMapMessage heightMapMessage = log.getRequestPacket().getHeightMapMessage();

      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      List<Point3D> bodyPath = bodyPathPoseWaypoints.stream().map(Pose3D::getPosition).collect(Collectors.toList());

      try
      {
         setup(heightMapData);
         smoother.doSmoothing(bodyPath, heightMapData);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      scs.cropBuffer();
   }

   public void runTimingTest()
   {
      HeightMapDataSetName[] datasets = new HeightMapDataSetName[]{Obstacle_Course};
      FootstepPlannerRequest[] requests = new FootstepPlannerRequest[datasets.length];
      HeightMapData[] heightMapData = new HeightMapData[datasets.length];
      FootstepPlannerOutput output = new FootstepPlannerOutput();

      AStarBodyPathPlanner planner = new AStarBodyPathPlanner(new DefaultFootstepPlannerParameters(), new AStarBodyPathPlannerParameters(), PlannerTools.createDefaultFootPolygons());
      AStarBodyPathSmoother smoother = new AStarBodyPathSmoother(new AStarBodyPathPlannerParameters());

      for (int i = 0; i < datasets.length; i++)
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         request.setTimeout(10.0);
         request.setStartFootPoses(0.15, datasets[i].getStart());
         request.setGoalFootPoses(0.15, datasets[i].getGoal());
         requests[i] = request;

         heightMapData[i] = datasets[i].getHeightMapData();
      }

      int warmupIterations = 2;
      for (int i = 0; i < warmupIterations; i++)
      {
         for (int j = 0; j < datasets.length; j++)
         {
            planner.setHeightMapData(heightMapData[j]);
            planner.handleRequest(requests[j], output);
            smoother.doSmoothing(output.getBodyPath().stream().map(Pose3D::getPosition).collect(Collectors.toList()), heightMapData[j]);
         }
      }

      int iterations = 5;
      long initialPlanTime = 0;
      long smoothTime = 0;

      for (int i = 0; i < iterations; i++)
      {
         for (int j = 0; j < datasets.length; j++)
         {
            planner.setHeightMapData(heightMapData[j]);
            long t0 = System.currentTimeMillis();
            planner.handleRequest(requests[j], output);
            long t1 = System.currentTimeMillis();
            smoother.doSmoothing(output.getBodyPath().stream().map(Pose3D::getPosition).collect(Collectors.toList()), heightMapData[j]);
            long t2 = System.currentTimeMillis();

            initialPlanTime += (t1 - t0);
            smoothTime += (t2 - t1);
         }
      }

      long timePerInitialPlanMillis = initialPlanTime / (iterations * datasets.length);
      long timePerSmoothMillis = smoothTime / (iterations * datasets.length);
      long timePerTotalMillis = (initialPlanTime + smoothTime) / (iterations * datasets.length);

      LogTools.info("Time per initial: " + Conversions.millisecondsToSeconds(timePerInitialPlanMillis) + " sec");
      LogTools.info("Time per smooth:  " + Conversions.millisecondsToSeconds(timePerSmoothMillis) + " sec");
      LogTools.info("Time per total:   " + Conversions.millisecondsToSeconds(timePerTotalMillis) + " sec");
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

   public static void main(String[] args)
   {
//      new AStarBodyPathSmootherVisualizer().visualizeSimpleSmoothing0();
//      new AStarBodyPathSmootherVisualizer().visualizeSimpleSmoothing1();
//      new AStarBodyPathSmootherVisualizer().visualizeSimpleSmoothing2();
//      new AStarBodyPathSmootherVisualizer().visualizeSimpleSmoothing3();
//      new AStarBodyPathSmootherVisualizer().visualizeCinders1();
//      new AStarBodyPathSmootherVisualizer().visualizeCinders2();
//      new AStarBodyPathSmootherVisualizer().visualizeObstacles1();
//      new AStarBodyPathSmootherVisualizer().visualizeObstacles2();
//      new AStarBodyPathSmootherVisualizer().visualizeRamp();
//      new AStarBodyPathSmootherVisualizer().visualizeRamp2();
      new AStarBodyPathSmootherVisualizer().visualizeRampAtAnAngle();
//      new AStarBodyPathSmootherVisualizer().visualizeStairs1();
//      new AStarBodyPathSmootherVisualizer().visualizeStairs2();
//      new AStarBodyPathSmootherVisualizer().visualizeStairsHuggingEdge();
//      new AStarBodyPathSmootherVisualizer().visualizeSteppingStones3();
//      new AStarBodyPathSmootherVisualizer().visualizeSteppingStones4();
//      new AStarBodyPathSmootherVisualizer().runTimingTest();
   }
}
