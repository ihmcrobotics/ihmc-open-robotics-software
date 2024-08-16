package us.ihmc.footstepPlanning.bodyPath;

import org.junit.jupiter.api.*;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Predicate;

@Disabled
public class AStarBodyPathFrameworkTest
{
   private static final double heightMapResolution = 0.04;

   // Threshold used to assert that the body path starts and ends where we asked it to.
   private static final double START_GOAL_EPSILON = 1.0e-1;

   // Whether to start the UI or not.
   private static boolean ENABLE_TIMERS = true;
   private static boolean GENERATE_LOG_FOR_FAILING_TESTS = true;

   private static final double walkerTotalTime = 300.0;


   private static final List<DataSetName> fastDatasets = Arrays.asList(DataSetName._20190514_163532_QuadrupedShortPlatformEnvironment,
                                                                       DataSetName._20190514_163532_QuadrupedPlatformEnvironment,
                                                                       DataSetName._20171114_135559_PartialShallowMaze,
                                                                       DataSetName._20171115_171243_SimplePlaneAndWall,
                                                                       DataSetName._20171215_201810_RampSteppingStones_Sim);

   // For enabling helpful prints.
   private static boolean DEBUG = true;


   // The following are used for collision checks.
   private static final double walkerOffsetHeight = 0.75;
   private static final Vector3D walkerRadii = new Vector3D(0.25, 0.25, 0.5);
   protected static double walkerMarchingSpeed = 0.25;

   // For the occlusion test
   private final FootstepPlanningModule planningModule = new FootstepPlanningModule("testModule");
   private final FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);

   @BeforeEach
   public void setup()
   {
      DEBUG = (DEBUG && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      GENERATE_LOG_FOR_FAILING_TESTS &= !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      planningModule.destroy();
   }

   @Test
   public void testDatasetsWithoutOcclusion()
   {
      Predicate<DataSet> dataSetFilter = dataSet ->
      {
         if(!dataSet.hasPlannerInput())
            return false;

         return dataSet.getPlannerInput().getVisGraphIsTestable();
      };
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSetFilter);
      runAssertionsOnAllDatasets(dataSets, this::runAssertionsWithoutOcclusion);
   }

   @Test
   @Tag("fast")
   public void testFewDataSetsNoOcclussionsSimulateDynamicReplanning()
   {
      Predicate<DataSet> dataSetFilter = dataSet ->
      {
         if(!dataSet.hasPlannerInput())
            return false;
         else if (!dataSet.getPlannerInput().getVisGraphIsTestable())
            return false;

         for (DataSetName namesToTest : fastDatasets)
         {
            if (dataSet.getName().equals(namesToTest.name().substring(1)))
               return true;
         }

         return false;
      };
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSetFilter);
      runAssertionsOnAllDatasets(dataSets, dataset -> runAssertionsSimulateDynamicReplanning(dataset, walkerMarchingSpeed, 10000));
   }

   @Test
   @Tag("path-planning-slow")
   public void testDatasetsNoOcclusionSimulateDynamicReplanning()
   {

      Predicate<DataSet> dataSetFilter = dataSet ->
      {
         if(!dataSet.hasPlannerInput())
            return false;
         else if (!dataSet.getPlannerInput().getVisGraphIsTestable())
            return false;

         for (DataSetName nameToIgnore : fastDatasets)
         {
            if (dataSet.getName().equals(nameToIgnore.name().substring(1)))
               return false;
         }

         return true;
      };
      List<DataSet> allDatasets = DataSetIOTools.loadDataSets(dataSetFilter);

      runAssertionsOnAllDatasets(allDatasets, dataset -> runAssertionsSimulateDynamicReplanning(dataset, walkerMarchingSpeed, 10000));
   }


   private void runAssertionsOnAllDatasets(List<DataSet> allDatasets, Function<DataSet, String> dataSetTester)
   {
      if (DEBUG)
      {
         LogTools.info("Unit test files found: " + allDatasets.size());
         for (int i = 0; i < allDatasets.size(); i++)
            System.out.println("\t" + allDatasets.get(i).getName());
      }

      int numberOfFailingDatasets = 0;
      String errorMessages = "";

      int currentDatasetIndex = 0;
      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      DataSet dataset = allDatasets.get(currentDatasetIndex);

      while (dataset != null)
      {
         if (DEBUG)
         {
            LogTools.info("Processing file: " + dataset.getName());
         }

         String errorMessagesForCurrentFile = dataSetTester.apply(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
            numberOfFailingDatasets++;
         errorMessages += errorMessagesForCurrentFile;

         if (DEBUG)
         {
            LogTools.info("Finished processing file: " + dataset.getName());
         }

            currentDatasetIndex++;
            if (currentDatasetIndex < allDatasets.size())
               dataset = allDatasets.get(currentDatasetIndex);
            else
               dataset = null;



         ThreadTools.sleep(100); // Apparently need to give some time for the prints to appear in the right order.
      }

      Assert.assertTrue("Number of failing datasets: " + numberOfFailingDatasets + " out of " + allDatasets.size() + ". Errors:" + errorMessages,
                        errorMessages.isEmpty());
   }

   private void runAssertionsOnDataset(Function<DataSet, String> dataSetTester, DataSetName datasetname)
   {
      DataSet dataSetToTest = DataSetIOTools.loadDataSet(datasetname);

      if (DEBUG)
      {
         LogTools.info("Processing file: " + dataSetToTest.getName());
      }

      String errorMessages = dataSetTester.apply(dataSetToTest);

//      Assert.assertTrue("Errors: " + errorMessages, errorMessages.isEmpty());
      LogTools.info("Finished testing.");
//      ThreadTools.sleepForever(); // Apparently need to give some time for the prints to appear in the right order.
   }

   private String runAssertionsWithoutOcclusion(DataSet dataset)
   {
      String datasetName = dataset.getName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();
      HeightMapMessage heightMap = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList, heightMapResolution);
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMap);

      PlannerInput plannerInput = dataset.getPlannerInput();
      Point3D start = plannerInput.getStartPosition();
      Point3D goal = plannerInput.getGoalPosition();

      String errorMessages = calculateAndTestAStarBodyPath(datasetName, start, goal, planarRegionsList, heightMapData);

      return addPrefixToErrorMessages(datasetName, errorMessages);
   }

   private String runAssertionsSimulateDynamicReplanning(DataSet dataset, double walkerSpeed, long maxSolveTimeInMilliseconds)
   {
      String datasetName = dataset.getName();

      PlanarRegionsList planarRegionsList = dataset.getPlanarRegionsList();
      HeightMapMessage heightMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList, heightMapResolution);
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      HashMap<PlanarRegion, List<Point3D>> pointsInRegions = new HashMap<>();
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
         pointsInRegions.put(planarRegion, new ArrayList<>());

      PlannerInput plannerInput = dataset.getPlannerInput();
      Point3D start = plannerInput.getStartPosition();
      Point3D goal = plannerInput.getGoalPosition();
      AtomicReference<Boolean> stopWalkerRequest = null;


      String errorMessages = "";

      List<Pose3DReadOnly> latestBodyPath = new ArrayList<>();

      Point3D walkerPosition = new Point3D(start);

      long totalStartTime = System.currentTimeMillis();

      while (!walkerPosition.geometricallyEquals(goal, 1.0e-2))
      {
         long startTime = System.currentTimeMillis();
         errorMessages += calculateAndTestAStarBodyPath(datasetName, walkerPosition, goal, planarRegionsList, heightMapData);
         long endTime = System.currentTimeMillis();
         if (ENABLE_TIMERS)
         {
            if (endTime - startTime > maxSolveTimeInMilliseconds)
               errorMessages += fail(datasetName, "Took too long to compute a new body path.");

            if (endTime - totalStartTime > Conversions.secondsToMilliseconds(walkerTotalTime))
               errorMessages += fail(datasetName, "Took too long to make it through the body path. Made it to " + walkerPosition + ", while the goal was " + goal);
         }

         if (!errorMessages.isEmpty())
            return addPrefixToErrorMessages(datasetName, errorMessages);


         if (stopWalkerRequest != null && stopWalkerRequest.get())
         {
            break;
         }

         walkerPosition.set(travelAlongBodyPath(walkerSpeed, latestBodyPath));
      }

      return addPrefixToErrorMessages(datasetName, errorMessages);
   }

   private static Point3DReadOnly travelAlongBodyPath(double distanceToTravel, List<Pose3DReadOnly> bodyPath)
   {
      double totalDesiredDistance = distanceToTravel;
      Point3D initialPosition = new Point3D(bodyPath.get(0).getPosition());

      Point3D positionWithShift = new Point3D();

      for (int i = 0; i < bodyPath.size() - 1; i++)
      {
         LineSegment3D segment = new LineSegment3D(bodyPath.get(i).getPosition(), bodyPath.get(i + 1).getPosition());
         if (segment.length() < 5e-3)
            continue;

         if (xyDistance(segment, initialPosition) < 1.0e-2)
         {
            Vector3DBasics segmentDirection = segment.getDirection(true);
            positionWithShift.scaleAdd(distanceToTravel, segmentDirection, initialPosition);

            double distanceFromStart = bodyPath.get(0).getPosition().distanceXY(positionWithShift);
            if (xyDistance(segment, positionWithShift) < 1.0e-2 && distanceFromStart >= totalDesiredDistance)
            {
               initialPosition = new Point3D(positionWithShift);
               break;
            }
            else
            {
               distanceToTravel = Math.max(distanceToTravel - initialPosition.distanceXY(segment.getSecondEndpoint()), 0.1);
               initialPosition = new Point3D(segment.getSecondEndpoint());
            }
         }
      }

      if (initialPosition.getZ() > 2.0)
         return initialPosition;
      else
         return initialPosition;
   }

   private static double xyDistance(LineSegment3DReadOnly segment, Point3DReadOnly point)
   {
      Point3DReadOnly lineSegmentStart = segment.getFirstEndpoint();
      Point3DReadOnly lineSegmentEnd = segment.getSecondEndpoint();
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point.getX(), point.getY(), lineSegmentStart.getX(), lineSegmentStart.getY(), lineSegmentEnd.getX(), lineSegmentEnd.getY());
   }

   private String calculateAndTestAStarBodyPath(String datasetName,
                                                Point3D start,
                                                Point3D goal,
                                                PlanarRegionsList planarRegionsList,
                                                HeightMapData heightMapData)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setHeightMapData(heightMapData);
      request.setPlanBodyPath(true);
      request.setPlanFootsteps(false);
      request.setAssumeFlatGround(false);

      for(RobotSide robotSide : RobotSide.values())
      {
         request.getStartFootPoses().get(robotSide).getPosition().set(start);
         request.getStartFootPoses().get(robotSide).getOrientation().setToZero();
         request.getGoalFootPoses().get(robotSide).getPosition().set(goal);
         request.getGoalFootPoses().get(robotSide).getOrientation().setToZero();
         request.getStartFootPoses().get(robotSide).appendTranslation(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         request.getGoalFootPoses().get(robotSide).appendTranslation(0.0, robotSide.negateIfRightSide(0.1), 0.0);
      }
      request.setTimeout(50.0);

      planningModule.handleRequest(request);

      FootstepPlannerOutput output = planningModule.getOutput();
      List<Pose3D> path = output.getBodyPath();

      if (GENERATE_LOG_FOR_FAILING_TESTS)
      {
         logger.logSession();
      }

      String errorMessages = basicBodyPathSanityChecks(datasetName, start, goal, path);

      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when path does not pass the basic sanity checks.


      // "Walk" along the body path and assert that the walker does not go through any region.
      int currentSegmentIndex = 0;
      Point3DReadOnly pathStart = path.get(0).getPosition();
      Point3DReadOnly pathEnd = path.get(path.size() - 1).getPosition();
      Point3D walkerCurrentPosition = new Point3D(pathStart);
      List<Point3D> collisions = new ArrayList<>();
      Ellipsoid3D walkerShape = new Ellipsoid3D();
      walkerShape.getRadii().set(walkerRadii);

      Point3D walkerBody3D = new Point3D(walkerCurrentPosition);
      walkerBody3D.addZ(walkerOffsetHeight);
      walkerShape.getPosition().set(walkerBody3D);

      errorMessages += walkerCollisionChecks(datasetName, walkerShape, planarRegionsList, collisions);

      while (!walkerCurrentPosition.geometricallyEquals(pathEnd, 1.0e-2))
      {
         walkerBody3D = new Point3D(walkerCurrentPosition);
         walkerBody3D.addZ(walkerOffsetHeight);
         walkerShape.getPosition().set(walkerBody3D);

         errorMessages += walkerCollisionChecks(datasetName, walkerShape, planarRegionsList, collisions);

         //         walkerCurrentPosition.set(travelAlongBodyPath(walkerMarchingSpeed, walkerCurrentPosition, path));
         Point3DReadOnly segmentStart = path.get(currentSegmentIndex).getPosition();
         Point3DReadOnly segmentEnd = path.get(currentSegmentIndex + 1).getPosition();


         Vector3D segmentDirection = new Vector3D();
         segmentDirection.sub(segmentEnd, segmentStart);
         segmentDirection.normalize();
         if (segmentDirection.containsNaN() || segmentDirection.length() < 1e-2)
         {
            currentSegmentIndex++;
            continue;
         }
         walkerCurrentPosition.scaleAdd(walkerMarchingSpeed, segmentDirection, walkerCurrentPosition);
         if (segmentStart.distance(segmentEnd) < segmentStart.distance(walkerCurrentPosition))
         {
            walkerCurrentPosition.set(segmentEnd);
            currentSegmentIndex++;
         }
      }

      return errorMessages;
   }

   private String walkerCollisionChecks(String datasetName, Ellipsoid3D walkerShapeWorld, PlanarRegionsList planarRegionsList, List<Point3D> collisionsToPack)
   {
      String errorMessages = "";
      walkerShapeWorld = new Ellipsoid3D(walkerShapeWorld); // Make a copy to ensure we are not modifying the argument

//      NavigableRegionFilter navigableFilter = visibilityGraphsParameters.getNavigableRegionFilter();
//      PlanarRegionFilter planarRegionFilter = visibilityGraphsParameters.getPlanarRegionFilter();
//      List<PlanarRegion> regionsToCheck = planarRegionsList.getPlanarRegionsAsList().stream().filter(query -> navigableFilter.isPlanarRegionNavigable(query, planarRegionsList.getPlanarRegionsAsList())).collect(Collectors.toList());
//      regionsToCheck = regionsToCheck.stream().filter(planarRegionFilter::isPlanarRegionRelevant).collect(Collectors.toList());
      List<PlanarRegion> regionsToCheck = planarRegionsList.getPlanarRegionsAsList();

      for (PlanarRegion planarRegion : regionsToCheck)
      {
         Point3D walkerPosition3D = new Point3D(walkerShapeWorld.getPosition());

         Plane3D plane = planarRegion.getPlane();
         Point3DBasics closestPoint = plane.orthogonalProjectionCopy(walkerPosition3D);

         if (!walkerShapeWorld.isPointInside(closestPoint))
            continue; // Not even close to the region plane, let's keep going.

         Ellipsoid3D walkerShapeLocal = new Ellipsoid3D(walkerShapeWorld);
         planarRegion.transformFromWorldToLocal(walkerShapeLocal);
         walkerPosition3D.set(walkerShapeLocal.getPosition());
         Point2D walkerPosition2D = new Point2D(walkerPosition3D);

         if (planarRegion.getNumberOfConvexPolygons() == 0)
         {
            List<Point2DReadOnly> concaveHullVertices = new ArrayList<>(planarRegion.getConcaveHull());
            double depthThreshold = 0.05;
            List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
            ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullVertices, depthThreshold, convexPolygons);
         }

         for (int i = 0; i < planarRegion.getNumberOfConvexPolygons(); i++)
         {
            ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(i);
            Point2DBasics closestPoint2D = convexPolygon.orthogonalProjectionCopy(walkerPosition2D);
            if (closestPoint2D == null)
            {
               if (convexPolygon.isPointInside(walkerPosition2D))
               {
                  collisionsToPack.add(walkerPosition3D);
                  errorMessages += fail(datasetName, "Body path is going through a region."); // TODO figure out the proper intersection
                  break;
               }
               else
               {
                  throw new RuntimeException("Not sure what went wrong to here.");
               }
            }
            closestPoint = new Point3D(closestPoint2D);

            if (walkerShapeLocal.isPointInside(closestPoint))
            {
               Point2DBasics intersectionLocal = closestPoint2D;
               Point3D intersectionWorld = new Point3D(intersectionLocal);
               planarRegion.transformFromLocalToWorld(intersectionWorld);
               errorMessages += fail(datasetName, "Walker is going through a region at: " + intersectionWorld);
               collisionsToPack.add(intersectionWorld);
            }
         }
      }
      return errorMessages;
   }

   private String basicBodyPathSanityChecks(String datasetName, Point3DReadOnly start, Point3DReadOnly goal, List<? extends Pose3DReadOnly> path)
   {
      String errorMessages = "";
      errorMessages += assertTrue(datasetName, "Path is null!", path != null);
      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when no path is returned.

      errorMessages += assertTrue(datasetName, "Path does not contain any waypoints", path.size() > 0);

      if (!errorMessages.isEmpty())
         return errorMessages; // Cannot test anything else when no path is returned.

      Point3DReadOnly pathEnd = path.get(path.size() - 1).getPosition();
      Point3DReadOnly pathStart = path.get(0).getPosition();

      Point2DReadOnly pathEnd2D = new Point2D(pathEnd);
      Point2DReadOnly pathStart2D = new Point2D(pathStart);

      Point2DReadOnly goal2D = new Point2D(goal);
      Point2DReadOnly start2D = new Point2D(start);

      errorMessages += assertTrue(datasetName, "Body path does not end at desired goal position: desired = " + goal + ", actual = " + pathEnd,
                                  pathEnd2D.geometricallyEquals(goal2D, START_GOAL_EPSILON));
      errorMessages += assertTrue(datasetName, "Body path does not start from desired start position: desired = " + start + ", actual = " + pathStart,
                                  pathStart2D.geometricallyEquals(start2D, START_GOAL_EPSILON));

      return errorMessages;
   }

   private static String addPrefixToErrorMessages(String datasetName, String errorMessages)
   {
      if (!errorMessages.isEmpty())
         return "\n" + datasetName + errorMessages;
      else
         return "";
   }

   private String fail(String datasetName, String message)
   {
      return assertTrue(datasetName, message, false);
   }

   private String assertTrue(String datasetName, String message, boolean condition)
   {
      if (DEBUG)
      {
         if (!condition)
            LogTools.error(datasetName + ": " + message);
      }
      return !condition ? "\n" + message : "";
   }

   public static void main(String[] args) throws Exception
   {
      AStarBodyPathFrameworkTest test = new AStarBodyPathFrameworkTest();

      DataSetName dataSetName = DataSetName._20191122_122216_MultiRegionFlatGround;

//      VISUALIZE = true;
//      test.setup();
//      if (VISUALIZE)
//      {
//         messager.submitMessage(UIVisibilityGraphsTopics.EnableWalkerAnimation, false);
//         messager.submitMessage(UIVisibilityGraphsTopics.WalkerOffsetHeight, walkerOffsetHeight);
//         messager.submitMessage(UIVisibilityGraphsTopics.WalkerSize, walkerRadii);
//         messager.submitMessage(UIVisibilityGraphsTopics.ShowInterRegionVisibilityMap, true);
//
//      }
//      test.runAssertionsOnDataset(dataset -> test.runAssertionsSimulateDynamicReplanning(dataset, walkerMarchingSpeed, 5000, false), dataSetName);
//      for (int i = 0; i < 50; i++)
      {
         test.runAssertionsOnDataset(test::runAssertionsWithoutOcclusion, dataSetName);
      }
      test.tearDown();

      Predicate<DataSet> dataSetFilter = dataSet ->
      {
         if(!dataSet.hasPlannerInput())
            return false;

         return dataSet.getPlannerInput().getVisGraphIsTestable();
      };
//      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSetFilter);
//      test.runAssertionsOnAllDatasets(dataSets, test::runAssertionsWithoutOcclusion);
//      test.tearDown();
   }
}
