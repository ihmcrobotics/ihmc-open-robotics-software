package us.ihmc.footstepPlanning.narrowPassage;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.ArrayList;

import static us.ihmc.robotics.Assert.*;

public class NarrowPassageBodyPathOptimizerTest
{
   private final double idealStanceWidth = new DefaultFootstepPlannerParameters().getIdealFootstepWidth();
   private final ExpandingPolytopeAlgorithm collisionDetector = new ExpandingPolytopeAlgorithm();
   private FootstepPlanningModule module;
   private FootstepPlannerParametersBasics footstepPlanningParameters;
   private PlanarRegionsList planarRegionsList;

   @BeforeEach
   public void setup()
   {
      FootstepPlanningModule defaultFootstepPlanningModule = new FootstepPlanningModule(getClass().getSimpleName());
      footstepPlanningParameters = defaultFootstepPlanningModule.getFootstepPlannerParameters();

      module = new FootstepPlanningModule(getClass().getSimpleName(),
                                          defaultFootstepPlanningModule.getAStarBodyPathPlannerParameters(),
                                          footstepPlanningParameters,
                                          defaultFootstepPlanningModule.getSwingPlannerParameters(),
                                          null,
                                          PlannerTools.createDefaultFootPolygons(),
                                          null);
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      module = null;
   }

   @Test
   public void testJerseyBarriers78cm()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_Jersey_Barriers_JSC_78cm);
      ArrayList<Pose3D> waypoints = runFootstepPlanningModuleTest(dataset);
      doCollisionChecks(waypoints);
   }

   @Test
   public void testJerseyBarriers65cm()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_Jersey_Barriers_IHMC_65cm);
      ArrayList<Pose3D> waypoints = runFootstepPlanningModuleTest(dataset);
      doCollisionChecks(waypoints);
   }

   @Test
   public void testJerseyBarriers60cm()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_Jersey_Barriers_JSC_60cm);
      ArrayList<Pose3D> waypoints = runFootstepPlanningModuleTest(dataset);
      doCollisionChecks(waypoints);
   }

   @Test
   public void testJerseyBarriers55cm()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20190220_172417_Jersey_Barriers_IHMC_55cm);
      ArrayList<Pose3D> waypoints = runFootstepPlanningModuleTest(dataset);
      doCollisionChecks(waypoints);
   }

   @Test
   public void testCorridor1Wall()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20191007_200400_Corridor1Wall);
      ArrayList<Pose3D> waypoints = runFootstepPlanningModuleTest(dataset);
      doCollisionChecks(waypoints);
   }

   @Test
   public void testDoor()
   {
      DataSet dataset = DataSetIOTools.loadDataSet(DataSetName._20210223_155750_Door);
      ArrayList<Pose3D> waypoints = runFootstepPlanningModuleTest(dataset);
      doCollisionChecks(waypoints);
   }

   // Test that the body-path result from running the request through the planning module is a FOUND_SOLUTION
   private ArrayList<Pose3D> runFootstepPlanningModuleTest(DataSet dataset)
   {
      PlannerInput plannerInput = dataset.getPlannerInput();
      planarRegionsList = dataset.getPlanarRegionsList();
      SideDependentList<Pose3D> startSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getStartPosition(),
                                                                                   plannerInput.getStartYaw(),
                                                                                   idealStanceWidth);
      SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getGoalPosition(), plannerInput.getGoalYaw(), idealStanceWidth);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPoses(startSteps.get(RobotSide.LEFT), startSteps.get(RobotSide.RIGHT));
      request.setGoalFootPoses(goalSteps.get(RobotSide.LEFT), goalSteps.get(RobotSide.RIGHT));
      request.setPerformAStarSearch(false);
      request.setPlanBodyPath(true);
      request.setSnapGoalSteps(false);
      request.setAssumeFlatGround(false);
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList)));

      FootstepPlannerOutput plannerOutput = module.handleRequest(request);
      assertEquals(BodyPathPlanningResult.FOUND_SOLUTION, plannerOutput.getBodyPathPlanningResult());
      ArrayList<Pose3D> waypoints = new ArrayList<>();
      for (int i = 1; i < module.getBodyPathPlan().getNumberOfWaypoints() - 1; i++)
      {
         waypoints.add((Pose3D) module.getBodyPathPlan().getWaypoint(i));
      }

      return waypoints;
   }

   // Perform collision check between bounding box at body-path waypoints and planar region environment
   private void doCollisionChecks(ArrayList<Pose3D> waypoints)
   {
      waypoints.remove(0);
      waypoints.remove(waypoints.size() - 1);
      for (int i = 0; i < waypoints.size(); i++)
      {
         boolean check = doCollisionCheck(waypoints.get(i));
         assertFalse(check);
      }
   }

   private boolean doCollisionCheck(Pose3DReadOnly waypoint)
   {
      FrameBox3D collisionBox = new FrameBox3D();
      double boxSizeX = footstepPlanningParameters.getBodyBoxDepth();
      double boxSizeY = footstepPlanningParameters.getBodyBoxWidth();
      double boxSizeZ = footstepPlanningParameters.getBodyBoxHeight();
      collisionBox.getSize().set(boxSizeX, boxSizeY, boxSizeZ);
      Vector3D boxCenterInSoleFrame = new Vector3D();
      boxCenterInSoleFrame.set(footstepPlanningParameters.getBodyBoxBaseX(), 0.0, boxSizeZ / 2.0 + footstepPlanningParameters.getBodyBoxBaseZ());

      PoseReferenceFrame waypointPoseFrame = new PoseReferenceFrame("waypointPoseFrame", ReferenceFrame.getWorldFrame());
      waypointPoseFrame.setPoseAndUpdate(waypoint);

      FramePose3D boxCenterPose = new FramePose3D(waypointPoseFrame);
      boxCenterPose.getPosition().set(boxCenterInSoleFrame);
      boxCenterPose.changeFrame(ReferenceFrame.getWorldFrame());

      collisionBox.getPose().set(boxCenterPose);
      collisionBox.getOrientation().set(waypoint.getOrientation());

      EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();
      collisionResult.setToZero();
      collisionResult.setSignedDistance(Double.POSITIVE_INFINITY);

      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);

         if (planarRegion.getBoundingBox3dInWorld().intersectsExclusive(collisionBox.getBoundingBox()))
         {
            EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
            collisionDetector.evaluateCollision(collisionBox, planarRegion, result);

            if (result.getSignedDistance() < collisionResult.getSignedDistance())
            {
               collisionResult.set(result);
            }
         }
         if (collisionResult.areShapesColliding())
            return true;
      }
      return false;
   }
}
