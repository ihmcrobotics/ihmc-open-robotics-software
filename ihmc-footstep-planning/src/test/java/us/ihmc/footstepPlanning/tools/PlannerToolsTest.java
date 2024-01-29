package us.ihmc.footstepPlanning.tools;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.ArrayList;
import java.util.List;

public class PlannerToolsTest
{
   @Test
   public void testCollisionAlongStraightPath()
   {
      List<Pose3D> waypoints = new ArrayList<>();
      waypoints.add(new Pose3D());
      waypoints.add(new Pose3D(10.0, 0.0, 0.0, 0.0, 0.0, 0.0));

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      parameters.setBodyBoxWidth(1.0);
      parameters.setBodyBoxDepth(0.3);
      parameters.setBodyBoxHeight(1.0);
      parameters.setBodyBoxBaseZ(0.5);

      Pose3D startPose = new Pose3D(waypoints.get(0));
      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();

      // simple block in the middle of the path
      planarRegionsListGenerator.translate(5.0, 0.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertTrue(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // small block in the middle of the path that's low enough
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(5.0, 0.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 0.2);
      Assertions.assertFalse(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // big blocks just before and just after the goal
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(-0.501 - 0.5 * parameters.getBodyBoxDepth(), 0.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      planarRegionsListGenerator.identity();
      planarRegionsListGenerator.translate(10.501 + 0.5 * parameters.getBodyBoxDepth(), 0.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertFalse(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // big block that just intersects the start
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(-0.499 - 0.5 * parameters.getBodyBoxDepth(), 0.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertTrue(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // check with start pose passed a collision
      startPose.getPosition().set(7.5, 0.0, 0.0);
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(5.0, 0.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertFalse(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));
   }

   @Test
   public void testAlongLShapedPath()
   {
      List<Pose3D> waypoints = new ArrayList<>();
      waypoints.add(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      waypoints.add(new Pose3D(0.0, 5.0, 1.0, 0.0, 0.0, 0.0));
      waypoints.add(new Pose3D(5.0, 5.0, 1.0, 0.0, 0.0, 0.0));

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      parameters.setBodyBoxWidth(1.0);
      parameters.setBodyBoxDepth(0.3);
      parameters.setBodyBoxHeight(1.0);
      parameters.setBodyBoxBaseZ(0.5);

      Pose3D startPose = new Pose3D(waypoints.get(0));
      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();

      // simple block in the middle of the first part of path
      planarRegionsListGenerator.translate(0.0, 2.5, 1.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertTrue(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // simple block in the middle of the second part of path
      planarRegionsListGenerator.translate(2.5, 5.0, 1.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertTrue(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // block in the middle of the path that's just too low
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(0.0, 5.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 0.99 + parameters.getBodyBoxBaseZ());
      Assertions.assertFalse(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // block in the middle of the path that's barely high enough
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(0.0, 5.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.01 + parameters.getBodyBoxBaseZ());
      Assertions.assertTrue(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // big blocks just before and just after the goal
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(0.0, -0.501 - 0.5 * parameters.getBodyBoxDepth(), 1.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      planarRegionsListGenerator.identity();
      planarRegionsListGenerator.translate(5.01 + 0.5 * parameters.getBodyBoxDepth(), 0.0, 0.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertFalse(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // big block that just intersects the start
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(0.0, -0.499 - 0.5 * parameters.getBodyBoxDepth(), 1.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertTrue(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, Double.POSITIVE_INFINITY));

      // big block that's after the horizon length
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(2.0, 5.0, 1.0);
      planarRegionsListGenerator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 1.0);
      Assertions.assertFalse(PlannerTools.doesPathContainBodyCollisions(startPose, waypoints, planarRegionsListGenerator.getPlanarRegionsList(), parameters, 5.0));
   }

   @Test
   public void testOverDebrisDataSet()
   {
      testDataSet(DataSetIOTools.loadDataSet(DataSetName._20200624_105726_FBDemoDebris_Small));
      testDataSet(DataSetIOTools.loadDataSet(DataSetName._20200624_105955_FBDemoDebris_Medium));
      testDataSet(DataSetIOTools.loadDataSet(DataSetName._20200624_110132_FBDemoDebris_Large));
   }

   private void testDataSet(DataSet debrisDataSet)
   {
      FootstepPlanningModule footstepPlanningModule = new FootstepPlanningModule("testModule");
      footstepPlanningModule.getFootstepPlannerParameters().setCheckForBodyBoxCollisions(false);
      footstepPlanningModule.getFootstepPlannerParameters().setBodyBoxDepth(0.3);
      footstepPlanningModule.getFootstepPlannerParameters().setBodyBoxWidth(0.8);
      footstepPlanningModule.getFootstepPlannerParameters().setBodyBoxHeight(1.5);
      footstepPlanningModule.getFootstepPlannerParameters().setBodyBoxBaseZ(0.5);

      PlanarRegionsList debrisRegions = debrisDataSet.getPlanarRegionsList();

      FootstepPlannerRequest request = new FootstepPlannerRequest();

      Pose3D startPose = new Pose3D();
      Pose3D goalPose = new Pose3D();
      startPose.getPosition().set(debrisDataSet.getPlannerInput().getStartPosition());
      goalPose.getPosition().set(debrisDataSet.getPlannerInput().getGoalPosition());

      request.setStartFootPoses(footstepPlanningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), startPose);
      request.setGoalFootPoses(footstepPlanningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(debrisDataSet.getPlanarRegionsList())));

      FootstepPlannerOutput footstepPlannerOutput = footstepPlanningModule.handleRequest(request);
      boolean collisionDetected = PlannerTools.doesPathContainBodyCollisions(startPose,
                                                                             footstepPlannerOutput.getBodyPath(),
                                                                             debrisRegions,
                                                                             footstepPlanningModule.getFootstepPlannerParameters(),
                                                                             Double.POSITIVE_INFINITY);
      Assertions.assertTrue(collisionDetected);
   }
}
