package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import org.junit.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlannerStart;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

import static junit.framework.TestCase.assertTrue;

public class QuadrupedAStarFootstepPlannerTest
{
   private static final long timeout = 30000;
   private static final boolean visualize = true;

   @Test(timeout = timeout)
   public void test()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      QuadrupedXGaitSettings xGaitSettings = new QuadrupedXGaitSettings();
      xGaitSettings.setStanceLength(1.0);
      xGaitSettings.setStanceWidth(0.5);
      FootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      QuadrupedAStarFootstepPlanner planner = QuadrupedAStarFootstepPlanner.createPlanner(parameters, xGaitSettings, null, expansion, registry);

      FramePose3D startPose = new FramePose3D();
      FramePose3D goalPose = new FramePose3D();
      goalPose.setPosition(2.0, 0.0, 0.0);

      QuadrupedFootstepPlannerStart start = new QuadrupedFootstepPlannerStart();
      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      start.setStartPose(startPose);
      goal.setGoalPose(goalPose);

      planner.setStart(start);
      planner.setGoal(goal);
      planner.setTimeout(10.0);

      FootstepPlanningResult result = planner.plan();

      assertTrue(result.validForExecution());
      List<? extends QuadrupedTimedStep> steps = planner.getSteps();

      if (visualize)
         visualizePlan(steps, null, startPose.getPosition(), goalPose.getPosition());

   }

   private void visualizePlan(List<? extends QuadrupedTimedStep> steps, PlanarRegionsList planarRegionsList, Point3DReadOnly start, Point3DReadOnly goal)
   {
      if (!visualize || ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         return;

      SimulationConstructionSet scs = new SimulationConstructionSet();

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      if (planarRegionsList != null)
         Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, planarRegionsList, YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray());
      scs.setGroundVisible(false);

      graphics3DObject.identity();
      graphics3DObject.translate(start);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Blue());

      graphics3DObject.identity();
      graphics3DObject.translate(goal);
      graphics3DObject.translate(0.0, 0.0, 0.05);
      graphics3DObject.addCone(0.3, 0.05, YoAppearance.Red());

      if (steps != null)
      {
         for (int i = 0; i < steps.size(); i++)
         {
            Point3DReadOnly point = steps.get(i).getGoalPosition();

            graphics3DObject.identity();
            graphics3DObject.translate(point);
            graphics3DObject.addSphere(0.1, YoAppearance.Orange());

            if (i != steps.size() - 1)
            {
               Point3DReadOnly nextPoint = steps.get(i + 1).getGoalPosition();
               Vector3D direction = new Vector3D(nextPoint);
               direction.sub(point);
               int pathPoints = (int) Math.round(point.distance(nextPoint) / 0.05);

               for (int j = 1; j < pathPoints; j++)
               {
                  Vector3D offset = new Vector3D(direction);
                  offset.scaleAdd(((double) j) / pathPoints, point);

                  graphics3DObject.identity();
                  graphics3DObject.translate(offset);
                  graphics3DObject.addSphere(0.025, YoAppearance.Orange());
               }
            }
         }
      }

      scs.addStaticLinkGraphics(graphics3DObject);

      scs.setCameraPosition(-15, -1.0, 25.0);
      scs.setCameraFix(-10, 0.0, 0.0);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }
}
