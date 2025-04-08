package us.ihmc.footstepPlanning;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.bodyPath.GPUAStarBodyPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.io.File;
import java.math.RoundingMode;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class FootstepPlannerCompletionCheckerTest
{
   /**
    * This test shows a case where the snapped footstep is wrong.
    * By using the {@link us.ihmc.footstepPlanning.ui.FootstepPlannerUI} we can see that the snapped step is getting set to the noise in the TerrainMapData.
    * And even though most of the heights are near the goal foot, because a few are higher, the snapping goes to that value.
    */
   @Test
   @Disabled
   public void testGoalReachedAndSnappedCorrectly()
   {
      // Load the log from resources
      // TODO can't put this in a resource cause its too large, leave a comment on where the file is stored and have them download it
      // TODO then the user can set the path correctly and load the log.
      // TODO by default can disable test
      URL footstepPlannerLogURL = getClass().getClassLoader().getResource("20250407_171648251_FootstepPlannerLog");
      File footstepPlannerLogFile = new File(footstepPlannerLogURL.getPath());
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      logLoader.load(footstepPlannerLogFile);
      FootstepPlannerLog log = logLoader.getLog();

      // Set the request packet from the log data
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setFromPacket(log.getRequestPacket());

      // Create the snapper with the required objects
      SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(defaultFootPolygons, parameters, environmentHandler);

      // Set up the environment to be correct
      environmentHandler.setHeightMap(request.getHeightMapData());
      environmentHandler.setTerrainMapData(request.getTerrainMapData());
      snapper.initialize();

      // This is the bad step we want to check
      // Pulled the lattice point from the footstep planner UI to get the bad step
      LatticePoint latticePoint = new LatticePoint(3, -18, 18);
      DiscreteFootstep badStep = new DiscreteFootstep(latticePoint, RobotSide.LEFT);
      FootstepSnapData footstepSnapData = snapper.snapFootstep(badStep);

      // Set the footstep to be at the bad position.
      PlannedFootstep footstep = new PlannedFootstep(RobotSide.LEFT);
      footstep.getFootstepPose().set(footstepSnapData.getSnappedStepTransform(badStep));

      // Position of the goal, this is what we want to try to snap too
      // In this case from using the footstep planner UI I know that the snapped step should be within 5cm of the goal in Z
      // The reason we snap so high is because of noise in the height map, but that shouldn't be where the step goes.
      Point3D position = request.getGoalFootPoses().get(RobotSide.LEFT).getPosition();

      double badStepZ = footstep.getFootstepPose().getPosition().getZ();
      double goalStepZ = position.getZ();

      double distanceBetweenSteps = Math.abs(badStepZ - goalStepZ);
      double roundedDistanceBetweenSteps = Math.round(distanceBetweenSteps * 10000.0) / 10000.0;

      Assertions.assertTrue(roundedDistanceBetweenSteps < 0.05,
                            "Distance between steps should be less than (0.05 cms), but got (" + roundedDistanceBetweenSteps + " cms).\n"
                            + "The goalStepZ was (" + goalStepZ + ") and the badStepZ was (" + badStepZ + ")");
   }
}
