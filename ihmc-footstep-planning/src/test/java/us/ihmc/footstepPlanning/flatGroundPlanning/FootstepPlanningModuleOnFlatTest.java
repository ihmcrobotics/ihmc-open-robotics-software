package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl.global.OpenCL;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.simplePlanners.FlatGroundPlanningUtils;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class FootstepPlanningModuleOnFlatTest
{
   static
   {
      // This is required to load the javacpp shared library immediately.
      // We allocate native memory very early (inline in fields) before OpenCLManager is initialized, so this is required.
      Loader.load(OpenCL.class);
   }

   private final Random random = new Random(727434726273L);
   private static final double stepWidth = 0.3;
   private final double idealStanceWidth = new DefaultFootstepPlannerParameters().getIdealFootstepWidth();

   @Test
   public void testStraightLine()
   {
      double xGoal = 5.0;
      double yGoal = - stepWidth / 2.0;
      double yawGoal = 0.0;
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2D initialMidFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialMidFootPose2d = new FramePose2D(ReferenceFrame.getWorldFrame(), initialMidFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialMidFootPose = FlatGroundPlanningUtils.poseFormPose2d(initialMidFootPose2d);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setAssumeFlatGround(true);
      request.setGoalFootPoses(idealStanceWidth, goalPose3d);
      request.setRequestedInitialStanceSide(initialStanceFootSide);
      request.setStartFootPoses(idealStanceWidth, initialMidFootPose);

      runTest(goalPose3d, request);
   }

   @Test
   public void testATightTurn()
   {
      double xGoal = 1.0;
      double yGoal = 0.5;
      double yawGoal = 0.0;
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2D initialMidFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialMidFootPose2d = new FramePose2D(ReferenceFrame.getWorldFrame(), initialMidFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialMidFootPose = FlatGroundPlanningUtils.poseFormPose2d(initialMidFootPose2d);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setAssumeFlatGround(true);
      request.setGoalFootPoses(idealStanceWidth, goalPose3d);
      request.setRequestedInitialStanceSide(initialStanceFootSide);
      request.setStartFootPoses(idealStanceWidth, initialMidFootPose);

      runTest(goalPose3d, request);
   }

   @Test
   public void testStraightLineWithInitialTurn()
   {
      double xGoal = 5.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = Math.toRadians(20.0);
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = Math.toRadians(20.0);
      Point2D initialMidFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialMidFootPose2d = new FramePose2D(ReferenceFrame.getWorldFrame(), initialMidFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialMidFootPose = FlatGroundPlanningUtils.poseFormPose2d(initialMidFootPose2d);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setAssumeFlatGround(true);
      request.setGoalFootPoses(idealStanceWidth, goalPose3d);
      request.setRequestedInitialStanceSide(initialStanceFootSide);
      request.setStartFootPoses(idealStanceWidth, initialMidFootPose);

      runTest(goalPose3d, request);
   }

   @Test
   public void testJustTurnInPlace()
   {
      double xGoal = 0.0;
      double yGoal = -stepWidth/2.0;
      double yawGoal = Math.toRadians(160.0);
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2D initialMidFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialMidFootPose2d = new FramePose2D(ReferenceFrame.getWorldFrame(), initialMidFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialMidFootPose = FlatGroundPlanningUtils.poseFormPose2d(initialMidFootPose2d);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setAssumeFlatGround(true);
      request.setGoalFootPoses(idealStanceWidth, goalPose3d);
      request.setRequestedInitialStanceSide(initialStanceFootSide);
      request.setStartFootPoses(idealStanceWidth, initialMidFootPose);

      runTest(goalPose3d, request);
   }

   @Test
   public void testRandomPoses()
   {
      double xGoal = random.nextDouble();
      double yGoal = random.nextDouble();
      double yawGoal = 0.0;
      Point2D goalPosition = new Point2D(xGoal, yGoal);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = random.nextDouble();
      double yInitialStanceFoot = random.nextDouble();
      double yawInitial = 0.0;
      Point2D initialMidFootPosition = new Point2D(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2D initialMidFootPose2d = new FramePose2D(ReferenceFrame.getWorldFrame(), initialMidFootPosition, yawInitial);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialMidFootPose = FlatGroundPlanningUtils.poseFormPose2d(initialMidFootPose2d);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setAssumeFlatGround(true);
      request.setGoalFootPoses(idealStanceWidth, goalPose3d);
      request.setRequestedInitialStanceSide(initialStanceFootSide);
      request.setStartFootPoses(idealStanceWidth, initialMidFootPose);

      runTest(goalPose3d, request);
   }

   private void runTest(FramePose3D goalPose3d, FootstepPlannerRequest request)
   {
      // test without A*
      request.setPerformAStarSearch(false);
      request.setPlanBodyPath(false);
      FootstepPlannerOutput plannerOutput = new FootstepPlanningModule(getClass().getSimpleName()).handleRequest(request);
      assertTrue(PlannerTools.isGoalNextToLastStep(goalPose3d, plannerOutput.getFootstepPlan()));

      // test with A* without body path
      request.setPerformAStarSearch(true);
      request.setPlanBodyPath(false);
      plannerOutput = new FootstepPlanningModule(getClass().getSimpleName()).handleRequest(request);
      assertTrue(PlannerTools.isGoalNextToLastStep(goalPose3d, plannerOutput.getFootstepPlan()));

      // test with A* and body path
      request.setPerformAStarSearch(true);
      request.setPlanBodyPath(true);
      plannerOutput = new FootstepPlanningModule(getClass().getSimpleName()).handleRequest(request);
      assertTrue(PlannerTools.isGoalNextToLastStep(goalPose3d, plannerOutput.getFootstepPlan()));
   }
}
