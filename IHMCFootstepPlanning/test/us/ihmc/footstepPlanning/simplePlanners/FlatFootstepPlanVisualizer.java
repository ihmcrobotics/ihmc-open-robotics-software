package us.ihmc.footstepPlanning.simplePlanners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.PlanningUtils;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPolygon;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class FlatFootstepPlanVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   public FlatFootstepPlanVisualizer(FramePose2d goalPose, FramePose2d initialStancePose, RobotSide initialRobotSide)
   {
      FootstepPlanner planner = new TurnWalkTurnPlanner();
      // create inputs
      planner.setInitialStanceFoot(PlanningUtils.poseFormPose2d(initialStancePose), initialRobotSide);
      planner.setGoalPose(PlanningUtils.poseFormPose2d(goalPose));

      // visualize output
      List<FramePose> footstepPlan3d = new ArrayList<>();
      planner.plan(footstepPlan3d);
      List<FramePose2d> footstepPlan = PlanningUtils.pose2dListFromPoseList(footstepPlan3d);
      RobotSide previousFootstepSide = initialRobotSide;
      YoFrameConvexPolygon2d yoDefaultFootPolygon = new YoFrameConvexPolygon2d("DefaultFootPolygon", worldFrame, 4, registry);

      ConvexPolygon2d defaultFootPolygon = new ConvexPolygon2d();
      defaultFootPolygon.addVertex(0.1, 0.05);
      defaultFootPolygon.addVertex(0.1, -0.05);
      defaultFootPolygon.addVertex(-0.1, 0.05);
      defaultFootPolygon.addVertex(-0.1, -0.05);
      defaultFootPolygon.update();
      yoDefaultFootPolygon.setConvexPolygon2d(defaultFootPolygon);

      int i = 0;
      for (FramePose2d footstepPose : footstepPlan)
      {
         AppearanceDefinition appearance = previousFootstepSide == RobotSide.LEFT ? YoAppearance.Green() : YoAppearance.Red();
         YoFramePose footPose = new YoFramePose("footPose" + (i++), worldFrame, registry);
         footPose.setX(footstepPose.getX());
         footPose.setY(footstepPose.getY());
         footPose.setYaw(footstepPose.getYaw());

         YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + (i++), yoDefaultFootPolygon, footPose, 1.0, appearance);
         graphicsListRegistry.registerYoGraphic("viz", footstepViz);
         previousFootstepSide = previousFootstepSide.getOppositeSide();
      }

      // make goal viz
      YoFramePoint yoGoal = new YoFramePoint("GoalPosition", worldFrame, registry);
      yoGoal.set(goalPose.getX(), goalPose.getY(), 0.0);
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("GoalViz", yoGoal, 0.05, YoAppearance.White()));
      Pose2dReferenceFrame goalFrame = new Pose2dReferenceFrame("GoalFrame", goalPose);
      FrameVector goalOrientation = new FrameVector(goalFrame, 0.5, 0.0, 0.0);
      goalOrientation.changeFrame(worldFrame);
      YoFrameVector yoGoalOrientation = new YoFrameVector("GoalVector", worldFrame, registry);
      yoGoalOrientation.set(goalOrientation);
      graphicsListRegistry.registerYoGraphic("vizOrientation", new YoGraphicVector("GoalOrientationViz", yoGoal, yoGoalOrientation, 1.0, YoAppearance.White()));

      // spawn scs
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"));
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry, true);
      scs.run();
   }

//   public static void main(String[] args)
//   {
//      FramePose2d goalPose = new FramePose2d(worldFrame, new Point2d(1.0, 1.0), Math.PI / 2.0);
//      FramePose2d initialStancePose = new FramePose2d(worldFrame);
//      RobotSide initialRobotSide = RobotSide.RIGHT;
//
//
//      PrintTools.info("Visualizing Footstep Plan on Flat Ground");
//      new FlatFootstepPlanVisualizer(goalPose, initialStancePose, initialRobotSide);
//   }
}
