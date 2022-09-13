package us.ihmc.footstepPlanning.baselinePlanner;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.interfaces.PoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class PreviewWindowFootstepPlanner
{
   private final int VIZ_SAMPLE_COUNT = 40;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final double previewTime;
   private final double dt;
   private final PreviewWindowPoseTrajectoryGenerator trajectory;
   private final SamplingPoseTrajectoryVisualizer trajectoryViz;
   private int headFootstepPlanSize;
   private int tailFootstepPlanSize;
   private final List<SimpleTimedFootstep> headFootstepPlan;
   private final List<SimpleTimedFootstep> tailFootstepPlan;
   private final TimedFootstepPlanVisualization footstepPlanViz;
   private final BaselineFootstepPlanner footstepPlanner;
   private final SideDependentList<FramePose3D> headFootholds;
   private final SideDependentList<FramePose3D> tailFootholds;
   private RobotSide nextFootstepSide;
   private final SimpleTimedFootstep ongoingFootstep;
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> vizHeadFootholds;
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> vizHeadFootholdsPreviewDuringSwing;
   private final YoBoolean visualizeFootstepPlan;
   private final YoBoolean visualizeCurrentFootholds;
   private final YoBoolean visualizeBaselineTrajectory;
   private final BaselineFootstepPlannerParameters parameters;

   public PreviewWindowFootstepPlanner(BaselineFootstepPlannerParameters parameters, double previewTime, double dt, int maxFootsteps,
                                       SideDependentList<ConvexPolygon2D> footPolygons, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.parameters = parameters;
      this.previewTime = previewTime;
      this.dt = dt;
      this.visualizeFootstepPlan = new YoBoolean("visualizeFootstepPlan", registry);
      this.visualizeFootstepPlan.set(true);
      this.visualizeCurrentFootholds = new YoBoolean("visualizeCurrentFootholds", registry);
      this.visualizeCurrentFootholds.set(true);
      this.visualizeBaselineTrajectory = new YoBoolean("visualizeBaselineTrajectory", registry);
      this.visualizeBaselineTrajectory.set(true);

      // Initialize footstep plans.
      this.headFootstepPlanSize = 0;
      this.tailFootstepPlanSize = 0;
      this.headFootstepPlan = new ArrayList<>();
      this.tailFootstepPlan = new ArrayList<>();
      for (int i = 0; i < maxFootsteps; i++)
      {
         this.headFootstepPlan.add(new SimpleTimedFootstep());
         this.tailFootstepPlan.add(new SimpleTimedFootstep());
      }

      // Initialize footholds.
      this.nextFootstepSide = RobotSide.LEFT;
      this.headFootholds = new SideDependentList<>();
      this.tailFootholds = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         this.headFootholds.put(robotSide, new FramePose3D());
         this.tailFootholds.put(robotSide, new FramePose3D());
      }
      this.ongoingFootstep = new SimpleTimedFootstep();

      // Initialize footstep planner.
      this.footstepPlanner = new BaselineFootstepPlanner(parameters);
      this.footstepPlanViz = new TimedFootstepPlanVisualization(headFootstepPlan, headFootstepPlanSize, footPolygons, parentRegistry,
                                                                graphicsListRegistry);
      this.footstepPlanViz.setPreviewTime(previewTime);

      // Initialize baseline trajectory.
      this.trajectory = new PreviewWindowPoseTrajectoryGenerator(ReferenceFrame.getWorldFrame(), (int) Math.ceil(previewTime / dt), dt);
      this.trajectoryViz = new SamplingPoseTrajectoryVisualizer("footstep", trajectory, ReferenceFrame.getWorldFrame(), VIZ_SAMPLE_COUNT, 0.014, false, registry,
                                                                graphicsListRegistry);

      // Create foothold graphics.
      this.vizHeadFootholds = new SideDependentList<>();
      this.vizHeadFootholdsPreviewDuringSwing = new SideDependentList<>();
      initializeFootholdGraphics(footPolygons, graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void initialize(SideDependentList<FramePose3D> initialFootholds)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.headFootholds.get(robotSide).setIncludingFrame(initialFootholds.get(robotSide));
         this.tailFootholds.get(robotSide).setIncludingFrame(initialFootholds.get(robotSide));
      }

      RobotSide lastFootstepSide = nextFootstepSide.getOppositeSide();
      this.ongoingFootstep.setSoleFramePose(initialFootholds.get(lastFootstepSide));
      this.ongoingFootstep.getTimeInterval().setEndTime(0.0);
      this.ongoingFootstep.setRobotSide(lastFootstepSide);

      this.headFootstepPlanSize = 0;
      this.tailFootstepPlanSize = 0;

      FramePose3D initialBaselinePose = new FramePose3D();
      footstepPlanner.computeBaselinePoseFromFootholdPose(initialBaselinePose, initialFootholds.get(lastFootstepSide), lastFootstepSide);
      this.trajectory.reset(initialBaselinePose);
   }

   public void update(double currentTime, double forwardVelocity, double lateralVelocity, double turningVelocity)
   {
      // Update baseline trajectory.
      trajectory.append(forwardVelocity, lateralVelocity, turningVelocity);
      if (visualizeBaselineTrajectory.getBooleanValue())
         trajectoryViz.redraw(0, previewTime);
      else
         trajectoryViz.hide();

      // Plan new footsteps.
      double plannerStartTime;
      if (headFootstepPlanSize > 0)
      {
         SimpleTimedFootstep tailFootstep = headFootstepPlan.get(headFootstepPlanSize - 1);
         tailFootstep.getSoleFramePose(tailFootholds.get(tailFootstep.getRobotSide()));
         nextFootstepSide = tailFootstep.getRobotSide().getOppositeSide();
         plannerStartTime = tailFootstep.getTimeInterval().getEndTime() - currentTime;
      }
      else
      {
         nextFootstepSide = ongoingFootstep.getRobotSide().getOppositeSide();
         plannerStartTime = ongoingFootstep.getTimeInterval().getEndTime() - currentTime;
      }
      plannerStartTime = Math.max(plannerStartTime, previewTime - (parameters.getMinimumTransferDuration() + parameters.getSwingDuration())) - dt;
      tailFootstepPlanSize = footstepPlanner.compute(tailFootstepPlan, trajectory, tailFootholds, nextFootstepSide, plannerStartTime, previewTime);

      // Remove footsteps that have already started.
      if (headFootstepPlanSize > 0)
      {
         SimpleTimedFootstep headFootstep = headFootstepPlan.get(0);
         if (headFootstep.getTimeInterval().getStartTime() < currentTime)
         {
            ongoingFootstep.set(headFootstep);
            removeHeadFootstep();
         }
      }

      // Add new footsteps.
      for (int i = 0; i < tailFootstepPlanSize; i++)
      {
         tailFootstepPlan.get(i).getTimeInterval().shiftInterval(currentTime);
         addTailFootstep(tailFootstepPlan.get(i));
      }

      // Update footstep visualization.
      if (visualizeFootstepPlan.getBooleanValue())
      {
         footstepPlanViz.setNumberOfValidSteps(headFootstepPlanSize);
         footstepPlanViz.update(currentTime);
      }

      // Update foothold visualization.
      RobotSide robotSide = ongoingFootstep.getRobotSide();
      ongoingFootstep.getSoleFramePose(headFootholds.get(robotSide));
      vizHeadFootholds.get(robotSide).setZ(-100.0); // hide polygon
      vizHeadFootholdsPreviewDuringSwing.get(robotSide).setZ(-100.0); // hide polygon
      double endTime = ongoingFootstep.getTimeInterval().getEndTime();
      if (visualizeCurrentFootholds.getBooleanValue() && endTime < currentTime)
         vizHeadFootholds.get(robotSide).setMatchingFrame(headFootholds.get(robotSide));
      if (visualizeFootstepPlan.getBooleanValue() && endTime > currentTime)
         vizHeadFootholdsPreviewDuringSwing.get(robotSide).setMatchingFrame(headFootholds.get(robotSide));
   }

   public PoseTrajectoryGenerator getPreviewTrajectory()
   {
      return trajectory;
   }

   public double getPreviewTime()
   {
      return previewTime;
   }

   public List<SimpleTimedFootstep> getFootstepPlan()
   {
      return headFootstepPlan;
   }

   public int getFootstepPlanSize()
   {
      return headFootstepPlanSize;
   }

   private void removeHeadFootstep()
   {
      if (headFootstepPlanSize > 0)
      {
         for (int i = 0; i < headFootstepPlanSize - 1; i++)
         {
            headFootstepPlan.get(i).set(headFootstepPlan.get(i + 1));
         }
         headFootstepPlanSize--;
      }
   }

   private void addTailFootstep(SimpleTimedFootstep tailFootstep)
   {
      if (headFootstepPlanSize < headFootstepPlan.size())
      {
         headFootstepPlan.get(headFootstepPlanSize).set(tailFootstep);
         headFootstepPlanSize++;
      }
   }

   private void initializeFootholdGraphics(SideDependentList<ConvexPolygon2D> footPolygons, YoGraphicsListRegistry graphicsListRegistry)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Create yo footstep polygons.
      SideDependentList<YoFrameConvexPolygon2D> yoFootPolygon = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         yoFootPolygon.put(robotSide, new YoFrameConvexPolygon2D(robotSide.getLowerCaseName() + "FootPolygon", "", worldFrame, 6, registry));
         yoFootPolygon.get(robotSide).set(footPolygons.get(robotSide));
      }

      // Create foothold graphics.
      for (RobotSide robotSide : RobotSide.values)
      {
         vizHeadFootholdsPreviewDuringSwing.put(robotSide, new YoFramePoseUsingYawPitchRoll(robotSide + "FootholdPoseEarly", worldFrame, registry));
         vizHeadFootholdsPreviewDuringSwing.get(robotSide).setZ(-100); // hide the graphic
         vizHeadFootholds.put(robotSide, new YoFramePoseUsingYawPitchRoll(robotSide + "FootholdPose", worldFrame, registry));
         vizHeadFootholds.get(robotSide).setMatchingFrame(headFootholds.get(robotSide));
         AppearanceDefinition appearancePreviewDuringSwing = (robotSide == RobotSide.LEFT) ? YoAppearance.Magenta() : YoAppearance.Gold();
         AppearanceDefinition appearance = (robotSide == RobotSide.LEFT) ? YoAppearance.DarkTurquoise() : YoAppearance.LawnGreen();
         graphicsListRegistry.registerYoGraphic("FootholdPreviewDuringSwing",
                                                new YoGraphicPolygon(robotSide + "FootholdPreviewDuringSwing", yoFootPolygon.get(robotSide), vizHeadFootholdsPreviewDuringSwing.get(robotSide),
                                                                     1.0, appearancePreviewDuringSwing));
         graphicsListRegistry.registerYoGraphic("Foothold",
                                                new YoGraphicPolygon(robotSide + "Foothold", yoFootPolygon.get(robotSide), vizHeadFootholds.get(robotSide), 1.0, appearance));
      }
   }
}
