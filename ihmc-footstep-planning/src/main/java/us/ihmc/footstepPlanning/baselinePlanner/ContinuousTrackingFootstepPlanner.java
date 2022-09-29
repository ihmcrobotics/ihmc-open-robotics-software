package us.ihmc.footstepPlanning.baselinePlanner;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.math.trajectories.interfaces.PoseTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class ContinuousTrackingFootstepPlanner
{
   private final int numberOfTrajectoryPoints = 40;

   private final double previewTime;
   private double dt;
   private final PreviewWindowPoseTrajectoryGenerator trajectory;
   private int headFootstepPlanSize;
   private int tailFootstepPlanSize;
   private final List<SimpleTimedFootstep> headFootstepPlan;
   private final List<SimpleTimedFootstep> tailFootstepPlan;
   private final BaselineFootstepPlanner footstepPlanner;
   private final SideDependentList<FramePose3D> headFootholds;
   private final SideDependentList<FramePose3D> tailFootholds;
   private RobotSide nextFootstepSide;
   private final SimpleTimedFootstep ongoingFootstep;
   private final BaselineFootstepPlannerParameters parameters;

   private double swingTime = 0.6;
   private double transferTime = 0.25;
   private FootstepDataListMessage plannedFootsteps = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
   private ArrayList<SimpleTimedFootstep> allSteps = new ArrayList<>();

   public ContinuousTrackingFootstepPlanner(BaselineFootstepPlannerParameters parameters,
                                            double previewTime,
                                            double dt,
                                            int maxFootsteps,
                                            double maxVelocityX,
                                            double maxVelocityY,
                                            double maxVelocityYaw)
   {
      this.parameters = parameters;
      this.previewTime = previewTime;
      this.dt = dt;

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

      // Initialize baseline trajectory.
      this.trajectory = new PreviewWindowPoseTrajectoryGenerator(ReferenceFrame.getWorldFrame(),
                                                                 (int) Math.ceil(previewTime / dt),
                                                                 dt,
                                                                 maxVelocityX,
                                                                 maxVelocityY,
                                                                 maxVelocityYaw);
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
      trajectory.append(forwardVelocity, lateralVelocity, turningVelocity);
      planAndVisualize(currentTime, dt);
   }

   public void update(double currentTime, double deltaTime, FramePose3D framePose3D)
   {
      trajectory.append(deltaTime, framePose3D);
      planAndVisualize(currentTime, deltaTime);
   }

   public void update(double currentTime, double deltaTime, double forwardVelocity, double lateralVelocity, double turningVelocity)
   {
      // Update baseline trajectory.
      trajectory.append(deltaTime, forwardVelocity, lateralVelocity, turningVelocity);
      planAndVisualize(currentTime, deltaTime);
   }

   public void planAndVisualize(double currentTime, double deltaTime)
   {
      compute(0, previewTime);

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
      plannerStartTime = Math.max(plannerStartTime, previewTime - (parameters.getMinimumTransferDuration() + parameters.getSwingDuration())) - deltaTime;
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

         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(tailFootstep.getRobotSide(),
                                                                                           tailFootstep.getSoleFramePose().getPosition(),
                                                                                           tailFootstep.getSoleFramePose().getOrientation());
         plannedFootsteps.getFootstepDataList().add().set(footstepData);
         allSteps.add(tailFootstep);
      }
   }

   public FootstepDataListMessage getPlannedFootsteps()
   {
      return plannedFootsteps;
   }

   public void removePublishedFootSteps()
   {
      plannedFootsteps.getFootstepDataList().remove(0);
   }

   public void compute(double t0, double tf)
   {
      double dt = (tf - t0) / (numberOfTrajectoryPoints - 1);
      for (int sampleIdx = 0; sampleIdx < numberOfTrajectoryPoints; sampleIdx++)
      {
         trajectory.compute(t0 + sampleIdx * dt);
      }
   }

   public ArrayList<SimpleTimedFootstep> getAllSteps()
   {
      return allSteps;
   }
}
