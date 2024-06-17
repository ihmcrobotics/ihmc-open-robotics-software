package us.ihmc.footstepPlanning.ui.components;

import javafx.animation.AnimationTimer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.messager.Messager;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class FootPoseFromMidFootUpdater extends AnimationTimer
{
   private final Messager messager;
   private final AtomicBoolean recomputeStart = new AtomicBoolean();
   private final AtomicBoolean recomputeGoal = new AtomicBoolean();

   private final AtomicReference<Point3D> startMidFootPosition;
   private final AtomicReference<Quaternion> startMidFootOrientation;
   private final AtomicReference<Point3D> goalMidFootPosition;
   private final AtomicReference<Quaternion> goalMidFootOrientation;

   private final AtomicReference<DefaultFootstepPlannerParametersReadOnly> parameters;

   private final Pose3D leftFootPose = new Pose3D();
   private final Pose3D rightFootPose = new Pose3D();

   public FootPoseFromMidFootUpdater(Messager messager)
   {
      this.messager = messager;

      messager.addTopicListener(FootstepPlannerMessagerAPI.StartMidFootPosition, position -> recomputeStart.set(true));
      messager.addTopicListener(FootstepPlannerMessagerAPI.StartMidFootOrientation, orientation -> recomputeStart.set(true));
      messager.addTopicListener(FootstepPlannerMessagerAPI.GoalMidFootPosition, position -> recomputeGoal.set(true));
      messager.addTopicListener(FootstepPlannerMessagerAPI.GoalMidFootOrientation, orientation -> recomputeGoal.set(true));

      startMidFootPosition = messager.createInput(FootstepPlannerMessagerAPI.StartMidFootPosition, new Point3D());
      startMidFootOrientation = messager.createInput(FootstepPlannerMessagerAPI.StartMidFootOrientation, new Quaternion());
      goalMidFootPosition = messager.createInput(FootstepPlannerMessagerAPI.GoalMidFootPosition, new Point3D());
      goalMidFootOrientation = messager.createInput(FootstepPlannerMessagerAPI.GoalMidFootOrientation, new Quaternion());

      parameters = messager.createInput(FootstepPlannerMessagerAPI.PlannerParameters, new DefaultFootstepPlannerParameters());
   }

   @Override
   public void handle(long now)
   {
      if (recomputeStart.getAndSet(false))
      {
         leftFootPose.set(startMidFootPosition.get(), startMidFootOrientation.get());
         rightFootPose.set(startMidFootPosition.get(), startMidFootOrientation.get());

         double idealFootstepWidth = parameters.get().getIdealFootstepWidth();
         leftFootPose.appendTranslation(0.0, 0.5 * idealFootstepWidth, 0.0);
         rightFootPose.appendTranslation(0.0, -0.5 * idealFootstepWidth, 0.0);

         messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, leftFootPose);
         messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, rightFootPose);
      }

      if (recomputeGoal.getAndSet(false))
      {
         leftFootPose.set(goalMidFootPosition.get(), goalMidFootOrientation.get());
         rightFootPose.set(goalMidFootPosition.get(), goalMidFootOrientation.get());

         double idealFootstepWidth = parameters.get().getIdealFootstepWidth();
         leftFootPose.appendTranslation(0.0, 0.5 * idealFootstepWidth, 0.0);
         rightFootPose.appendTranslation(0.0, -0.5 * idealFootstepWidth, 0.0);

         messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, leftFootPose);
         messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, rightFootPose);
      }
   }
}
