package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import controller_msgs.LocalizationPacket;
import controller_msgs.PelvisPoseErrorPacket;
import ihmc_common_msgs.StampedPosePacket;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Runs several pelvis pose corrections one after another each estimator tick. Each correction starts from the pelvis pose the
 * previous one left, so a later correction refines the result of an earlier one.
 * <p>
 * The external pose subscriber is read once per tick, and each new external pose is offered to every correction that is
 * active on that tick. A correction that is not active is skipped entirely: it neither changes the pelvis pose nor receives
 * the poses that arrive while it is off.
 * </p>
 */
public class PelvisPoseCorrectionSeries implements PelvisPoseHistoryCorrectionInterface
{
   private final String name;
   private final List<Stage> stages = new ArrayList<>();
   private final StampedPosePacket latestExternalPose = new StampedPosePacket();
   private PelvisPoseCorrectionCommunicatorInterface externalPoseSubscriber;

   public PelvisPoseCorrectionSeries()
   {
      this(PelvisPoseCorrectionSeries.class.getSimpleName());
   }

   public PelvisPoseCorrectionSeries(String name)
   {
      this.name = name;
   }

   /** Adds a correction that always runs, after the ones already added. */
   public void addCorrection(PelvisPoseHistoryCorrectionInterface correction)
   {
      addCorrection(correction, () -> true);
   }

   /** Adds a correction, after the ones already added, that runs only on the ticks where {@code active} is true. */
   public void addCorrection(PelvisPoseHistoryCorrectionInterface correction, BooleanSupplier active)
   {
      Stage stage = new Stage(Objects.requireNonNull(correction), Objects.requireNonNull(active));
      stages.add(stage);
      if (externalPoseSubscriber != null)
         correction.setExternalPelvisCorrectorSubscriber(stage);
   }

   @Override
   public void doControl(long timestamp)
   {
      boolean hasNewExternalPose = false;
      if (externalPoseSubscriber != null && externalPoseSubscriber.hasNewPose())
      {
         StampedPosePacket externalPose = externalPoseSubscriber.getNewExternalPose();
         if (externalPose != null)
         {
            latestExternalPose.set(externalPose);
            hasNewExternalPose = true;
         }
      }

      for (int i = 0; i < stages.size(); i++)
      {
         Stage stage = stages.get(i);
         boolean active = stage.active.getAsBoolean();
         stage.hasNewPose = hasNewExternalPose && active;
         if (active)
            stage.correction.doControl(timestamp);
      }
   }

   /** Each correction receives its own view of this subscriber, so every one of them sees every new external pose. */
   @Override
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      externalPoseSubscriber = externalPelvisPoseSubscriber;
      for (int i = 0; i < stages.size(); i++)
         stages.get(i).correction.setExternalPelvisCorrectorSubscriber(stages.get(i));
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      for (int i = 0; i < stages.size(); i++)
      {
         YoGraphicDefinition graphics = stages.get(i).correction.getSCS2YoGraphics();
         if (graphics != null)
            group.addChild(graphics);
      }
      return group;
   }

   /** One correction in the series, and the view of the external pose subscriber that it is given. */
   private class Stage implements PelvisPoseCorrectionCommunicatorInterface
   {
      private final PelvisPoseHistoryCorrectionInterface correction;
      private final BooleanSupplier active;
      private boolean hasNewPose;

      private Stage(PelvisPoseHistoryCorrectionInterface correction, BooleanSupplier active)
      {
         this.correction = correction;
         this.active = active;
      }

      @Override
      public boolean hasNewPose()
      {
         return hasNewPose;
      }

      @Override
      public StampedPosePacket getNewExternalPose()
      {
         if (!hasNewPose)
            return null;
         hasNewPose = false;
         return latestExternalPose;
      }

      @Override
      public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
      {
         if (externalPoseSubscriber != null)
            externalPoseSubscriber.sendPelvisPoseErrorPacket(pelvisPoseErrorPacket);
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
         if (externalPoseSubscriber != null)
            externalPoseSubscriber.sendLocalizationResetRequest(localizationPacket);
      }

      @Override
      public void receivedPacket(StampedPosePacket packet)
      {
         if (externalPoseSubscriber != null)
            externalPoseSubscriber.receivedPacket(packet);
      }
   }
}
