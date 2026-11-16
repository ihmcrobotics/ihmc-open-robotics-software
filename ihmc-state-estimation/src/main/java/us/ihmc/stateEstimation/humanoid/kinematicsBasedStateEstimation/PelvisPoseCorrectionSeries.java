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

/**
 * Runs several pelvis pose corrections one after another each estimator tick. Each correction starts from the pelvis pose the
 * previous one left, so a later correction refines the result of an earlier one.
 * <p>
 * A correction runs only on the ticks where it {@link PelvisPoseHistoryCorrectionInterface#isActive() is active}. One that is
 * not active is skipped entirely: it neither changes the pelvis pose nor receives the poses that arrive while it is off.
 * {@link PelvisPoseCorrectionMode} sets which corrections are active.
 * </p>
 * <p>
 * The external pose subscriber is read once per tick. Each new external pose is first checked by the
 * {@link StampedPosePacketValidator}, and a valid one is offered to every correction that is active on that tick. After the
 * corrections have run, the resulting pelvis pose is recorded in the validator, so later external poses are compared with the
 * estimator's output.
 * </p>
 */
public class PelvisPoseCorrectionSeries implements PelvisPoseHistoryCorrectionInterface
{
   private final String name;
   private final StampedPosePacketValidator stampedPosePacketValidator;
   private final List<Stage> stages = new ArrayList<>();
   private final StampedPosePacket latestExternalPose = new StampedPosePacket();
   private PelvisPoseCorrectionCommunicatorInterface externalPoseSubscriber;

   /** @param stampedPosePacketValidator checks each new external pose before any correction sees it. */
   public PelvisPoseCorrectionSeries(StampedPosePacketValidator stampedPosePacketValidator)
   {
      this(PelvisPoseCorrectionSeries.class.getSimpleName(), stampedPosePacketValidator);
   }

   /** @param stampedPosePacketValidator checks each new external pose before any correction sees it. */
   public PelvisPoseCorrectionSeries(String name, StampedPosePacketValidator stampedPosePacketValidator)
   {
      this.name = name;
      this.stampedPosePacketValidator = Objects.requireNonNull(stampedPosePacketValidator);
   }

   /** Adds a correction after the ones already added. It runs on the ticks where it is active. */
   public void addCorrection(PelvisPoseHistoryCorrectionInterface correction)
   {
      Stage stage = new Stage(Objects.requireNonNull(correction));
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
         if (externalPose != null && stampedPosePacketValidator.isValid(externalPose))
         {
            latestExternalPose.set(externalPose);
            hasNewExternalPose = true;
         }
      }

      for (int i = 0; i < stages.size(); i++)
      {
         Stage stage = stages.get(i);
         boolean active = stage.correction.isActive();
         stage.hasNewPose = hasNewExternalPose && active;
         if (active)
            stage.correction.doControl(timestamp);
      }

      // Every correction has run, so the root joint holds this tick's output pelvis pose.
      stampedPosePacketValidator.recordEstimatedPelvisPose(timestamp);
   }

   /** Each correction receives its own view of this subscriber, so every one of them sees every new external pose. */
   @Override
   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      externalPoseSubscriber = externalPelvisPoseSubscriber;
      for (int i = 0; i < stages.size(); i++)
         stages.get(i).correction.setExternalPelvisCorrectorSubscriber(stages.get(i));
   }

   /** Whether any correction in the series is active. */
   @Override
   public boolean isActive()
   {
      for (int i = 0; i < stages.size(); i++)
      {
         if (stages.get(i).correction.isActive())
            return true;
      }
      return false;
   }

   /** Switches every correction in the series on or off. */
   @Override
   public void setActive(boolean active)
   {
      for (int i = 0; i < stages.size(); i++)
         stages.get(i).correction.setActive(active);
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
      private boolean hasNewPose;

      private Stage(PelvisPoseHistoryCorrectionInterface correction)
      {
         this.correction = correction;
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
