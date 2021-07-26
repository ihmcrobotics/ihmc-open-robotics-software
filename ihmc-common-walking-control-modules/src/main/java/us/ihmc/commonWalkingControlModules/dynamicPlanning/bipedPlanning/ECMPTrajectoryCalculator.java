package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateBasics;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerTools.sufficientlyLongTime;

public class ECMPTrajectoryCalculator<T extends ContactStateBasics<T>>
{
   private static final boolean debug = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RecyclingArrayList<T> contactStateProviders;
   private final double weight;

   private static final int maxPoints = 20;

   private final List<YoFrameVector2D> ecmpStartOffsets = new ArrayList<>();
   private final List<YoFrameVector2D> ecmpEndOffsets = new ArrayList<>();

   public ECMPTrajectoryCalculator(double mass, double gravity, Supplier<T> contactStateSupplier, YoRegistry parentRegistry)
   {
      gravity = Math.abs(gravity);

      contactStateProviders = new RecyclingArrayList<>(contactStateSupplier);
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      for (int i = 0; i < maxPoints; i++)
      {
         YoFrameVector2D ecmpStartOffset = new YoFrameVector2D("ecmpStartOffset" + i, worldFrame, registry);
         YoFrameVector2D ecmpEndOffset = new YoFrameVector2D("ecmpEndOffset" + i, worldFrame, registry);

         ecmpStartOffset.setToNaN();
         ecmpEndOffset.setToNaN();

         ecmpStartOffsets.add(ecmpStartOffset);
         ecmpEndOffsets.add(ecmpEndOffset);
      }

      parentRegistry.addChild(registry);

      weight = mass * gravity;
   }

   private final FramePoint3D ecmpPosition = new FramePoint3D();
   private final FrameVector3D ecmpVelocity = new FrameVector3D();

   private final FrameVector2D offset = new FrameVector2D();

   public List<T> computeECMPTrajectory(List<T> copTrajectories, MultipleSegmentPositionTrajectoryGenerator<?> desiredAngularMomentumTrajectories)
   {
      contactStateProviders.clear();
      int length = copTrajectories.size();

      for (int i = 0; i < length; i++)
      {
         contactStateProviders.add().set(copTrajectories.get(i));
      }

      int i = 0;
      for (; i < length - 1; i++)
      {
         T copTrajectory = copTrajectories.get(i);
         double startTime = Math.min(copTrajectory.getTimeInterval().getStartTime(), sufficientlyLongTime);
         double endTime = Math.min(copTrajectory.getTimeInterval().getEndTime(), sufficientlyLongTime);

         if (startTime > desiredAngularMomentumTrajectories.getEndTime() || endTime > desiredAngularMomentumTrajectories.getEndTime())
         {
            if (i < ecmpStartOffsets.size())
            {
               ecmpStartOffsets.get(i).setToNaN();
               ecmpEndOffsets.get(i).setToNaN();
            }

            break;
         }

         T eCMPTrajectory = contactStateProviders.get(i);

         if (!eCMPTrajectory.getContactState().isLoadBearing())
            continue;

         desiredAngularMomentumTrajectories.compute(startTime + 1e-8);

         computeECMPOffset(desiredAngularMomentumTrajectories.getVelocity(), offset);
         computeECMPVelocity(copTrajectory.getECMPStartVelocity(), desiredAngularMomentumTrajectories.getAcceleration(), ecmpVelocity);

         if (debug && offset.containsNaN())
            LogTools.info("Error.");
         if (debug && ecmpVelocity.containsNaN())
            LogTools.info("Error.");

         ecmpPosition.set(copTrajectory.getECMPStartPosition());
         ecmpPosition.add(offset.getX(), offset.getY(), 0.0);
         if (i < ecmpStartOffsets.size())
            ecmpStartOffsets.get(i).set(offset);

         eCMPTrajectory.setStartECMPPosition(ecmpPosition);
         eCMPTrajectory.setStartECMPVelocity(ecmpVelocity);

         desiredAngularMomentumTrajectories.compute(endTime - 1e-8);

         computeECMPOffset(desiredAngularMomentumTrajectories.getVelocity(), offset);
         computeECMPVelocity(copTrajectory.getECMPEndVelocity(), desiredAngularMomentumTrajectories.getAcceleration(), ecmpVelocity);

         if (debug && offset.containsNaN())
            LogTools.info("Error.");
         if (debug && ecmpVelocity.containsNaN())
            LogTools.info("Error.");

         ecmpPosition.set(copTrajectory.getECMPEndPosition());
         ecmpPosition.add(offset.getX(), offset.getY(), 0.0);
         if (i < ecmpEndOffsets.size())
            ecmpEndOffsets.get(i).set(offset);

         eCMPTrajectory.setEndECMPPosition(ecmpPosition);
         eCMPTrajectory.setEndECMPVelocity(ecmpVelocity);
      }

      for (; i < maxPoints; i++)
      {
         ecmpStartOffsets.get(i).setToNaN();
         ecmpEndOffsets.get(i).setToNaN();
      }

      return contactStateProviders;
   }

   public void computeCoPPosition(FramePoint3DReadOnly desiredECMPPosition, FrameVector3DReadOnly desiredAngularMomentumRate, FixedFramePoint3DBasics copPositionToPack)
   {
      copPositionToPack.setX(desiredAngularMomentumRate.getY());
      copPositionToPack.setY(-desiredAngularMomentumRate.getX());
      copPositionToPack.scale(-1.0 / weight);
      copPositionToPack.add(desiredECMPPosition);
   }

   public void computeECMPOffset(FrameVector3DReadOnly desiredAngularMomentumRate, FixedFrameVector2DBasics ecmpOffsetToPack)
   {
      ecmpOffsetToPack.setX(desiredAngularMomentumRate.getY());
      ecmpOffsetToPack.setY(-desiredAngularMomentumRate.getX());
      ecmpOffsetToPack.scale(1.0 / weight);
   }

   public void computeECMPVelocity(FrameVector3DReadOnly desiredCopVelocity, FrameVector3DReadOnly desiredAngularMomentumAcceleration, FixedFrameVector3DBasics ecmpVelocityToPack)
   {
      ecmpVelocityToPack.setX(desiredAngularMomentumAcceleration.getY());
      ecmpVelocityToPack.setY(-desiredAngularMomentumAcceleration.getX());
      ecmpVelocityToPack.scale(1.0 / weight);
      ecmpVelocityToPack.add(desiredCopVelocity);
   }

   public RecyclingArrayList<T> getContactStateProviders()
   {
      return contactStateProviders;
   }
}
