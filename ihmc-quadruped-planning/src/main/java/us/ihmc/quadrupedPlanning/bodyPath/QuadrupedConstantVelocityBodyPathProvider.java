package us.ihmc.quadrupedPlanning.bodyPath;

import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedConstantVelocityBodyPathProvider implements QuadrupedPlanarBodyPathProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final YoDouble timestamp;
   private final ReferenceFrame supportFrame;
   private final YoBoolean footstepPlanHasBeenComputed = new YoBoolean("footstepPlanHasBeenComputed", registry);
   private final YoBoolean shiftPlanBasedOnStepAdjustment = new YoBoolean("shiftPlanBasedOnStepAdjustment", registry);

   private final YoFramePoint2D startPoint = new YoFramePoint2D("startPoint", worldFrame, registry);
   private final YoDouble startYaw = new YoDouble("startYaw", registry);
   private final YoDouble startTime = new YoDouble("startTime", registry);
   private final DoubleProvider firstStepDelay;

   private final YoFramePoint3D achievedStepAdjustment = new YoFramePoint3D("achievedStepAdjustment", worldFrame, registry);
   private final YoEnum<RobotQuadrant> mostRecentTouchdown = new YoEnum<>("mostRecentTouchdown", registry, RobotQuadrant.class);
   private final YoFrameVector3D desiredPlanarVelocity = new YoFrameVector3D("desiredPlanarVelocity", worldFrame, registry);
   private final FramePose2D initialPose = new FramePose2D();

   private final QuadrantDependentList<AtomicReference<QuadrupedFootstepStatusMessage>> footstepStartStatuses = new QuadrantDependentList<>();
   private final QuadrantDependentList<AtomicReference<QuadrupedFootstepStatusMessage>> footstepCompleteStatuses = new QuadrantDependentList<>();
   private final AtomicBoolean recomputeInitialPose = new AtomicBoolean();
   private final AtomicBoolean recomputeStepAdjustment = new AtomicBoolean();

   private final FramePose2D tempPose = new FramePose2D();
   private final QuaternionBasedTransform tempTransform = new QuaternionBasedTransform();

   public QuadrupedConstantVelocityBodyPathProvider(QuadrupedReferenceFrames referenceFrames, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                    DoubleProvider firstStepDelay, YoDouble timestamp, YoVariableRegistry parentRegistry)
   {
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.firstStepDelay = firstStepDelay;

      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         footstepStartStatuses.set(quadrant, new AtomicReference<>());
         footstepCompleteStatuses.set(quadrant, new AtomicReference<>());
      }

      this.xGaitSettings = xGaitSettings;
      this.timestamp = timestamp;

      parentRegistry.addChild(registry);
   }

   public void completedFootstep(RobotQuadrant robotQuadrant, QuadrupedFootstepStatusMessage message)
   {
      footstepCompleteStatuses.get(robotQuadrant).set(message);
      recomputeInitialPose.set(true);
   }

   public void startedFootstep(RobotQuadrant robotQuadrant, QuadrupedFootstepStatusMessage message)
   {
      footstepStartStatuses.get(robotQuadrant).set(message);
      recomputeInitialPose.set(true);
   }

   @Override
   public void initialize()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         footstepStartStatuses.get(quadrant).set(null);
         footstepCompleteStatuses.get(quadrant).set(null);
      }

      recomputeInitialPose.set(false);
      footstepPlanHasBeenComputed.set(false);
      recomputeStepAdjustment.set(false);
   }

   public void setPlanarVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityYaw)
   {
      desiredPlanarVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityYaw);
   }

   @Override
   public void getPlanarPose(double time, FramePose2D poseToPack)
   {
      if (recomputeStepAdjustment.getAndSet(false))
      {
         computeStepAdjustmentFromFootstepStatus();
      }
      if (recomputeInitialPose.getAndSet(false))
      {
         footstepPlanHasBeenComputed.set(true);
         setStartConditionsFromFootstepStatus();
      }
      else if (!footstepPlanHasBeenComputed.getBooleanValue())
      {
         footstepPlanHasBeenComputed.set(true);
         setStartConditionsFromCurrent();
      }

      initialPose.set(startPoint.getX(), startPoint.getY(), startYaw.getDoubleValue());
      if (shiftPlanBasedOnStepAdjustment.getBooleanValue())
      {
         initialPose.prependTranslation(achievedStepAdjustment.getX(), achievedStepAdjustment.getY());
      }

      extrapolatePose(time - startTime.getDoubleValue(), poseToPack, initialPose, desiredPlanarVelocity);
   }

   private static void extrapolatePose(double time, FramePose2D poseToPack, FramePose2D initialPose, Tuple3DReadOnly planarVelocity)
   {
      double a0 = initialPose.getYaw();
      double x0 = initialPose.getX();
      double y0 = initialPose.getY();

      // initialize forward, lateral, and rotational velocity in pose frame
      double u = planarVelocity.getX();
      double v = planarVelocity.getY();
      double phi = planarVelocity.getZ();

      // compute extrapolated pose assuming a constant planar velocity
      double a, x, y;
      double epsilon = 0.001;
      if (Math.abs(phi) > epsilon)
      {
         a = a0 + phi * time;
         x = x0 + u / phi * (Math.sin(a) - Math.sin(a0)) + v / phi * (Math.cos(a) - Math.cos(a0));
         y = y0 - u / phi * (Math.cos(a) - Math.cos(a0)) + v / phi * (Math.sin(a) - Math.sin(a0));
      }
      else
      {
         a = a0;
         x = x0 + (u * Math.cos(a) - v * Math.sin(a)) * time;
         y = y0 + (u * Math.sin(a) + v * Math.cos(a)) * time;
      }

      poseToPack.setX(x);
      poseToPack.setY(y);
      poseToPack.setYaw(a);
   }

   private void setStartConditionsFromCurrent()
   {
      RigidBodyTransform supportTransform = supportFrame.getTransformToWorldFrame();
      startTime.set(timestamp.getDoubleValue() + firstStepDelay.getValue() + xGaitSettings.getStepDuration());
      startYaw.set(supportTransform.getRotationMatrix().getYaw());
      startPoint.set(supportTransform.getTranslationVector());
   }

   private void setStartConditionsFromFootstepStatus()
   {
      QuadrupedFootstepStatusMessage latestStatusMessage = getLatestStartStatusMessage();
      if (latestStatusMessage == null)
         return;

      double previousStartTime = startTime.getDoubleValue();
      double newStartTime = latestStatusMessage.getDesiredStepInterval().getEndTime();
      startTime.set(newStartTime);

      if (shiftPlanBasedOnStepAdjustment.getBooleanValue())
      {
         RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) latestStatusMessage.getFootstepQuadrant());
         Point3DReadOnly latestMessageSoleDesiredPosition = latestStatusMessage.getDesiredTouchdownPositionInWorld();
         double halfStanceLength = quadrant.getEnd().negateIfFrontEnd(0.5 * xGaitSettings.getStanceLength());
         double halfStanceWidth = quadrant.getSide().negateIfLeftSide(0.5 * xGaitSettings.getStanceWidth());

         tempPose.set(halfStanceLength, halfStanceWidth, 0.0);
         tempTransform.setRotationYaw(startYaw.getDoubleValue());
         tempPose.applyTransform(tempTransform);
         tempPose.prependTranslation(latestMessageSoleDesiredPosition.getX(), latestMessageSoleDesiredPosition.getY());
         startPoint.set(tempPose.getPosition());
         startYaw.add(desiredPlanarVelocity.getZ() * (newStartTime - previousStartTime));
      }
      else
      {
         tempPose.set(startPoint.getX(), startPoint.getY(), startYaw.getDoubleValue());
         extrapolatePose(newStartTime - previousStartTime, tempPose, tempPose, desiredPlanarVelocity);
         startPoint.set(tempPose.getPosition());
         startYaw.add(desiredPlanarVelocity.getZ() * (newStartTime - previousStartTime));
      }
   }

   private void computeStepAdjustmentFromFootstepStatus()
   {
      QuadrupedFootstepStatusMessage latestCompleteStatusMessage = getLatestCompleteStatusMessage();
      RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) latestCompleteStatusMessage.getFootstepQuadrant());
      QuadrupedFootstepStatusMessage startStatusMessage = footstepStartStatuses.get(quadrant).get();

      if (startStatusMessage == null || latestCompleteStatusMessage == null)
         return;

      Point3DReadOnly soleDesiredAtStartOfStep = startStatusMessage.getDesiredTouchdownPositionInWorld();
      Point3DReadOnly soleDesiredAtTouchdown = latestCompleteStatusMessage.getDesiredTouchdownPositionInWorld();

      mostRecentTouchdown.set(quadrant);
      achievedStepAdjustment.sub(soleDesiredAtTouchdown, soleDesiredAtStartOfStep);
   }

   private QuadrupedFootstepStatusMessage getLatestStartStatusMessage()
   {
      double latestEndTime = Double.NEGATIVE_INFINITY;
      QuadrupedFootstepStatusMessage latestMessage = null;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         QuadrupedFootstepStatusMessage message = footstepStartStatuses.get(quadrant).get();
         if (message != null && message.getDesiredStepInterval().getEndTime() > latestEndTime)
         {
            latestEndTime = message.getDesiredStepInterval().getEndTime();
            latestMessage = message;
         }
      }
      return latestMessage;
   }

   private QuadrupedFootstepStatusMessage getLatestCompleteStatusMessage()
   {
      double latestEndTime = Double.NEGATIVE_INFINITY;
      QuadrupedFootstepStatusMessage latestMessage = null;
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         QuadrupedFootstepStatusMessage message = footstepCompleteStatuses.get(quadrant).get();
         if (message != null && message.getDesiredStepInterval().getEndTime() > latestEndTime)
         {
            latestEndTime = message.getDesiredStepInterval().getEndTime();
            latestMessage = message;
         }
      }
      return latestMessage;
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shiftPlanBasedOnStepAdjustment)
   {
      this.shiftPlanBasedOnStepAdjustment.set(shiftPlanBasedOnStepAdjustment);
   }
}
