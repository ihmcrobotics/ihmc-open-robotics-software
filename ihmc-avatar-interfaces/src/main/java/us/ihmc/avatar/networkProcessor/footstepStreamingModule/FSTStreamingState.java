package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxInputCommand;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This the main class of the Footstep streaming. This is a good starting to understand the mechanics of it.
 */
public class FSTStreamingState implements State
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FSTTools tools;
   private final SideDependentList<ReferenceFrame> ankleTrackerFrames = new SideDependentList<>();
   private final SideDependentList<Boolean> isUserStepping = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> initialTrackersTransform = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> previousTrackersTransform = new SideDependentList<>();
   private final SideDependentList<Integer> stableIterationCounts = new SideDependentList<>();
   private final SideDependentList<FrameVector2D> directionsCtrl = new SideDependentList<>(); // Controlled direction vector for each foot.
   private final SideDependentList<RigidBodyTransform> initialFootstepTransformsInWorld = new SideDependentList<>();

   private final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
   private final YoDouble timeSinceLastInput = new YoDouble("timeSinceLastInput", registry);
   private final YoDouble rawInputFrequency = new YoDouble("rawInputFrequency", registry);
   private final AlphaFilteredYoVariable inputFrequency;
   private final double toolboxControllerPeriod;
   private final YoDouble publishingPeriod = new YoDouble("publishingPeriod", registry);

   private final YoDouble defaultStepThreshold = new YoDouble("defaultStepThreshold", registry);
   private final YoDouble defaultLiftThreshold = new YoDouble("defaultLiftThreshold", registry);
   private final YoDouble defaultStrideLength = new YoDouble("defaultStrideLength", registry);
   private final YoDouble defaultKpDirection = new YoDouble("defaultKpDirection", registry);
   private final YoDouble defaultTurningThreshold = new YoDouble("defaultTurningThreshold", registry);
   private final YoDouble defaultTurnDegrees = new YoDouble("defaultTurnDegrees", registry);
   private final YoDouble defaultStabilityThreshold = new YoDouble("defaultStabilityThreshold", registry);
   private final YoInteger defaultStabilityIterations = new YoInteger("defaultStabilityIterations", registry);

   public FSTStreamingState(FSTTools tools)
   {
      FootstepStreamingToolboxParameters parameters = tools.getParameters();
      this.tools = tools;
      toolboxControllerPeriod = tools.getToolboxControllerPeriod();

      tools.getRegistry().addChild(registry);

      publishingPeriod.set(parameters.getPublishingPeriod());

      YoDouble inputFrequencyAlpha = new YoDouble("inputFrequencyFilter", registry);
      inputFrequencyAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(2.0, toolboxControllerPeriod));
      inputFrequency = new AlphaFilteredYoVariable("inputFrequency", registry, inputFrequencyAlpha, rawInputFrequency);

      defaultStepThreshold.set(parameters.getStepThreshold());
      defaultLiftThreshold.set(parameters.getLiftThreshold());
      defaultStrideLength.set(parameters.getStrideLength());
      defaultKpDirection.set(parameters.getKpDirection());
      defaultTurningThreshold.set(parameters.getTurningThreshold());
      defaultTurnDegrees.set(parameters.getTurnDegrees());
      defaultStabilityThreshold.set(parameters.getStabilityThreshold());
      defaultStabilityIterations.set(parameters.getStabilityIterations());
   }

   @Override
   public void onEntry()
   {
      for (RobotSide side : RobotSide.values())
      {
         isUserStepping.put(side, false);
         stableIterationCounts.put(side, 0);
         previousTrackersTransform.put(side, new RigidBodyTransform());
         // Initialize controlled direction to (1,0) (forward)
         directionsCtrl.put(side, new FrameVector2D(ReferenceFrame.getWorldFrame(), 1.0, 0.0));

         ankleTrackerFrames.put(side, null);
         initialTrackersTransform.put(side, null);
      }

      timeOfLastInput.set(Double.NaN);
      timeSinceLastInput.set(Double.NaN);
      inputFrequency.reset();
   }

   /**
    * This method is called continuously. It updates footstep targets based on ankle tracker movement.
    * If a user starts stepping, we enter stepping mode and use PD control to continuously update the footstep.
    * Once the user’s foot begins descending, we stabilize the stride length and finalize the step once stable.
    */
   @Override
   public void doAction(double timeInState)
   {
      FootstepStreamingToolboxInputCommand latestInput = tools.getLatestInput();

      if (latestInput != null)
      {
         RobotSide side = latestInput.getSide();
         RigidBodyTransform currentTrackerTransform =  new RigidBodyTransform(latestInput.getCurrentPose());
         RigidBodyTransform initialTrackerTransform = initialTrackersTransform.get(side);

         if (!isUserStepping.get(side)) // Tracker is not moving yet by a lot
         {
            if (initialTrackerTransform == null)
            {
               initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
               previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
            }

            Vector3D translation = new Vector3D();
            translation.sub(currentTrackerTransform.getTranslation(), initialTrackerTransform.getTranslation());
            FrameVector2D translationXY = new FrameVector2D(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY());
            // Check if the tracker has moved in any direction AND the foot has been lifted
            if (translationXY.norm() >= defaultStepThreshold.getDoubleValue() && translation.getZ() >= defaultLiftThreshold.getDoubleValue())
            {
               // Get the current robot foot position in world
               RigidBodyTransform footstepTransformInWorld = new RigidBodyTransform(latestInput.getRobotFootPose());

               FramePoint2D initialXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                         footstepTransformInWorld.getTranslation().getX(),
                                                         footstepTransformInWorld.getTranslation().getY());
               initialFootstepTransformsInWorld.put(side, footstepTransformInWorld);
               // Normalize the translation direction to have a fixed stride distance
               translationXY.normalize();
               // Scale the normalized direction to the fixed stride distance
               translationXY.scale(defaultStrideLength.getDoubleValue());
               // Apply the translation to the current robot foot position
               FramePoint2D predictedXY = new FramePoint2D(initialXY);
               predictedXY.add(translationXY);
               footstepTransformInWorld.getTranslation().setX(predictedXY.getX());
               footstepTransformInWorld.getTranslation().setY(predictedXY.getY());
               // Compute yaw variation
               double newYaw = footstepTransformInWorld.getRotation().getYaw();
               double yawVariation = currentTrackerTransform.getRotation().getYaw() - initialTrackerTransform.getRotation().getYaw();
               if (Math.toDegrees(yawVariation) >= defaultTurningThreshold.getDoubleValue())
               {
                  newYaw += Math.toRadians(defaultTurnDegrees.getDoubleValue());
               }
               else if (Math.toDegrees(yawVariation) <= -defaultTurningThreshold.getDoubleValue())
               {
                  newYaw -= Math.toRadians(defaultTurnDegrees.getDoubleValue());
               }
               // Update yaw of footstep
               footstepTransformInWorld.getRotation()
                                       .setYawPitchRoll(newYaw,
                                                        footstepTransformInWorld.getRotation().getPitch(),
                                                        footstepTransformInWorld.getRotation().getRoll());

               // Publish footstep to UI
               FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
               outputStatus.setRobotSide(side.toByte());
               outputStatus.getDesiredFootOrientation().set(footstepTransformInWorld.getRotation());
               outputStatus.getDesiredFootPosition().set(footstepTransformInWorld.getTranslation());
               tools.getStatusOutputManager().reportStatusMessage(outputStatus);
               isUserStepping.put(side, true);
            }
         }
         else // Already stepping, tracker is moving
         {
            if (initialTrackerTransform != null)
            {
               // Check if tracker is not moving much anymore
               Vector3D translationFromInitialPosition = new Vector3D();
               translationFromInitialPosition.sub(currentTrackerTransform.getTranslation(), previousTrackersTransform.get(side).getTranslation());

               // If the tacker is not moving much, then start counting for stability
               if (translationFromInitialPosition.norm() <= defaultStabilityThreshold.getDoubleValue())
               {
                  int stableCount = stableIterationCounts.get(side);
                  stableCount++;
                  stableIterationCounts.put(side, stableCount);

                  if (stableCount >= defaultStabilityIterations.getIntegerValue())
                  {
                     isUserStepping.put(side, false);
                     initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                     stableIterationCounts.put(side, 0);
                  }
               }
               else  // Still moving
               {
                  stableIterationCounts.put(side, 0); // reset stability count
//                  // Only update direction if acceleration is not too small
//                  if (accelerationMag > ACCELERATION_THRESHOLD)
//                  {
//                     // Compute current direction from ankle movement
//                     Vector3D translation = new Vector3D();
//                     translation.sub(currentTrackerTransform.getTranslation(), initialTrackerTransform.getTranslation());
//                     FrameVector2D directionDesired = new FrameVector2D(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY());
//                     directionDesired.normalize();
//                  }
//                  else
//                  {
//                     // If acceleration is very low, do not update direction any further.
//                     // Just keep the desired direction as is.
//                  }
//
//                  // P-control: error = desired_direction - current_controlled_direction
//                  FrameVector2D directionCtrl = directionsCtrl.get(side);
//                  FrameVector2D directionError = new FrameVector2D(directionDesired);
//                  directionError.sub(directionCtrl);
//
//                  // Update the controlled direction based on error
//                  directionCtrl.addX(KP_DIRECTION * directionError.getX());
//                  directionCtrl.addY(KP_DIRECTION * directionError.getY());
//
//                  // Re-normalize direction
//                  directionCtrl.normalize();
//                  directionsCtrl.put(side, new FrameVector2D(directionCtrl));
//
//                  // Predict new footstep position
//                  RigidBodyTransform footstepTransformInWorld = new RigidBodyTransform(initialFootstepTransformsInWorld.get(side));
//                  FramePoint2D initialXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
//                                                                     footstepTransformInWorld.getTranslation().getX(),
//                                                                     footstepTransformInWorld.getTranslation().getY());
//
//                  FramePoint2D predictedXY = new FramePoint2D(initialXY);
//                  predictedXY.add(directionCtrl.getX() * STRIDE_LENGTH, directionCtrl.getY() * STRIDE_LENGTH);
//
//                  footstepTransformInWorld.getTranslation().setX(predictedXY.getX());
//                  footstepTransformInWorld.getTranslation().setY(predictedXY.getY());
//
//                  FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
//                  outputStatus.setRobotSide(side.toByte());
//                  outputStatus.getDesiredFootOrientation().set(footstepTransformInWorld.getRotation());
//                  outputStatus.getDesiredFootPosition().set(footstepTransformInWorld.getTranslation());
//                  tools.getStatusOutputManager().reportStatusMessage(outputStatus);
               }

               // Update the previous tracker position for the next iteration
               previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
            }
         }
      }

      // Updating some statistics
      if (tools.hasNewInputCommand())
      {
         if (Double.isFinite(timeSinceLastInput.getValue()) && timeSinceLastInput.getValue() > 0.0)
         {
            rawInputFrequency.set(1.0 / timeSinceLastInput.getValue());
            inputFrequency.update();
         }

         timeOfLastInput.set(timeInState);
      }

      if (Double.isFinite(timeOfLastInput.getValue()))
      {
         timeSinceLastInput.set(timeInState - timeOfLastInput.getValue());
      }
   }

   @Override
   public void onExit(double timeInState)
   {
      tools.flushInputCommands();
   }
}
