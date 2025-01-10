package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import toolbox_msgs.msg.dds.FootstepStreamingToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxTrackerCommand;
import us.ihmc.log.LogTools;
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
   private final SideDependentList<FrameVector2D> directionTrackersCtrl = new SideDependentList<>(); // Controlled direction vector for each foot.
   private boolean lastDirectionAdjusment = false;
   private final SideDependentList<RigidBodyTransform> initialRobotSwingFootTransformsInWorld = new SideDependentList<>();
   private final SideDependentList<Double> previousTrackerTimestamp = new SideDependentList<>();

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
   private final YoDouble defaultAccelerationThreshold = new YoDouble("defaultAccelerationThreshold", registry);
   private final YoDouble defaultHorizontalAccelerationWeight = new YoDouble("defaultHorizontalAccelerationWeight", registry);
   private final YoDouble defaultVerticalComponentWeight = new YoDouble("defaultVerticalComponentWeight", registry);
   private final FrameVector3D accumulatedLinearVelocity = new FrameVector3D();
   private final FrameVector3D accumulatedAngularVelocity = new FrameVector3D();

   private final FrameVector3D displacementEstimatedFromLinearVelocity = new FrameVector3D();
   private final FrameVector3D angleEstimatedFromAngularVelocity = new FrameVector3D();

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
      defaultAccelerationThreshold.set(parameters.getAccelerationThreshold());
      defaultHorizontalAccelerationWeight.set(parameters.getHorizontalAccelerationWeight());
      defaultVerticalComponentWeight.set(parameters.getVerticalComponentWeight());
   }

   @Override
   public void onEntry()
   {
      for (RobotSide side : RobotSide.values())
      {
         isUserStepping.put(side, false);
         stableIterationCounts.put(side, 0);
         previousTrackersTransform.put(side, new RigidBodyTransform());
         // Initialize controlled direction
         directionTrackersCtrl.put(side, new FrameVector2D());

         ankleTrackerFrames.put(side, null);
         initialTrackersTransform.put(side, null);
         previousTrackerTimestamp.put(side, Double.NaN);
         initialRobotSwingFootTransformsInWorld.put(side, null);
      }

      lastDirectionAdjusment = false;
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
      if (tools.hasNewInputCommand())
      {
         FootstepStreamingToolboxInputCommand latestInput = tools.getLatestInput();

         if (latestInput.isCommandValid())
         {
            for (RobotSide side : RobotSide.values)
            {
               FootstepStreamingToolboxTrackerCommand sideCommand = latestInput.getInputFor(side);
               if (sideCommand != null)
               {
                  RigidBodyTransform currentTrackerTransform = new RigidBodyTransform(sideCommand.getCurrentPose());
                  RigidBodyTransform initialTrackerTransform = initialTrackersTransform.get(side);
                  RigidBodyTransform currentRobotFootTransformInWorld = new RigidBodyTransform(sideCommand.getRobotFootPose());

                  if (!isUserStepping.get(side)) // Tracker is not moving yet by a lot
                  {
                     if (initialTrackerTransform == null)
                     {
                        initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                        previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                        initialRobotSwingFootTransformsInWorld.put(side, new RigidBodyTransform(currentRobotFootTransformInWorld));
                     }

                     Vector3D translationTracker = new Vector3D();
                     translationTracker.sub(currentTrackerTransform.getTranslation(), initialTrackersTransform.get(side).getTranslation());
                     FrameVector2D translationTrackerXY = new FrameVector2D(ReferenceFrame.getWorldFrame(),
                                                                            translationTracker.getX(),
                                                                            translationTracker.getY());
                     // Check if the tracker has moved in any direction AND the foot has been lifted
                     if (translationTrackerXY.norm() >= defaultStepThreshold.getDoubleValue()
                         && translationTracker.getZ() >= defaultLiftThreshold.getDoubleValue())
                     {
                        // Normalize the translation direction to have a fixed stride distance
                        translationTrackerXY.normalize();
                        // Scale the normalized direction to the fixed stride distance
                        translationTrackerXY.scale(defaultStrideLength.getDoubleValue());
                        RigidBodyTransformReadOnly robotFootstepTransformInWorld = computeTargetFootstepFromStance(latestInput,
                                                                                                                   side,
                                                                                                                   translationTrackerXY,
                                                                                                                   initialTrackersTransform.get(side));
                        // Publish footstep to UI
                        FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
                        outputStatus.setAdjustmentFootstep(false); // first estimate
                        outputStatus.setLastAdjustment(false);
                        outputStatus.setRobotSide(side.toByte());
                        outputStatus.getDesiredFootOrientation().set(robotFootstepTransformInWorld.getRotation());
                        outputStatus.getDesiredFootPosition().set(robotFootstepTransformInWorld.getTranslation());
                        tools.getStatusOutputManager().reportStatusMessage(outputStatus);
                        isUserStepping.put(side, true);
                     }
                     else
                     {
                        // set now the initial position for robot stance foot, which will be the next swing
                        initialRobotSwingFootTransformsInWorld.put(side, new RigidBodyTransform(currentRobotFootTransformInWorld));
                        directionTrackersCtrl.put(side, new FrameVector2D());
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
                              directionTrackersCtrl.put(side, new FrameVector2D());
                              initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                              stableIterationCounts.put(side, 0);
                              lastDirectionAdjusment = false;
                           }
                        }
                        else  // Still moving
                        {
                           stableIterationCounts.put(side, 0); // reset stability count
                           // Only update direction if acceleration is not too small
                           //                        if (!tools.hasPreviousInput())
                           //                        {
                           //                           LogTools.error("Cannot update footstep estimate because previous input is missing");
                           //                        }
                           //                        else
                           //                        {
                           //                           // Get tracker acceleration
                           //                           var previousCommand = tools.getPreviousInput().getInputFor(side);
                           //                           previousTrackerTimestamp.put(side, 1.0 * previousCommand.getTimestamp());
                           //                           double dt = sideCommand.getTimestamp() - previousTrackerTimestamp.get(side);
                           //                           FixedFrameVector3DBasics linearAcceleration = new FrameVector3D();
                           //                           FixedFrameVector3DBasics angularAcceleration = new FrameVector3D();
                           //                           if (Double.isFinite(dt))
                           //                           {
                           //                              tools.computeAcceleration(dt,
                           //                                                        previousCommand.getCurrentVelocity().getLinearPart(),
                           //                                                        sideCommand.getCurrentVelocity().getLinearPart(),
                           //                                                        linearAcceleration);
                           //                              tools.computeAcceleration(dt,
                           //                                                        previousCommand.getCurrentVelocity().getAngularPart(),
                           //                                                        sideCommand.getCurrentVelocity().getAngularPart(),
                           //                                                        angularAcceleration);
                           //                           }
                           //                           // Extract horizontal (XY) component from the 3D linear acceleration
                           //                           FrameVector2D linearAccelerationXY = new FrameVector2D(ReferenceFrame.getWorldFrame(),
                           //                                                                                  linearAcceleration.getX(),
                           //                                                                                  linearAcceleration.getY());
                           //                           //  If ankle tracker is not decelerating
                           //                           if (linearAccelerationXY.norm() > defaultAccelerationThreshold.getValue())
                           //                           {
                           //                              Vector3D translationTracker = new Vector3D();
                           //                              translationTracker.sub(currentTrackerTransform.getTranslation(), initialTrackerTransform.getTranslation());
                           //                              FrameVector2D translationTrackerXY = computeDirectionAdjustment(side, translationTracker);
                           //                              applyStrideScaling(translationTrackerXY,
                           //                                                 translationTracker.norm(),
                           //                                                 linearAccelerationXY.norm(),
                           //                                                 Math.abs(translationTracker.getZ()));
                           //
                           //                              RigidBodyTransformReadOnly robotFootstepTransformInWorld = computeTargetFootstepFromStance(latestInput,
                           //                                                                                                                         side,
                           //                                                                                                                         translationTrackerXY,
                           //                                                                                                                         initialTrackerTransform);
                           //                              FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
                           //                              outputStatus.setRobotSide(side.toByte());
                           //                              outputStatus.setAdjustmentFootstep(true);
                           //                              outputStatus.setLastAdjustment(false);
                           //                              outputStatus.getDesiredFootOrientation().set(robotFootstepTransformInWorld.getRotation());
                           //                              outputStatus.getDesiredFootPosition().set(robotFootstepTransformInWorld.getTranslation());
                           //                              tools.getStatusOutputManager().reportStatusMessage(outputStatus);
                           //                           }
                           //                           else if (!lastDirectionAdjusment)
                           //                           {
                           //                              // If acceleration is very low, do not update direction any further.
                           //                              // Just keep the desired direction as is
                           //                              FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
                           //                              outputStatus.setRobotSide(side.toByte());
                           //                              outputStatus.setAdjustmentFootstep(true);
                           //                              outputStatus.setLastAdjustment(true);
                           //                              tools.getStatusOutputManager().reportStatusMessage(outputStatus);
                           //
                           //                              lastDirectionAdjusment = true;
                           //                           }
                           //                        }
                        }

                        // Update the previous tracker position for the next iteration
                        previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                     }
                  }
               }
               else
               {
                  LogTools.warn("Did not receive information for side {}", side.getSideNameFirstLetter());
               }
            } // End side
         } // End command

         // Updating some statistics
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

   private RigidBodyTransformReadOnly computeTargetFootstepFromStance(FootstepStreamingToolboxInputCommand latestInput,
                                                                      RobotSide side,
                                                                      FrameVector2DReadOnly translationTrackerXY,
                                                                      RigidBodyTransformReadOnly initialTrackerTransform)
   {
      // Apply the translation to the initial tracker position
      FramePoint2D initialTrackerXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                       initialTrackerTransform.getTranslationX(),
                                                       initialTrackerTransform.getTranslationY());
      FramePoint2D predictedTrackerXY = new FramePoint2D(initialTrackerXY);
      predictedTrackerXY.add(translationTrackerXY);

      // Get predicted tracker position in stance frame
      ReferenceFrame stanceTrackerFrame = new FixedReferenceFrame("stanceTracker",
                                                                  ReferenceFrame.getWorldFrame(),
                                                                  initialTrackersTransform.get(side.getOppositeSide()));
      predictedTrackerXY.changeFrameAndProjectToXYPlane(stanceTrackerFrame);

      // Get the current robot stance foot position in world
      FramePose3D robotStanceFootTransformInWorld = latestInput.getInputFor(side.getOppositeSide()).getRobotFootPose();
      ReferenceFrame robotStanceFootFrame = new FixedReferenceFrame("robotStanceFoot", ReferenceFrame.getWorldFrame(), robotStanceFootTransformInWorld);
      FramePoint2D robotPredictedFootXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                           robotStanceFootTransformInWorld.getTranslationX(),
                                                           robotStanceFootTransformInWorld.getTranslationY());
      robotPredictedFootXY.changeFrameAndProjectToXYPlane(robotStanceFootFrame); // This has value 0 now
      // Apply stance-to-predicted-footstep translation
      robotPredictedFootXY.setX(predictedTrackerXY.getX());
      robotPredictedFootXY.setY(predictedTrackerXY.getY());
      robotPredictedFootXY.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      //                     // Compute yaw variation
      //                     double newYaw = initialRobotSwingFootTransformsInWorld.get(side).getRotation().getYaw();
      //                     double yawVariation = currentTrackerTransform.getRotation().getYaw() - initialTrackerTransform.getRotation().getYaw();
      //                     if (Math.toDegrees(yawVariation) >= defaultTurningThreshold.getDoubleValue())
      //                     {
      //                        newYaw += Math.toRadians(defaultTurnDegrees.getDoubleValue());
      //                     }
      //                     else if (Math.toDegrees(yawVariation) <= -defaultTurningThreshold.getDoubleValue())
      //                     {
      //                        newYaw -= Math.toRadians(defaultTurnDegrees.getDoubleValue());
      //                     }
      //                     // Update yaw of footstep
      //                     initialRobotSwingFootTransformsInWorld.get(side).getRotation().setYawPitchRoll(newYaw,
      //                                                                                                    initialRobotSwingFootTransformsInWorld.get(side).getRotation().getPitch(),
      //                                                                                                    initialRobotSwingFootTransformsInWorld.get(side).getRotation().getRoll());

      RigidBodyTransform robotFootstepTransformInWorld = new RigidBodyTransform(initialRobotSwingFootTransformsInWorld.get(side));
      robotFootstepTransformInWorld.getTranslation().setX(robotPredictedFootXY.getX());
      robotFootstepTransformInWorld.getTranslation().setY(robotPredictedFootXY.getY());

      return robotFootstepTransformInWorld;
   }

   private FrameVector2D computeDirectionAdjustment(RobotSide side, Vector3DReadOnly translationTracker)
   {
      // Compute current direction from tracker motion
      FrameVector2D directionTrackerDesired = new FrameVector2D(ReferenceFrame.getWorldFrame(), translationTracker.getX(), translationTracker.getY());
      directionTrackerDesired.normalize();

      // P-control: error = desired_direction - current_controlled_direction
      FrameVector2D directionTrackerCtrl = directionTrackersCtrl.get(side);
      FrameVector2D directionTrackerError = new FrameVector2D(directionTrackerDesired);
      directionTrackerError.sub(directionTrackerCtrl);

      // Update the controlled direction based on error
      directionTrackerCtrl.addX(defaultKpDirection.getValue() * directionTrackerError.getX());
      directionTrackerCtrl.addY(defaultKpDirection.getValue() * directionTrackerError.getY());

      // Re-normalize direction
      directionTrackerCtrl.normalize();
      directionTrackersCtrl.put(side, new FrameVector2D(directionTrackerCtrl));

      return directionTrackerCtrl;
   }

   private void applyStrideScaling(FrameVector2D translationTrackerXYToPack,
                                   double measuredTrackerDistance,
                                   double horizontalTrackerAccelerationMag,
                                   double trackerHeight)
   {
      double rawStride = measuredTrackerDistance + defaultHorizontalAccelerationWeight.getValue() * horizontalTrackerAccelerationMag
                         + defaultVerticalComponentWeight.getValue() * trackerHeight;
      double clampedStride = Math.max(0.0, Math.min(rawStride, defaultStrideLength.getDoubleValue()));

      // Scale the normalized direction to the computed stride distance
      translationTrackerXYToPack.scale(clampedStride);
   }

   @Override
   public void onExit(double timeInState)
   {
      tools.flushInputCommands();
   }

   /**
    * This method estimate the footstep position using the average velocities and swing time.
    * Average Velocity is obtained as   <p> <pre>V<sub>bar</sub> =  ∫<sub>i=0</sub><sup>i=N</sup> v<sub>i</sub></pre>, </p>
    * <pre>v<sub>i</sub> = (P<sup>XY</sup><sub>i</sub> - P<sup>XY</sup><sub>i-1</sub> ) / dt<sub>i</sub></pre>  <br>
    * P<sub>foot step</sub> = P<sub>foot init </sub> + V\u0305 * T<sub>swing</sub> <br>
    * P<sub>foot step</sub> is the predicted / estimated footstep over the ground <br>
    * P<sub>foot init</sub> is the initial swing foot position at the moment the foot starts moving
    * T<sub>swing</sub> is the constant swing duration <br>
    * This assumes <br>
    * - Vive tracker is recorded pretty precise. <br>
    * - Swing duration is constant
    *
    * @param latestInput
    * @param side
    * @return
    */
   public RigidBodyTransformReadOnly estimateFootstepUsingAverageVelocity(FootstepStreamingToolboxInputCommand latestInput,
                                                                          RobotSide side,
                                                                          double swingDuration,
                                                                          RigidBodyTransformReadOnly initialTrackerTransform,
                                                                          FootstepStreamingToolboxTrackerCommand latesCommand)
   {
      FramePoint2D initialTrackerXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                       initialTrackerTransform.getTranslationX(),
                                                       initialTrackerTransform.getTranslationY());
      FramePoint2D predictedTrackerXY = new FramePoint2D(initialTrackerXY);

      accumulatedLinearVelocity.add(latesCommand.getCurrentVelocity().getLinearPart());
      accumulatedAngularVelocity.add(latesCommand.getCurrentVelocity().getAngularPart());
      displacementEstimatedFromLinearVelocity.scaleAdd(swingDuration, accumulatedLinearVelocity);
      angleEstimatedFromAngularVelocity.scaleAdd(swingDuration, accumulatedAngularVelocity);

      predictedTrackerXY.add(displacementEstimatedFromLinearVelocity.getX(), displacementEstimatedFromLinearVelocity.getY());

      FramePose3D robotStanceFootTransformInWorld = latestInput.getInputFor(side.getOppositeSide()).getRobotFootPose();
      ReferenceFrame robotStanceFootFrame = new FixedReferenceFrame("robotStanceFoot", ReferenceFrame.getWorldFrame(), robotStanceFootTransformInWorld);
      FramePoint2D robotPredictedFootXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                           robotStanceFootTransformInWorld.getTranslationX(),
                                                           robotStanceFootTransformInWorld.getTranslationY());
      robotPredictedFootXY.changeFrame(robotStanceFootFrame);
      robotPredictedFootXY.set(predictedTrackerXY);
      robotPredictedFootXY.changeFrame(ReferenceFrame.getWorldFrame());

      RigidBodyTransform robotFootstepTransformInWorld = new RigidBodyTransform(initialRobotSwingFootTransformsInWorld.get(side));

      robotFootstepTransformInWorld.getTranslation().setX(robotPredictedFootXY.getX());
      robotFootstepTransformInWorld.getTranslation().setY(robotPredictedFootXY.getY());


      return robotFootstepTransformInWorld;
   }
}
