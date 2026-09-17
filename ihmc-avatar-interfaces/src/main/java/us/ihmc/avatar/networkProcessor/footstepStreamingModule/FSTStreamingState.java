package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import toolbox_msgs.FootstepStreamingToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI.FootstepStreamingToolboxSideCommand;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This the main class of the Footstep streaming. This is a good starting to understand the mechanics of it.
 */
public class FSTStreamingState implements State
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FSTTools tools;
   private final SideDependentList<Boolean> isUserStepping = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> initialTrackersTransform = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> previousTrackersTransform = new SideDependentList<>();
   private final SideDependentList<Integer> stableIterationCounts = new SideDependentList<>();
   private final SideDependentList<FrameVector2D> directionTrackersCtrl = new SideDependentList<>(); // Controlled direction vector for each foot.
   private final SideDependentList<RigidBodyTransform> initialRobotSwingFootTransformsInWorld = new SideDependentList<>();
   private final SideDependentList<Double> maxFeetHeight = new SideDependentList<>(0.0, 0.0);
   private double robotStepDuration = 0.0;
   private double robotSwingDuration = 0.0;
   private double robotElapsedTimeCurrentStep = 0.0;
   private RobotSide robotSwingSide;
   private boolean robotSwingFootIsLanding = false;
   private double currentStrideEstimate;
   private double currentYawEstimate;
   private double velocitySum = 0.0;
   private int velocityCount = 0;
   private double yawDotSum = 0.0;
   private int yawDotCount = 0;
   private boolean latestAdjustmentSent = false;

   private final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
   private final YoDouble timeSinceLastInput = new YoDouble("timeSinceLastInput", registry);
   private final YoDouble rawInputFrequency = new YoDouble("rawInputFrequency", registry);
   private final AlphaFilteredYoVariable inputFrequency;

   private final YoBoolean computeStepFromStance = new YoBoolean("computeStepFromStance", registry);
   private final YoDouble stepThreshold = new YoDouble("stepThreshold", registry);
   private final YoDouble liftThreshold = new YoDouble("liftThreshold", registry);
   private final YoDouble stabilityThreshold = new YoDouble("stabilityThreshold", registry);
   private final YoInteger stabilityIterations = new YoInteger("stabilityIterations", registry);
   private final YoDouble footstepMarginTime = new YoDouble("footstepMarginTime", registry);
   private final YoDouble defaultStride = new YoDouble("defaultStride", registry);
   private final YoDouble maxStride = new YoDouble("maxStride", registry);
   private final YoDouble maxDistanceToStance = new YoDouble("maxDistanceToStance", registry);
   private final YoDouble minDistanceToStance = new YoDouble("minDistanceToStance", registry);
   private final YoDouble kpDirection = new YoDouble("kpDirection", registry);
   private final YoDouble kpStride = new YoDouble("kpStride", registry);
   private final YoDouble strideVelocityScalingFactor = new YoDouble("strideVelocityScalingFactor", registry);
   private final YoDouble yawVelocityScalingFactor = new YoDouble("yawVelocityScalingFactor", registry);
   private final YoDouble defaultTurningThresholdDegrees = new YoDouble("defaultTurningThreshold", registry);
   private final YoDouble defaultTurnDegrees = new YoDouble("defaultTurn", registry);
   private final YoDouble maxYawRotation = new YoDouble("maxYawRotation", registry);
   private final YoDouble kpYaw = new YoDouble("kpYaw", registry);
   private final YoDouble maxYawToStance = new YoDouble("maxYawToStance", registry);
   private final YoDouble minYawToStance = new YoDouble("minYawToStance", registry);

   private final YoDouble yoRobotStepDuration = new YoDouble("robotStepDuration", registry);
   private final YoDouble yoRobotElapsedTimeCurrentStep = new YoDouble("robotElapsedTimeCurrentStep", registry);
   private final YoInteger yoRobotSwingSide = new YoInteger("robotSwingSide", registry);
   private final YoBoolean yoRobotSwingFootIsLanding =  new YoBoolean("robotSwingFootIsLanding", registry);
   private final YoFramePoint3D yoInitialRobotSwingFootPosition = new YoFramePoint3D("robotInitialSwingPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble yoInitialRobotSwingFootYaw = new YoDouble("robotInitialSwingYaw", registry);
   private final YoFramePoint3D yoRobotSwingFootPosition = new YoFramePoint3D("robotSwingPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble yoRobotSwingFootYaw = new YoDouble("robotSwingYaw", registry);

   private final SideDependentList<YoFramePoint3D> yoInitialTrackersPosition =  new SideDependentList<>();
   private final SideDependentList<YoDouble> yoInitialTrackersYaw = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> yoTrackersPosition =  new SideDependentList<>();
   private final SideDependentList<YoDouble> yoTrackersYaw =  new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> yoTrackersTranslation =  new SideDependentList<>();
   private final SideDependentList<YoDouble> yoTrackersYawVariation =  new SideDependentList<>();

   private final SideDependentList<YoBoolean> yoIsUserStepping =  new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> yoDesiredFootPosition =  new SideDependentList<>();
   private final SideDependentList<YoDouble> yoDesiredFootYaw =  new SideDependentList<>();

   private final YoDouble yoLandingFactorStride = new YoDouble("landingFactorStride", registry);
   private final YoDouble yoRawStride = new YoDouble("rawStride", registry);
   private final YoDouble yoMeasuredStride = new YoDouble("measuredStride", registry);
   private final YoDouble yoDesiredStride = new YoDouble("desiredStride", registry);
   private final YoDouble yoErrorStride = new YoDouble("errorStride", registry);
   private final YoDouble yoControlledStride = new YoDouble("controlledStride", registry);

   public FSTStreamingState(FSTTools tools)
   {
      FootstepStreamingToolboxParameters parameters = tools.getParameters();
      this.tools = tools;
      double toolboxControllerPeriod = tools.getToolboxControllerPeriod();

      tools.getRegistry().addChild(registry);

      YoDouble inputFrequencyAlpha = new YoDouble("inputFrequencyFilter", registry);
      inputFrequencyAlpha.set(AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(2.0, toolboxControllerPeriod));
      inputFrequency = new AlphaFilteredYoVariable("inputFrequency", registry, inputFrequencyAlpha, rawInputFrequency);

      stepThreshold.set(parameters.getStepThreshold());
      liftThreshold.set(parameters.getLiftThreshold());
      stabilityThreshold.set(parameters.getStabilityThreshold());
      stabilityIterations.set(parameters.getStabilityIterations());
      footstepMarginTime.set(parameters.getFootstepMarginTime());
      defaultStride.set(parameters.getDefaultStride());
      maxStride.set(parameters.getMaxStride());
      maxDistanceToStance.set(parameters.getMaxDistanceToStance());
      minDistanceToStance.set(parameters.getMinDistanceToStance());
      kpDirection.set(parameters.getKpDirection());
      kpStride.set(parameters.getKpStride());
      strideVelocityScalingFactor.set(parameters.getStrideVelocityScalingFactor());
      yawVelocityScalingFactor.set(parameters.getYawVelocityScalingFactor());
      defaultTurningThresholdDegrees.set(parameters.getTurningThresholdDegrees());
      defaultTurnDegrees.set(parameters.getTurnDegrees());
      maxYawRotation.set(Math.toRadians(parameters.getMaxYawRotationDegrees()));
      kpYaw.set(parameters.getKpYaw());
      maxYawToStance.set(Math.toRadians(parameters.getMaxYawToStanceDegrees()));
      minYawToStance.set(Math.toRadians(parameters.getMinYawToStanceDegrees()));
      computeStepFromStance.set(parameters.getComputeFromStance());

      for (RobotSide side : RobotSide.values)
      {
         yoInitialTrackersPosition.put(side, new YoFramePoint3D(side + "_initialTrackerPosition", ReferenceFrame.getWorldFrame(), registry));
         yoInitialTrackersYaw.put(side, new YoDouble(side + "_initialTrackerYaw", registry));
         yoTrackersPosition.put(side, new YoFramePoint3D(side + "_trackerPosition", ReferenceFrame.getWorldFrame(), registry));
         yoTrackersYaw.put(side, new YoDouble(side + "_trackerYaw", registry));
         yoTrackersTranslation.put(side, new YoFramePoint3D(side + "_trackerTranslation", ReferenceFrame.getWorldFrame(), registry));
         yoTrackersYawVariation.put(side, new YoDouble(side + "_trackerYawVariation", registry));

         yoIsUserStepping.put(side, new YoBoolean(side + "_isUserStepping", registry));
         yoDesiredFootPosition.put(side, new YoFramePoint3D(side + "_desiredFootPosition", ReferenceFrame.getWorldFrame(), registry));
         yoDesiredFootYaw.put(side, new YoDouble(side + "_desiredFootYaw", registry));
      }
   }

   @Override
   public void onEntry()
   {
      LogTools.debug("Footstep Streaming enabled");
      for (RobotSide side : RobotSide.values())
      {
         isUserStepping.put(side, false);
         yoIsUserStepping.get(side).set(false);
         stableIterationCounts.put(side, 0);
         previousTrackersTransform.put(side, new RigidBodyTransform());
         // Initialize controlled direction
         directionTrackersCtrl.put(side, new FrameVector2D());
         currentStrideEstimate = defaultStride.getValue();
         currentYawEstimate = Math.toRadians(defaultTurnDegrees.getValue());

         initialTrackersTransform.put(side, null);
         initialRobotSwingFootTransformsInWorld.put(side, null);
         maxFeetHeight.put(side, 0.0);
      }

      velocitySum = 0.0;
      velocityCount = 0;
      yawDotSum = 0.0;
      yawDotCount = 0;
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
            robotStepDuration = latestInput.getRobotStepDuration();
            robotSwingDuration = latestInput.getRobotStepDuration();
            robotElapsedTimeCurrentStep = latestInput.getRobotElapsedTimeCurrentStep();

            yoRobotStepDuration.set(robotStepDuration);
            yoRobotElapsedTimeCurrentStep.set(robotElapsedTimeCurrentStep);

            if (robotElapsedTimeCurrentStep == 0.0)
               latestAdjustmentSent = false;

            robotSwingSide = latestInput.getRobotSwingSide();
            robotSwingFootIsLanding = latestInput.isRobotSwingFootLanding();

            yoRobotSwingSide.set(robotSwingSide.toByte());
            yoRobotSwingFootIsLanding.set(robotSwingFootIsLanding);

            if (robotElapsedTimeCurrentStep > robotStepDuration)
               robotElapsedTimeCurrentStep = robotStepDuration;

            for (RobotSide side : RobotSide.values)
            {
               FootstepStreamingToolboxSideCommand sideCommand = latestInput.getInputFor(side);
               if (sideCommand != null)
               {
                  RigidBodyTransform currentTrackerTransform = new RigidBodyTransform(sideCommand.getCurrentPose());
                  RigidBodyTransform initialTrackerTransform = initialTrackersTransform.get(side);
                  RigidBodyTransform currentRobotFootTransformInWorld = new RigidBodyTransform(sideCommand.getRobotFootPose());

                  yoTrackersPosition.get(side).set(currentTrackerTransform.getTranslation());
                  yoTrackersYaw.get(side).set(currentTrackerTransform.getRotation().getYaw());
                  if (side.equals(robotSwingSide))
                  {
                     yoRobotSwingFootPosition.set(currentRobotFootTransformInWorld.getTranslation());
                     yoRobotSwingFootYaw.set(currentRobotFootTransformInWorld.getRotation().getYaw());
                  }

                  if (initialTrackerTransform == null)
                  {
                     initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                     previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                     initialRobotSwingFootTransformsInWorld.put(side, new RigidBodyTransform(currentRobotFootTransformInWorld));

                     yoInitialTrackersPosition.get(side).set(initialTrackersTransform.get(side).getTranslation());
                     yoInitialTrackersYaw.get(side).set(initialTrackersTransform.get(side).getRotation().getYaw());
                     if (side.equals(robotSwingSide))
                     {
                        yoInitialRobotSwingFootPosition.set(initialRobotSwingFootTransformsInWorld.get(side).getTranslation());
                        yoInitialRobotSwingFootYaw.set(initialRobotSwingFootTransformsInWorld.get(side).getRotation().getYaw());
                     }
                  }

                  // Get translation of current tracker position wrt to initial one
                  Vector3D translationTracker = new Vector3D();
                  translationTracker.sub(currentTrackerTransform.getTranslation(), initialTrackersTransform.get(side).getTranslation());
                  FrameVector2D translationTrackerXY = new FrameVector2D(ReferenceFrame.getWorldFrame(),
                                                                         translationTracker.getX(),
                                                                         translationTracker.getY());

                  if (translationTracker.getZ() > maxFeetHeight.get(side))
                  {
                     maxFeetHeight.put(side, translationTracker.getZ());
                  }

                  double rotationTracker = currentTrackerTransform.getRotation().getYaw() - initialTrackersTransform.get(side).getRotation().getYaw();

                  yoTrackersTranslation.get(side).set(translationTracker);
                  yoTrackersYawVariation.get(side).set(rotationTracker);

                  if (!isUserStepping.get(side)) // Tracker is not moving by a lot yet
                  {
                     // Check if the tracker has moved in any direction AND the foot has been lifted
                     if (translationTrackerXY.norm() >= stepThreshold.getDoubleValue()
                         && translationTracker.getZ() >= liftThreshold.getDoubleValue())
                     {
                        // Normalize the translation direction to have a fixed stride distance
                        translationTrackerXY.normalize();
                        directionTrackersCtrl.put(side, translationTrackerXY);
                        // Scale the normalized direction to the fixed stride distance
                        translationTrackerXY.scale(defaultStride.getDoubleValue());

                        // Use default rotation is the yaw variation exceeds threshold
                        double defaultYawRotation = Math.abs(rotationTracker) > Math.toRadians(defaultTurningThresholdDegrees.getValue()) ? Math.toRadians(defaultTurnDegrees.getValue()) * Math.signum(rotationTracker) : 0.0;

                        // Compute robot footstep from estimate
                        RigidBodyTransformReadOnly robotFootstepTransformInWorld = computeStepFromStance.getValue() ?
                              computeTargetFootstepFromStance(latestInput, side, translationTrackerXY, defaultYawRotation, initialTrackersTransform.get(side)) :
                              computeTargetFootstepFromInitialSwing(side, translationTrackerXY, defaultYawRotation, initialTrackersTransform.get(side));
                        // Publish footstep to UI
                        FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
                        outputStatus.setAdjustmentFootstep(false); // first estimate
                        outputStatus.setLastAdjustment(false);
                        outputStatus.setRobotSide(side.toByte());
                        outputStatus.getDesiredFootOrientation().set(robotFootstepTransformInWorld.getRotation());
                        outputStatus.getDesiredFootPosition().set(robotFootstepTransformInWorld.getTranslation());
                        tools.getStatusOutputManager().reportStatusMessage(outputStatus);
                        isUserStepping.put(side, true);
                        // Avoid false stepping detection when already in swing with one side
                        isUserStepping.put(side.getOppositeSide(), false);

                        yoIsUserStepping.get(side).set(true);
                        yoIsUserStepping.get(side.getOppositeSide()).set(false);
                     }
                     else
                     {
                        // set now the initial position for robot stance foot, which will be the next swing
                        initialRobotSwingFootTransformsInWorld.put(side, new RigidBodyTransform(currentRobotFootTransformInWorld));
                        if (side.equals(robotSwingSide))
                        {
                           yoInitialRobotSwingFootPosition.set(initialRobotSwingFootTransformsInWorld.get(side).getTranslation());
                           yoInitialRobotSwingFootYaw.set(initialRobotSwingFootTransformsInWorld.get(side).getRotation().getYaw());
                        }
                        // set first value for control of direction adjustment, used later
                        directionTrackersCtrl.put(side, new FrameVector2D());
                     }
                  }
                  else // Already stepping, tracker is moving
                  {
                     // Avoid false stepping detection when already in swing with one side
                     isUserStepping.put(side.getOppositeSide(), false);
                     yoIsUserStepping.get(side.getOppositeSide()).set(false);
                     if (initialTrackerTransform != null)
                     {
                        // Check if tracker is not moving much anymore
                        Vector3D translationFromPreviousPosition = new Vector3D();
                        translationFromPreviousPosition.sub(currentTrackerTransform.getTranslation(), previousTrackersTransform.get(side).getTranslation());

                        // If the tracker is not moving much, then start counting for stability
                        if (translationFromPreviousPosition.norm() <= stabilityThreshold.getDoubleValue() &&
                            translationTracker.getZ() < liftThreshold.getDoubleValue())
                        {
                           int stableCount = stableIterationCounts.get(side);
                           stableCount++;
                           stableIterationCounts.put(side, stableCount);

                           if (stableCount >= stabilityIterations.getIntegerValue())
                           {
                              if (!latestAdjustmentSent)
                              {
                                 FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
                                 outputStatus.setRobotSide(side.toByte());
                                 outputStatus.setAdjustmentFootstep(true);
                                 outputStatus.setLastAdjustment(true);
                                 tools.getStatusOutputManager().reportStatusMessage(outputStatus);
                                 latestAdjustmentSent = true;
                                 LogTools.debug("Sent last footstep adjustment {}", side);
                              }

                              reset(side, currentTrackerTransform);
                              LogTools.debug("User completed stepping with {}", side);
                           }
                        }
                        else  // Still moving
                        {
                           stableIterationCounts.put(side, 0); // reset stability count
                           if (!tools.hasPreviousInput())
                           {
                              LogTools.error("Cannot update footstep estimate because previous input is missing");
                           }
                           else // Send adjustment
                           {
                              LogTools.debug("side: {}, landing: {}", robotSwingSide, robotSwingFootIsLanding);
                              LogTools.debug("latestAdjustment: {}", latestAdjustmentSent);

                              if((robotElapsedTimeCurrentStep < robotStepDuration - footstepMarginTime.getValue())
                                 && !latestAdjustmentSent
                                 && !robotSwingFootIsLanding)
                              {
                                 FrameVector2D adjustedTranslationTrackerXY = new FrameVector2D(ReferenceFrame.getWorldFrame(), translationTracker.getX(), translationTracker.getY());
                                 double stride = defaultStride.getDoubleValue();
                                 if (sideCommand.getHasCurrentVelocity())
                                 {
                                    stride = computeStrideEstimate(side,
                                                                   adjustedTranslationTrackerXY.norm(),
                                                                   translationTracker.getZ(),
                                                                   sideCommand.getCurrentVelocity().getLinearPart());
                                 }
                                 adjustedTranslationTrackerXY.normalize();
                                 adjustedTranslationTrackerXY.scale(stride);

                                 double adjustedYawRotation = computeYawEstimate(side,
                                                                                 rotationTracker,
                                                                                 translationTracker.getZ(),
                                                                                 sideCommand.getCurrentVelocity().getLinearPartZ(),
                                                                                 sideCommand.getCurrentVelocity().getAngularPartZ());

                                 RigidBodyTransformReadOnly robotFootstepTransformInWorld = computeStepFromStance.getValue() ?
                                       computeTargetFootstepFromStance(latestInput, side, adjustedTranslationTrackerXY, adjustedYawRotation, initialTrackerTransform) :
                                       computeTargetFootstepFromInitialSwing(side, adjustedTranslationTrackerXY, adjustedYawRotation, initialTrackerTransform);

                                 FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
                                 outputStatus.setRobotSide(side.toByte());
                                 outputStatus.setAdjustmentFootstep(true);
                                 outputStatus.setLastAdjustment(false);
                                 outputStatus.getDesiredFootOrientation().set(robotFootstepTransformInWorld.getRotation());
                                 outputStatus.getDesiredFootPosition().set(robotFootstepTransformInWorld.getTranslation());
                                 tools.getStatusOutputManager().reportStatusMessage(outputStatus);
                                 LogTools.debug("Sent footstep adjustment {}", side);

                                 yoDesiredFootPosition.get(side).set(robotFootstepTransformInWorld.getTranslation());
                                 yoDesiredFootYaw.get(side).set(robotFootstepTransformInWorld.getRotation().getYaw());
                              }
                              else if (!latestAdjustmentSent)
                              {
                                 // Do not update direction any further.
                                 // Just keep the desired direction as is
                                 FootstepStreamingToolboxOutputStatus outputStatus = new FootstepStreamingToolboxOutputStatus();
                                 outputStatus.setRobotSide(side.toByte());
                                 outputStatus.setAdjustmentFootstep(true);
                                 outputStatus.setLastAdjustment(true);
                                 tools.getStatusOutputManager().reportStatusMessage(outputStatus);
                                 latestAdjustmentSent = true;
                                 LogTools.debug("Sent last footstep adjustment {}", side);
                              }
                           }
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

   private void reset(RobotSide side, RigidBodyTransform currentTrackerTransform)
   {
      isUserStepping.put(side, false);
      yoIsUserStepping.get(side).set(false);

      directionTrackersCtrl.put(side, new FrameVector2D());
      initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
      yoInitialTrackersPosition.get(side).set(initialTrackersTransform.get(side).getTranslation());
      yoInitialTrackersYaw.get(side).set(initialTrackersTransform.get(side).getRotation().getYaw());
      stableIterationCounts.put(side, 0);
      maxFeetHeight.put(side, 0.0);
      velocitySum = 0.0;
      velocityCount = 0;
      yawDotSum = 0.0;
      yawDotCount = 0;
      latestAdjustmentSent = false;
   }

   private RigidBodyTransformReadOnly computeTargetFootstepFromInitialSwing(RobotSide side,
                                                                            FrameVector2DReadOnly translationTrackerXY,
                                                                            double yawRotationTracker,
                                                                            RigidBodyTransformReadOnly initialTrackerTransform)
   {
      // Apply the translation to the initial tracker position
      FramePoint2D initialTrackerXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                       initialTrackerTransform.getTranslationX(),
                                                       initialTrackerTransform.getTranslationY());
      FramePoint2D predictedTrackerXY = new FramePoint2D(initialTrackerXY);
      predictedTrackerXY.add(translationTrackerXY);

      // Create the initial swing frame
      ReferenceFrame initialSwingTrackerFrame = new FixedReferenceFrame("initialSwingTracker",
                                                                  ReferenceFrame.getWorldFrame(),
                                                                        initialTrackerTransform);
      // Get predicted tracker position in initial swing frame
      predictedTrackerXY.changeFrameAndProjectToXYPlane(initialSwingTrackerFrame);

      // Get the initial robot swing foot position in world
      RigidBodyTransform robotInitialSwingFootTransformInWorld = initialRobotSwingFootTransformsInWorld.get(side);
      ReferenceFrame robotInitialSwingFootFrame = new FixedReferenceFrame("robotInitialSwingFoot", ReferenceFrame.getWorldFrame(), robotInitialSwingFootTransformInWorld);
      FramePoint2D robotPredictedFootXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                           robotInitialSwingFootTransformInWorld.getTranslationX(),
                                                           robotInitialSwingFootTransformInWorld.getTranslationY());
      robotPredictedFootXY.changeFrameAndProjectToXYPlane(robotInitialSwingFootFrame); // This has value 0 now

      // Apply initial-swing-to-predicted-footstep translation
      robotPredictedFootXY.setX(predictedTrackerXY.getX());
      robotPredictedFootXY.setY(predictedTrackerXY.getY());
      robotPredictedFootXY.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      // Compute yaw rotation
      double newYaw = robotInitialSwingFootTransformInWorld.getRotation().getYaw() + yawRotationTracker;

      RigidBodyTransform robotFootstepTransformInWorld = new RigidBodyTransform(robotInitialSwingFootTransformInWorld);
      robotFootstepTransformInWorld.getTranslation().setX(robotPredictedFootXY.getX());
      robotFootstepTransformInWorld.getTranslation().setY(robotPredictedFootXY.getY());
      robotFootstepTransformInWorld.getRotation().setYawPitchRoll(newYaw,
                                                                  robotFootstepTransformInWorld.getRotation().getPitch(),
                                                                  robotFootstepTransformInWorld.getRotation().getRoll());

      return robotFootstepTransformInWorld;
   }

   private RigidBodyTransformReadOnly computeTargetFootstepFromStance(FootstepStreamingToolboxInputCommand latestInput,
                                                                      RobotSide side,
                                                                      FrameVector2DReadOnly translationTrackerXY,
                                                                      double yawRotationTracker,
                                                                      RigidBodyTransformReadOnly initialTrackerTransform)
   {
      // Apply the translation to the initial tracker position
      FramePoint2D initialTrackerXY = new FramePoint2D(ReferenceFrame.getWorldFrame(),
                                                       initialTrackerTransform.getTranslationX(),
                                                       initialTrackerTransform.getTranslationY());
      FramePoint2D predictedTrackerXY = new FramePoint2D(initialTrackerXY);
      predictedTrackerXY.add(translationTrackerXY);

      // Create the stance frame
      ReferenceFrame stanceTrackerFrame = new FixedReferenceFrame("stanceTracker",
                                                                  ReferenceFrame.getWorldFrame(),
                                                                  initialTrackersTransform.get(side.getOppositeSide()));
      // Get predicted tracker position in stance frame
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
      // Clamp value of lateral distance according to limits relative to stance foot
      if (side == RobotSide.LEFT)
         robotPredictedFootXY.setY(Math.max(minDistanceToStance.getValue(), Math.min(maxDistanceToStance.getValue(), robotPredictedFootXY.getY())));
      else
         robotPredictedFootXY.setY(Math.max(-maxDistanceToStance.getValue(), Math.min(-minDistanceToStance.getValue(), robotPredictedFootXY.getY())));
      robotPredictedFootXY.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      // Compute predicted rotation relative to stance frame
      double initialSwingTrackerYaw = initialTrackerTransform.getRotation().getYaw();
      double initialStanceTrackerYaw = initialTrackersTransform.get(side.getOppositeSide()).getRotation().getYaw();
      double newYawInStanceFrame = yawRotationTracker + (initialSwingTrackerYaw - initialStanceTrackerYaw);

      // Clamp value of yaw according to limits relative to stance foot
      newYawInStanceFrame = side == RobotSide.LEFT ?
            Math.max(minYawToStance.getValue(), Math.min(maxYawToStance.getValue(), newYawInStanceFrame)) :
            Math.max(-maxYawToStance.getValue(), Math.min(minYawToStance.getValue(), newYawInStanceFrame));
      // Update yaw value based on prediction expressed in robot stance foot
      double newYaw = robotStanceFootTransformInWorld.getRotation().getYaw() + newYawInStanceFrame;

      RigidBodyTransform robotFootstepTransformInWorld = new RigidBodyTransform(robotStanceFootTransformInWorld);
      robotFootstepTransformInWorld.getTranslation().setX(robotPredictedFootXY.getX());
      robotFootstepTransformInWorld.getTranslation().setY(robotPredictedFootXY.getY());
      robotFootstepTransformInWorld.getRotation().setYawPitchRoll(newYaw,
                                                                  robotFootstepTransformInWorld.getRotation().getPitch(),
                                                                  robotFootstepTransformInWorld.getRotation().getRoll());

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
      directionTrackerCtrl.addX(kpDirection.getValue() * directionTrackerError.getX());
      directionTrackerCtrl.addY(kpDirection.getValue() * directionTrackerError.getY());

      // Re-normalize direction
      directionTrackerCtrl.normalize();
      directionTrackersCtrl.put(side, new FrameVector2D(directionTrackerCtrl));

      return directionTrackerCtrl;
   }

   public double computeStrideEstimate(RobotSide side,
                                       double measuredHorizontalDistance,
                                       double verticalPosition,
                                       FixedFrameVector3DBasics linearVelocity)
   {

      // 1) Basic horizontal-based raw stride estimate
      double rawStride = measuredHorizontalDistance + strideVelocityScalingFactor.getValue() * getAverageHorizontalVelocity(linearVelocity) * (robotSwingDuration
                                                                                                                                               - robotElapsedTimeCurrentStep);
      // 2) "Landing factor" from vertical motion
      //    If the foot is descending (verticalVel < 0), we reduce the stride.
      //    One approach is an interpolation factor landingFactor in [0,1], where 1 => no reduction,
      //    0 => fully trust measured distance only.
      double landingFactor = 1.0; // default is 1 => no reduction
      if (linearVelocity.getZ() < 0.0)
      {
         // Normalize foot height to [0, 1]
         landingFactor = verticalPosition / maxFeetHeight.get(side);
         landingFactor = Math.max(0.0, Math.min(1.0, landingFactor));
      }
      yoLandingFactorStride.set(landingFactor);

      // 3) The stride is pulled toward the measuredDistance if foot is descending
      double blendedStride = landingFactor * rawStride
                             + (1.0 - landingFactor) * measuredHorizontalDistance;
      yoRawStride.set(rawStride);
      yoMeasuredStride.set(measuredHorizontalDistance);

      // 4) Clamp to [0, maxStride] pre-P-control
      double desiredStride = Math.max(0.0, Math.min(blendedStride, maxStride.getValue()));
      yoDesiredStride.set(desiredStride);

      // 5) Apply P-control
      double error = desiredStride - currentStrideEstimate;
      yoErrorStride.set(error);
      double newStrideEstimate = currentStrideEstimate + kpStride.getValue() * error;

      // 6) Final clamp
      newStrideEstimate = Math.max(0.0, Math.min(newStrideEstimate, maxStride.getValue()));
      currentStrideEstimate = newStrideEstimate;
      yoControlledStride.set(currentStrideEstimate);

      return currentStrideEstimate;
   }

   private double getAverageHorizontalVelocity(FixedFrameVector3DBasics currentLinearVelocity)
   {
      FrameVector2D currentXY = new FrameVector2D(ReferenceFrame.getWorldFrame());
      currentXY.set(currentLinearVelocity);

      velocitySum += currentXY.norm();
      velocityCount++;
      return velocitySum / velocityCount;
   }

   public double computeYawEstimate(RobotSide side,
                                    double measuredYawRotation,
                                    double verticalPosition,
                                    double linearVerticalVelocity,
                                    double angularVelocity)
   {
      // 1) Basic raw yaw rotation estimate
      double rawYawRotation = measuredYawRotation + yawVelocityScalingFactor.getValue() *
                                                    getAverageAngularVelocity(angularVelocity) * (robotSwingDuration - robotElapsedTimeCurrentStep);

      // 2) "Landing factor" from vertical motion
      //    Interpolation factor landingFactor in [0,1], where 1 => no reduction,
      //    0 => fully trust measured rotation only.
      double landingFactor = 1.0; // default is 1 => no reduction
      if (linearVerticalVelocity < 0.0)
      {
         // Normalize foot height to [0, 1]
         landingFactor = verticalPosition / maxFeetHeight.get(side);
         landingFactor = Math.max(0.0, Math.min(1.0, landingFactor));
      }

      // 3) The rotation is pulled toward the measuredYaw if foot is descending
      double blendedYawRotation = landingFactor * rawYawRotation
                             + (1.0 - landingFactor) * measuredYawRotation;

      // 4) Clamp to [0, maxYawRotation] pre-P-control
      double desiredYawRotation = Math.max(-maxYawRotation.getValue(), Math.min(blendedYawRotation, maxYawRotation.getValue()));

      // 5) Apply P-control
      double error = desiredYawRotation - currentYawEstimate;
      double newYawEstimate = currentYawEstimate + kpYaw.getValue() * error;

      // 6) Final clamp
      newYawEstimate = Math.max(-maxYawRotation.getValue(), Math.min(newYawEstimate, maxYawRotation.getValue()));
      currentYawEstimate = newYawEstimate;

      return currentYawEstimate;
   }

   private double getAverageAngularVelocity(double currentAngularVelocity)
   {
      yawDotSum += currentAngularVelocity;
      yawDotCount++;
      return yawDotSum / yawDotCount;
   }

   @Override
   public void onExit(double timeInState)
   {
      LogTools.debug("Footstep Streaming disabled");
      tools.flushInputCommands();
   }
}