package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitPlanner;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsInputProvider;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.quadrupedRobotics.util.YoPreallocatedList;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.euclid.tuple3D.Vector3D;
import java.util.ArrayList;

public class QuadrupedXGaitStepStream implements QuadrupedStepStream
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter initialStepDelayParameter = parameterFactory.createDouble("initialStepDelay", 0.5);
   private final DoubleParameter minimumStepClearanceParameter = parameterFactory.createDouble("minimumStepClearance", 0.075);
   private static int NUMBER_OF_PREVIEW_STEPS = 16;

   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;
   private final QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider;
   private final FramePoint supportCentroid;
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final ReferenceFrame worldFrame;
   private final double controlDT;
   private final DoubleYoVariable timestamp;

   private final DoubleYoVariable bodyYaw;
   private final YoFrameOrientation bodyOrientation;
   private final QuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final EndDependentList<YoQuadrupedTimedStep> xGaitCurrentSteps;
   private final ArrayList<YoQuadrupedTimedStep> xGaitPreviewSteps;
   private final YoPreallocatedList<YoQuadrupedTimedStep> stepSequence;

   public QuadrupedXGaitStepStream(QuadrupedPlanarVelocityInputProvider planarVelocityProvider, QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider,
         QuadrupedReferenceFrames referenceFrames, double controlDT, DoubleYoVariable timestamp, YoVariableRegistry parentRegistry)
   {
      this.planarVelocityProvider = planarVelocityProvider;
      this.xGaitSettingsProvider = xGaitSettingsProvider;
      this.supportCentroid = new FramePoint();
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.worldFrame = ReferenceFrame.getWorldFrame();
      this.controlDT = controlDT;
      this.timestamp = timestamp;

      this.bodyYaw = new DoubleYoVariable("bodyYaw", registry);
      this.bodyOrientation = new YoFrameOrientation("bodyOrientation", worldFrame, registry);
      this.xGaitSettings = new QuadrupedXGaitSettings();
      this.xGaitStepPlanner = new QuadrupedXGaitPlanner();
      this.xGaitCurrentSteps = new EndDependentList<>();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         xGaitCurrentSteps.set(robotEnd, new YoQuadrupedTimedStep("currentStep" + robotEnd.getCamelCaseNameForMiddleOfExpression(), registry));
      }
      this.xGaitPreviewSteps = new ArrayList<>(NUMBER_OF_PREVIEW_STEPS);
      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new YoQuadrupedTimedStep("previewStep" + i, registry));
      }
      this.stepSequence = new YoPreallocatedList<>("stepSequence", registry, NUMBER_OF_PREVIEW_STEPS + 2,
            new YoPreallocatedList.DefaultElementFactory<YoQuadrupedTimedStep>()
            {
               @Override
               public YoQuadrupedTimedStep createDefaultElement(String prefix, YoVariableRegistry registry)
               {
                  return new YoQuadrupedTimedStep(prefix, registry);
               }
            });

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   private void updateXGaitSettings()
   {
      xGaitSettingsProvider.getSettings(xGaitSettings);

      // increase stance dimensions as a function of velocity to prevent self collisions
      double strideRotation = planarVelocityProvider.get().getZ() * xGaitSettings.getStepDuration();
      double strideLength = Math.abs(2 * planarVelocityProvider.get().getX() * xGaitSettings.getStepDuration());
      double strideWidth = Math.abs(2 * planarVelocityProvider.get().getY() * xGaitSettings.getStepDuration());
      strideLength += Math.abs(xGaitSettings.getStanceWidth() / 2 * Math.sin(2 * strideRotation));
      strideWidth += Math.abs(xGaitSettings.getStanceLength() / 2 * Math.sin(2 * strideRotation));
      xGaitSettings.setStanceLength(Math.max(xGaitSettings.getStanceLength(), strideLength / 2 + minimumStepClearanceParameter.get()));
      xGaitSettings.setStanceWidth(Math.max(xGaitSettings.getStanceWidth(), strideWidth / 2 + minimumStepClearanceParameter.get()));
   }

   @Override
   public void onEntry()
   {
      // initialize body orientation
      bodyOrientation.setFromReferenceFrame(bodyZUpFrame);
      bodyYaw.set(bodyOrientation.getYaw().getDoubleValue());

      // initialize step queue
      updateXGaitSettings();
      supportCentroid.setToZero(supportFrame);
      double initialYaw = bodyYaw.getDoubleValue();
      double initialTime = timestamp.getDoubleValue() + initialStepDelayParameter.get();
      Vector3D initialVelocity = planarVelocityProvider.get();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, initialVelocity, initialQuadrant, supportCentroid, initialTime, initialYaw, xGaitSettings);
      for (int i = 0; i < 2; i++)
      {
         RobotEnd robotEnd = xGaitPreviewSteps.get(i).getRobotQuadrant().getEnd();
         xGaitCurrentSteps.get(robotEnd).set(xGaitPreviewSteps.get(i));
      }
      this.process();
   }

   @Override
   public void process()
   {
      double currentTime = timestamp.getDoubleValue();

      // update body orientation
      bodyYaw.add(planarVelocityProvider.get().getZ() * controlDT);
      bodyOrientation.setYawPitchRoll(bodyYaw.getDoubleValue(), 0.0, 0.0);

      // update xgait current steps
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         QuadrupedTimedStep xGaitPreviewStep = xGaitPreviewSteps.get(i);
         if (xGaitPreviewStep.getTimeInterval().getStartTime() <= currentTime)
         {
            xGaitCurrentSteps.get(xGaitPreviewStep.getRobotQuadrant().getEnd()).set(xGaitPreviewStep);
         }
      }

      // update xgait preview steps
      updateXGaitSettings();
      double currentYaw = bodyYaw.getDoubleValue();
      Vector3D inputVelocity = planarVelocityProvider.get();
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, xGaitCurrentSteps, inputVelocity, currentTime, currentYaw, xGaitSettings);

      // update step sequence
      stepSequence.clear();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (xGaitCurrentSteps.get(robotEnd).getTimeInterval().getEndTime() >= currentTime)
         {
            stepSequence.add();
            stepSequence.get(stepSequence.size() - 1).set(xGaitCurrentSteps.get(robotEnd));
         }
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         if (xGaitPreviewSteps.get(i).getTimeInterval().getEndTime() >= currentTime)
         {
            stepSequence.add();
            stepSequence.get(stepSequence.size() - 1).set(xGaitPreviewSteps.get(i));
         }
      }
   }

   @Override
   public void onExit()
   {

   }

   @Override
   public void getBodyOrientation(FrameOrientation bodyOrientation)
   {
      bodyOrientation.setIncludingFrame(this.bodyOrientation.getFrameOrientation());
   }

   @Override
   public PreallocatedList<? extends QuadrupedTimedStep> getSteps()
   {
      return stepSequence;
   }
}
