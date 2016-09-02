package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitPlanner;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedXGaitSettingsInputProvider;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Vector3d;
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
   private final QuadrupedXGaitSettings xGaitSettings;
   private final QuadrupedXGaitPlanner xGaitStepPlanner;
   private final ArrayList<QuadrupedTimedStep> xGaitPreviewSteps;
   private final EndDependentList<QuadrupedTimedStep> xGaitCurrentSteps;
   private final FramePoint supportCentroid;
   private final ReferenceFrame supportFrame;
   private final ReferenceFrame bodyZUpFrame;
   private final ReferenceFrame worldFrame;
   private final double controlDT;
   private final DoubleYoVariable timestamp;
   private final FrameOrientation bodyOrientation;
   private double bodyYaw;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;

   public QuadrupedXGaitStepStream(QuadrupedPlanarVelocityInputProvider planarVelocityProvider, QuadrupedXGaitSettingsInputProvider xGaitSettingsProvider,
         QuadrupedReferenceFrames referenceFrames, double controlDT, DoubleYoVariable timestamp, YoVariableRegistry parentRegistry)
   {
      this.planarVelocityProvider = planarVelocityProvider;
      this.xGaitSettingsProvider = xGaitSettingsProvider;
      this.xGaitSettings = new QuadrupedXGaitSettings();
      this.xGaitStepPlanner = new QuadrupedXGaitPlanner();
      this.xGaitPreviewSteps = new ArrayList<>(NUMBER_OF_PREVIEW_STEPS);
      this.xGaitCurrentSteps = new EndDependentList<>();
      for (int i = 0; i < NUMBER_OF_PREVIEW_STEPS; i++)
      {
         xGaitPreviewSteps.add(new QuadrupedTimedStep());
      }
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         xGaitCurrentSteps.set(robotEnd, new QuadrupedTimedStep());
      }
      this.supportCentroid = new FramePoint();
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      this.bodyZUpFrame = referenceFrames.getBodyZUpFrame();
      this.worldFrame = ReferenceFrame.getWorldFrame();
      this.controlDT = controlDT;
      this.timestamp = timestamp;
      this.bodyOrientation = new FrameOrientation();
      this.stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, NUMBER_OF_PREVIEW_STEPS + 2);

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
      bodyOrientation.setToZero(bodyZUpFrame);
      bodyOrientation.changeFrame(worldFrame);
      bodyYaw = bodyOrientation.getYaw();

      // initialize step queue
      updateXGaitSettings();
      supportCentroid.setToZero(supportFrame);
      double initialTime = timestamp.getDoubleValue() + initialStepDelayParameter.get();
      Vector3d initialVelocity = planarVelocityProvider.get();
      RobotQuadrant initialQuadrant = (xGaitSettings.getEndPhaseShift() < 90) ? RobotQuadrant.HIND_LEFT : RobotQuadrant.FRONT_LEFT;
      xGaitStepPlanner.computeInitialPlan(xGaitPreviewSteps, initialVelocity, initialQuadrant, supportCentroid, initialTime, bodyYaw, xGaitSettings);
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
      bodyYaw += planarVelocityProvider.get().getZ() * controlDT;
      bodyOrientation.changeFrame(worldFrame);
      bodyOrientation.setYawPitchRoll(bodyYaw, 0.0, 0.0);

      // update xgait current steps
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         QuadrupedTimedStep xGaitPreviewStep = xGaitPreviewSteps.get(i);
         if (xGaitPreviewStep.getTimeInterval().getStartTime() <= currentTime)
         {
            xGaitCurrentSteps.get(xGaitPreviewStep.getRobotEnd()).set(xGaitPreviewStep);
         }
      }

      // update xgait preview steps
      updateXGaitSettings();
      Vector3d inputVelocity = planarVelocityProvider.get();
      xGaitStepPlanner.computeOnlinePlan(xGaitPreviewSteps, xGaitCurrentSteps, inputVelocity, currentTime, bodyYaw, xGaitSettings);

      // update step queue
      stepQueue.clear();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (xGaitCurrentSteps.get(robotEnd).getTimeInterval().getEndTime() >= currentTime)
         {
            stepQueue.enqueue();
            stepQueue.getTail().set(xGaitCurrentSteps.get(robotEnd));
         }
      }
      for (int i = 0; i < xGaitPreviewSteps.size(); i++)
      {
         if (xGaitPreviewSteps.get(i).getTimeInterval().getEndTime() >= currentTime)
         {
            stepQueue.enqueue();
            stepQueue.getTail().set(xGaitPreviewSteps.get(i));
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
      bodyOrientation.set(this.bodyOrientation);
   }

   @Override
   public PreallocatedQueue<QuadrupedTimedStep> getSteps()
   {
      return stepQueue;
   }
}
