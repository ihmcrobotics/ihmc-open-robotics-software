package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.LinearInvertedPendulumModel;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStepCrossoverProjection;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;

public class QuadrupedStepAdjustmentController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrantDependentList<FixedFrameVector3DBasics> instantaneousStepAdjustments = new QuadrantDependentList<>();
   private final QuadrantDependentList<RateLimitedYoFrameVector> limitedInstantaneousStepAdjustments = new QuadrantDependentList<>();
   private final DoubleParameter maxStepAdjustmentRate = new DoubleParameter("maxStepAdjustmentRate", registry, 5.0);

   private final QuadrantDependentList<YoDouble> dcmStepAdjustmentMultipliers = new QuadrantDependentList<>();
   private final YoFrameVector3D dcmError = new YoFrameVector3D("dcmError", worldFrame, registry);

   private final DoubleParameter dcmStepAdjustmentGain = new DoubleParameter("dcmStepAdjustmentGain", registry, 1.0);
   private final DoubleParameter dcmErrorThresholdForStepAdjustment = new DoubleParameter("dcmErrorThresholdForStepAdjustment", registry, 0.0);

   private final QuadrupedControllerToolbox controllerToolbox;
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final LinearInvertedPendulumModel lipModel;

   private final RecyclingArrayList<QuadrupedStep> adjustedActiveSteps;

   private final YoDouble controllerTime;

   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();
   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedStepAdjustmentController(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.controllerTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      this.lipModel = controllerToolbox.getLinearInvertedPendulumModel();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getShortName();

         YoFrameVector3D instantaneousStepAdjustment = new YoFrameVector3D(prefix + "InstantaneousStepAdjustment", worldFrame, registry);
         RateLimitedYoFrameVector limitedInstantaneousStepAdjustment = new RateLimitedYoFrameVector(prefix + "LimitedInstantaneousStepAdjustment", "", registry, maxStepAdjustmentRate,
                                                                                controllerToolbox.getRuntimeEnvironment().getControlDT(),
                                                                                instantaneousStepAdjustment);

         YoDouble dcmStepAdjustmentMultiplier = new YoDouble(prefix + "DcmStepAdjustmentMultiplier", registry);
         instantaneousStepAdjustment.setToNaN();
         limitedInstantaneousStepAdjustment.setToNaN();
         dcmStepAdjustmentMultiplier.setToNaN();

         instantaneousStepAdjustments.put(robotQuadrant, instantaneousStepAdjustment);
         limitedInstantaneousStepAdjustments.put(robotQuadrant, limitedInstantaneousStepAdjustment);
         dcmStepAdjustmentMultipliers.put(robotQuadrant, dcmStepAdjustmentMultiplier);
      }

      adjustedActiveSteps = new RecyclingArrayList<>(10, QuadrupedStep.class, QuadrupedStep::new);
      adjustedActiveSteps.clear();

      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      crossoverProjection = new QuadrupedStepCrossoverProjection(referenceFrames.getBodyZUpFrame(), referenceFrames.getSoleFrames(), registry);

      parentRegistry.addChild(registry);
   }

   public void completedStep(RobotQuadrant robotQuadrant)
   {
      instantaneousStepAdjustments.get(robotQuadrant).setToNaN();
      limitedInstantaneousStepAdjustments.get(robotQuadrant).setToZero();
      dcmStepAdjustmentMultipliers.get(robotQuadrant).setToNaN();
   }

   public RecyclingArrayList<QuadrupedStep> computeStepAdjustment(ArrayList<YoQuadrupedTimedStep> activeSteps, FramePoint3DReadOnly desiredDCMPosition)
   {
      adjustedActiveSteps.clear();

      // compute step adjustment for ongoing steps (proportional to dcm tracking error)
      controllerToolbox.getDCMPositionEstimate(dcmPositionEstimate);
      dcmPositionEstimate.changeFrame(worldFrame);
      dcmPositionSetpoint.setIncludingFrame(desiredDCMPosition);
      dcmPositionSetpoint.changeFrame(worldFrame);


      dcmError.sub(dcmPositionSetpoint, dcmPositionEstimate);

      // adjust nominal step goal positions in foot state machine
      for (int i = 0; i < activeSteps.size(); i++)
      {
         YoQuadrupedTimedStep activeStep = activeSteps.get(i);
         QuadrupedStep adjustedStep = adjustedActiveSteps.add();
         adjustedStep.set(activeStep);

         RobotQuadrant robotQuadrant = activeStep.getRobotQuadrant();

         FixedFrameVector3DBasics instantaneousStepAdjustment = instantaneousStepAdjustments.get(robotQuadrant);
         RateLimitedYoFrameVector limitedInstantaneousStepAdjustment = limitedInstantaneousStepAdjustments.get(robotQuadrant);

         YoDouble dcmStepAdjustmentMultiplier = dcmStepAdjustmentMultipliers.get(robotQuadrant);

         if (dcmError.length() > dcmErrorThresholdForStepAdjustment.getValue() || instantaneousStepAdjustment.length() > 0.0)
         {
            double timeRemainingInStep = Math.max(activeStep.getTimeInterval().getEndTime() - controllerTime.getDoubleValue(), 0.0);
            dcmStepAdjustmentMultiplier.set(dcmStepAdjustmentGain.getValue() * Math.exp(timeRemainingInStep * lipModel.getNaturalFrequency()));

            instantaneousStepAdjustment.set(dcmError);
            instantaneousStepAdjustment.scale(-dcmStepAdjustmentMultiplier.getDoubleValue());
            instantaneousStepAdjustment.setZ(0);
         }
         else
         {
            instantaneousStepAdjustment.setToZero();
         }
         limitedInstantaneousStepAdjustment.update();

         activeStep.getGoalPosition(tempPoint);
         tempPoint.changeFrame(worldFrame);
         tempPoint.add(limitedInstantaneousStepAdjustment);
         crossoverProjection.project(tempPoint, robotQuadrant);
//         groundPlaneEstimator.projectZ(tempPoint);
         adjustedStep.setGoalPosition(tempPoint);
      }

      return adjustedActiveSteps;
   }

   public FrameVector3DReadOnly getStepAdjustment(RobotQuadrant robotQuadrant)
   {
      return limitedInstantaneousStepAdjustments.get(robotQuadrant);
   }
}
