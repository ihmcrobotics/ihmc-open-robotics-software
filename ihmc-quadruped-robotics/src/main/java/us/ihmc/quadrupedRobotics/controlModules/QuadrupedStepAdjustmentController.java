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
import us.ihmc.robotics.lists.GenericTypeBuilder;
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
   private final DoubleParameter maxStepAdjustmentRate = new DoubleParameter("maxStepAdjustmentRate", registry, 1.0);

   private final YoFrameVector3D dcmError = new YoFrameVector3D("dcmError", worldFrame, registry);
   private final YoDouble dcmStepAdjustmentMultiplier = new YoDouble("dcmStepAdjustmentMultiplier", registry);

   private final DoubleParameter dcmStepAdjustmentGain = new DoubleParameter("dcmStepAdjustmentGain", registry, 1.5);
   private final DoubleParameter dcmErrorThresholdForStepAdjustment = new DoubleParameter("dcmErrorThresholdForStepAdjustment", registry, 0.02);

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
         YoFrameVector3D instantaneousStepAdjustment = new YoFrameVector3D(robotQuadrant.getShortName() + "InstantaneousStepAdjustment", worldFrame, registry);
         RateLimitedYoFrameVector limitedInstantaneousStepAdjustment = new RateLimitedYoFrameVector("limitedInstantaneousStepAdjustment", "", registry, maxStepAdjustmentRate,
                                                                                controllerToolbox.getRuntimeEnvironment().getControlDT(),
                                                                                instantaneousStepAdjustment);

         instantaneousStepAdjustments.put(robotQuadrant, instantaneousStepAdjustment);
         limitedInstantaneousStepAdjustments.put(robotQuadrant, limitedInstantaneousStepAdjustment);
      }

      adjustedActiveSteps = new RecyclingArrayList<>(10, new GenericTypeBuilder<QuadrupedStep>()
      {
         @Override
         public QuadrupedStep newInstance()
         {
            return new QuadrupedStep();
         }
      });
      adjustedActiveSteps.clear();

      QuadrupedReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      crossoverProjection = new QuadrupedStepCrossoverProjection(referenceFrames.getBodyZUpFrame(), referenceFrames.getSoleFrames(), registry);

      parentRegistry.addChild(registry);
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
         instantaneousStepAdjustment.set(dcmError);
         instantaneousStepAdjustment.scale(-dcmStepAdjustmentGain.getValue());
         instantaneousStepAdjustment.setZ(0);
         limitedInstantaneousStepAdjustments.get(robotQuadrant).update();

         activeStep.getGoalPosition(tempPoint);
         tempPoint.changeFrame(worldFrame);
         tempPoint.add(limitedInstantaneousStepAdjustments.get(robotQuadrant));
         crossoverProjection.project(tempPoint, robotQuadrant);
         groundPlaneEstimator.projectZ(tempPoint);
         adjustedStep.setGoalPosition(tempPoint);
      }

      return adjustedActiveSteps;
   }

   public FrameVector3DReadOnly getStepAdjustment(RobotQuadrant robotQuadrant)
   {
      return limitedInstantaneousStepAdjustments.get(robotQuadrant);
   }
}
