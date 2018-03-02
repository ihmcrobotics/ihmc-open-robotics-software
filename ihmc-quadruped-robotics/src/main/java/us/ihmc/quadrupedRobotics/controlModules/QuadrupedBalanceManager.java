package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlanner;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Beige;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.Yellow;
import static us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType.BALL_WITH_CROSS;

public class QuadrupedBalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble robotTimestamp;

   private final LinearInvertedPendulumModel linearInvertedPendulumModel;
   private final QuadrupedMomentumRateOfChangeModule momentumRateOfChangeModule;

   private final DCMPlanner dcmPlanner;

   private final QuadrupedPostureInputProviderInterface postureProvider;

   private final DoubleParameter[] comPositionProportionalGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] comPositionDerivativeGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] comPositionIntegralGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] dcmPositionProportionalGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] dcmPositionDerivativeGainsParameter = new DoubleParameter[3];
   private final DoubleParameter[] dcmPositionIntegralGainsParameter = new DoubleParameter[3];

   private final double[] comPositionProportionalGains = new double[3];
   private final double[] comPositionDerivativeGains = new double[3];
   private final double[] comPositionIntegralGains = new double[3];
   private final double[] dcmPositionProportionalGains = new double[3];
   private final double[] dcmPositionDerivativeGains = new double[3];
   private final double[] dcmPositionIntegralGains = new double[3];

   private static final double defaultMinimumStepClearanceParameter = 0.075;
   private static final double defaultMaximumStepStrideParameter = 1.0;

   private final DoubleParameter comPositionMaxIntegralErrorParameter = new DoubleParameter("comPositionMaxIntegralError", registry, 0);
   private final DoubleParameter dcmPositionMaxIntegralErrorParameter = new DoubleParameter("dcmPositionMaxIntegralError", registry, 0);
   private final DoubleParameter vrpPositionRateLimitParameter = new DoubleParameter("vrpPositionRateLimit", registry, Double.MAX_VALUE);
   private final DoubleParameter comPositionGravityCompensationParameter = new DoubleParameter("comPositionGravityCompensation", registry, 1);
   private final DoubleParameter dcmPositionStepAdjustmentGainParameter = new DoubleParameter("dcmPositionStepAdjustmentGain", registry, 1.5);
   private final DoubleParameter minimumStepClearanceParameter = new DoubleParameter("minimumStepClearance", registry, defaultMinimumStepClearanceParameter);
   private final DoubleParameter maximumStepStrideParameter = new DoubleParameter("maximumStepStride", registry, defaultMaximumStepStrideParameter);
   private final DoubleParameter initialTransitionDurationParameter = new DoubleParameter("initialTransitionDuration", registry, 0.5);

   private final YoFrameVector instantaneousStepAdjustment = new YoFrameVector("instantaneousStepAdjustment", worldFrame, registry);
   private final YoFrameVector accumulatedStepAdjustment = new YoFrameVector("accumulatedStepAdjustment", worldFrame, registry);

   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();

   private final YoFramePoint yoDesiredDCMPosition = new YoFramePoint("desiredDCMPosition", worldFrame, registry);
   private final YoFrameVector yoDesiredDCMVelocity = new YoFrameVector("desiredDCMVelocity", worldFrame, registry);
   private final YoFramePoint yoFinalDesiredDCM = new YoFramePoint("finalDesiredDCMPosition", worldFrame, registry);
   private final YoFramePoint yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);

   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final RecyclingArrayList<QuadrupedStep> adjustedActiveSteps;

   private final QuadrantDependentList<FramePoint3D> currentSolePositions;
   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedBalanceManager(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                  YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.postureProvider = postureProvider;

      QuadrupedRuntimeEnvironment runtimeEnvironment = controllerToolbox.getRuntimeEnvironment();
      robotTimestamp = runtimeEnvironment.getRobotTimestamp();

      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();

      currentSolePositions = controllerToolbox.getTaskSpaceEstimates().getSolePositions();
      ReferenceFrame supportFrame = controllerToolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      dcmPlanner = new DCMPlanner(runtimeEnvironment.getGravity(), postureProvider.getComPositionInput().getZ(), robotTimestamp, supportFrame,
                                  currentSolePositions, registry, yoGraphicsListRegistry);

      linearInvertedPendulumModel = controllerToolbox.getLinearInvertedPendulumModel();
      momentumRateOfChangeModule = new QuadrupedMomentumRateOfChangeModule(controllerToolbox, postureProvider, registry, yoGraphicsListRegistry);

      crossoverProjection = new QuadrupedStepCrossoverProjection(controllerToolbox.getReferenceFrames().getBodyZUpFrame(), registry);

      for (int i = 0; i < 3; i++)
      {
         double comPositionProportionalGain = (i == 2) ? 5000.0 : 0.0;
         double comPositionDerivativeGain = (i == 2) ? 750.0 : 0.0;
         double dcmPositionProportionalGain = (i == 2) ? 0.0 : 1.0;

         comPositionProportionalGainsParameter[i] = new DoubleParameter("comPositionProportionalGain" + Axis.values[i], registry, comPositionProportionalGain);
         comPositionDerivativeGainsParameter[i] = new DoubleParameter("comPositionDerivativeGain" + Axis.values[i], registry, comPositionDerivativeGain);
         comPositionIntegralGainsParameter[i] = new DoubleParameter("comPositionIntegralGain" + Axis.values[i], registry, 0.0);
         dcmPositionProportionalGainsParameter[i] = new DoubleParameter("dcmPositionProportionalGain" + Axis.values[i], registry, dcmPositionProportionalGain);
         dcmPositionDerivativeGainsParameter[i] = new DoubleParameter("dcmPositionDerivativeGain" + Axis.values[i], registry, 0.0);
         dcmPositionIntegralGainsParameter[i] = new DoubleParameter("dcmPositionIntegralGain" + Axis.values[i], registry, 0.0);
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

      if (yoGraphicsListRegistry != null)
         setupGraphics(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupGraphics(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      String graphicsListName = getClass().getSimpleName();

      YoGraphicsList graphicsList = new YoGraphicsList(graphicsListName);
      ArtifactList artifactList = new ArtifactList(graphicsListName);

      YoGraphicPosition desiredDCMViz = new YoGraphicPosition("Desired DCM", yoDesiredDCMPosition, 0.01, Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition finalDesiredDCMViz = new YoGraphicPosition("Final Desired DCM", yoFinalDesiredDCM, 0.01, Beige(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition("Desired CMP", yoCmpPositionSetpoint, 0.012, YoAppearance.Purple(), BALL_WITH_CROSS);

      graphicsList.add(desiredDCMViz);
      graphicsList.add(finalDesiredDCMViz);
      graphicsList.add(yoCmpPositionSetpointViz);

      artifactList.add(desiredDCMViz.createArtifact());
      artifactList.add(finalDesiredDCMViz.createArtifact());
      artifactList.add(yoCmpPositionSetpointViz.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(graphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearStepSequence()
   {
      dcmPlanner.clearStepSequence();
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      dcmPlanner.addStepToSequence(step);
   }

   public void addStepsToSequence(List<? extends QuadrupedTimedStep> steps)
   {
      dcmPlanner.addStepsToSequence(steps);
   }

   private void initialize()
   {
      // update model
      linearInvertedPendulumModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update dcm estimate
      controllerToolbox.getDCMPositionEstimate(dcmPositionEstimate);

      yoDesiredDCMPosition.set(dcmPositionEstimate);
      yoDesiredDCMVelocity.setToZero();

      momentumRateOfChangeModule.initialize();

      // initialize timed contact sequence
      accumulatedStepAdjustment.setToZero();
   }

   public void initializeForStanding(QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      initialize();
      dcmPlanner.initializeForStanding();
   }

   public void initializeForStepping(QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      initialize();

      dcmPlanner.initializeForStepping(taskSpaceControllerSettings, dcmPositionEstimate);
   }


   public void compute(FrameVector3D linearMomentumRateOfChangeToPack, QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      // update model
      linearInvertedPendulumModel.setComHeight(postureProvider.getComPositionInput().getZ());

      // update dcm estimate
      controllerToolbox.getDCMPositionEstimate(dcmPositionEstimate);
      dcmPlanner.setCoMHeight(linearInvertedPendulumModel.getComHeight());

      // update desired horizontal com forces
      dcmPlanner.computeDcmSetpoints(taskSpaceControllerSettings, yoDesiredDCMPosition, yoDesiredDCMVelocity);
      dcmPlanner.getFinalDesiredDCM(yoFinalDesiredDCM);

      momentumRateOfChangeModule.compute(linearMomentumRateOfChangeToPack, yoVrpPositionSetpoint, yoCmpPositionSetpoint, dcmPositionEstimate,
                                         yoDesiredDCMPosition, yoDesiredDCMVelocity);
   }

   public void completedStep()
   {
      accumulatedStepAdjustment.add(instantaneousStepAdjustment);
      accumulatedStepAdjustment.setZ(0);
   }

   public RecyclingArrayList<QuadrupedStep> computeStepAdjustment(ArrayList<YoQuadrupedTimedStep> activeSteps)
   {
      adjustedActiveSteps.clear();
      if (robotTimestamp.getDoubleValue() > dcmPlanner.getFinalTime())
      {
         // compute step adjustment for ongoing steps (proportional to dcm tracking error)
         dcmPositionSetpoint.setIncludingFrame(yoDesiredDCMPosition);
         dcmPositionSetpoint.changeFrame(instantaneousStepAdjustment.getReferenceFrame());
         dcmPositionEstimate.changeFrame(instantaneousStepAdjustment.getReferenceFrame());

         instantaneousStepAdjustment.sub(dcmPositionEstimate, dcmPositionSetpoint);
         instantaneousStepAdjustment.scale(dcmPositionStepAdjustmentGainParameter.getValue());
         instantaneousStepAdjustment.setZ(0);

         // adjust nominal step goal positions in foot state machine
         for (int i = 0; i < activeSteps.size(); i++)
         {
            YoQuadrupedTimedStep activeStep = activeSteps.get(i);
            QuadrupedStep adjustedStep = adjustedActiveSteps.add();
            adjustedStep.set(activeStep);

            RobotQuadrant robotQuadrant = activeStep.getRobotQuadrant();
            activeStep.getGoalPosition(tempPoint);
            tempPoint.changeFrame(worldFrame);
            tempPoint.add(instantaneousStepAdjustment);
            crossoverProjection.project(tempPoint, currentSolePositions, robotQuadrant);
            groundPlaneEstimator.projectZ(tempPoint);
            adjustedStep.setGoalPosition(tempPoint);
         }
      }

      return adjustedActiveSteps;
   }

   public FrameVector3DReadOnly getAccumulatedStepAdjustment()
   {
      return accumulatedStepAdjustment;
   }
}
