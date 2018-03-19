package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
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

   private final DoubleParameter dcmPositionStepAdjustmentGainParameter = new DoubleParameter("dcmPositionStepAdjustmentGain", registry, 1.5);

   private final YoFrameVector instantaneousStepAdjustment = new YoFrameVector("instantaneousStepAdjustment", worldFrame, registry);
   private final YoFrameVector accumulatedStepAdjustment = new YoFrameVector("accumulatedStepAdjustment", worldFrame, registry);

   private final FramePoint3D dcmPositionSetpoint = new FramePoint3D();
   private final FramePoint3D dcmPositionEstimate = new FramePoint3D();

   private final YoFramePoint yoDesiredDCMPosition = new YoFramePoint("desiredDCMPosition", worldFrame, registry);
   private final YoFrameVector yoDesiredDCMVelocity = new YoFrameVector("desiredDCMVelocity", worldFrame, registry);
   private final YoFramePoint yoFinalDesiredDCM = new YoFramePoint("finalDesiredDCMPosition", worldFrame, registry);
   private final YoFramePoint yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector linearMomentumRateWeight = new YoFrameVector("linearMomentumRateWeight", worldFrame, registry);

   private final QuadrupedStepCrossoverProjection crossoverProjection;
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final RecyclingArrayList<QuadrupedStep> adjustedActiveSteps;

   private final QuadrantDependentList<FramePoint3D> currentSolePositions;
   private final FramePoint3D tempPoint = new FramePoint3D();

   private final FrameVector3D momentumRateForCommand = new FrameVector3D();
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();

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

      linearMomentumRateWeight.set(5.0, 5.0, 1.0);
      momentumRateCommand.setLinearWeights(linearMomentumRateWeight);
      momentumRateCommand.setSelectionMatrixForLinearControl();

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

      linearMomentumRateOfChangeToPack.changeFrame(worldFrame);
      momentumRateForCommand.setIncludingFrame(linearMomentumRateOfChangeToPack);
      momentumRateForCommand.subZ(controllerToolbox.getFullRobotModel().getTotalMass() * controllerToolbox.getRuntimeEnvironment().getGravity());

      momentumRateCommand.setLinearMomentumRate(momentumRateForCommand);
      momentumRateCommand.setLinearWeights(linearMomentumRateWeight);
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

   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      return momentumRateCommand;
   }
}
