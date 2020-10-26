package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.*;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.OptimizedCoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingCoPTrajectoryGeneratorState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingControllerToolbox;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.StandingCoPTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;

public class JumpingBalanceManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BipedSupportPolygons bipedSupportPolygons;

   private final JumpingMomentumRateControlModuleInput jumpingMomentumRateControlModuleInput = new JumpingMomentumRateControlModuleInput();

   private final JumpingControllerToolbox controllerToolbox;

   private final YoFramePoint3D yoDesiredDCM = new YoFramePoint3D("desiredDCM", worldFrame, registry);
   private final YoFrameVector3D yoDesiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
   private final YoFramePoint3D yoDesiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
   private final YoFrameVector3D yoDesiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
   private final YoFramePoint3D yoPerfectVRP = new YoFramePoint3D("perfectVRP", worldFrame, registry);


   private final YoBoolean comPlannerDone = new YoBoolean("ICPPlannerDone", registry);
   private final ExecutionTimer plannerTimer = new ExecutionTimer("icpPlannerTimer", registry);

   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final YoDouble currentStateDuration = new YoDouble("CurrentStateDuration", registry);
   private final YoDouble totalStateDuration = new YoDouble("totalStateDuration", registry);
   private final YoDouble timeInSupportSequence = new YoDouble("TimeInSupportSequence", registry);
   private final JumpingCoPTrajectoryGeneratorState copTrajectoryState;

   private final StandingCoPTrajectoryGenerator copTrajectoryForStanding;
   private final JumpingCoPTrajectoryGenerator copTrajectoryForJumping;

   private final OptimizedCoMTrajectoryPlanner comTrajectoryPlanner;

   public JumpingBalanceManager(JumpingControllerToolbox controllerToolbox,
                                CoPTrajectoryParameters copTrajectoryParameters,
                                YoRegistry parentRegistry)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      this.controllerToolbox = controllerToolbox;

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();

      soleFrames = controllerToolbox.getReferenceFrames().getSoleFrames();
      registry.addChild(copTrajectoryParameters.getRegistry());
      comTrajectoryPlanner = new OptimizedCoMTrajectoryPlanner(controllerToolbox.getGravityZ(), controllerToolbox.getOmega0Provider(), registry);
      copTrajectoryState = new JumpingCoPTrajectoryGeneratorState(registry);
      copTrajectoryState.registerStateToSave(copTrajectoryParameters);

      copTrajectoryForStanding = new StandingCoPTrajectoryGenerator(copTrajectoryParameters, registry);
      copTrajectoryForStanding.registerState(copTrajectoryState);

      copTrajectoryForJumping = new JumpingCoPTrajectoryGenerator(copTrajectoryParameters, registry);
      copTrajectoryForJumping.registerState(copTrajectoryState);

      String graphicListName = getClass().getSimpleName();

      if (yoGraphicsListRegistry != null)
      {
         comTrajectoryPlanner.setCornerPointViewer(new CornerPointViewer(true, false, registry, yoGraphicsListRegistry));

         YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredDCM, 0.01, Yellow(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoGraphicPosition desiredCoMViz = new YoGraphicPosition("Desired CoM", yoDesiredCoMPosition, 0.01, Red(), GraphicType.SOLID_BALL);
         YoGraphicPosition perfectVRPViz = new YoGraphicPosition("Perfect VRP", yoPerfectVRP, 0.002, BlueViolet());

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCapturePointViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCoMViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, perfectVRPViz);
         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCapturePointViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, desiredCoMViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact(graphicListName, perfectVRPViz.createArtifact());
      }
      yoDesiredDCM.setToNaN();
      yoPerfectVRP.setToNaN();

      parentRegistry.addChild(registry);
   }

   public void clearICPPlan()
   {
      copTrajectoryState.clear();
   }

   public void setDesiredCoMHeight(double height)
   {
      comTrajectoryPlanner.setNominalCoMHeight(height);
   }

   public void compute()
   {
      yoDesiredDCM.set(comTrajectoryPlanner.getDesiredDCMPosition());
      yoDesiredDCMVelocity.set(comTrajectoryPlanner.getDesiredDCMVelocity());
      yoPerfectVRP.set(comTrajectoryPlanner.getDesiredECMPPosition());
      yoDesiredCoMPosition.set(comTrajectoryPlanner.getDesiredCoMPosition());
      yoDesiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());

      double omega0 = controllerToolbox.getOmega0();
      if (Double.isNaN(omega0))
         throw new RuntimeException("omega0 is NaN");

      CapturePointTools.computeCentroidalMomentumPivot(yoDesiredDCM, yoDesiredDCMVelocity, omega0, yoPerfectVRP);

      jumpingMomentumRateControlModuleInput.setOmega0(omega0);
      jumpingMomentumRateControlModuleInput.setTimeInState(timeInSupportSequence.getDoubleValue());
      jumpingMomentumRateControlModuleInput.setVrpTrajectories(comTrajectoryPlanner.getVRPTrajectories());
   }


   public void computeCoMPlanForStanding()
   {
      plannerTimer.startMeasurement();

      // update online to account for foot slip
      for (RobotSide robotSide : RobotSide.values)
      {
         if (controllerToolbox.getFootContactState(robotSide).inContact())
            copTrajectoryState.initializeStance(robotSide, bipedSupportPolygons.getFootPolygonsInSoleZUpFrame().get(robotSide), soleFrames.get(robotSide));
      }
      copTrajectoryForStanding.compute(copTrajectoryState);

      comTrajectoryPlanner.solveForTrajectory(copTrajectoryForStanding.getContactStateProviders());
      comTrajectoryPlanner.compute(timeInSupportSequence.getDoubleValue());

      // If this condition is false we are experiencing a late touchdown or a delayed liftoff. Do not advance the time in support sequence!
      timeInSupportSequence.add(controllerToolbox.getControlDT());

      comPlannerDone.set(timeInSupportSequence.getValue() >= currentStateDuration.getValue());

      plannerTimer.stopMeasurement();
   }

   public void computeCoMPlanForJumping()
   {
      plannerTimer.startMeasurement();

      // update online to account for foot slip
      for (RobotSide robotSide : RobotSide.values)
      {
         if (controllerToolbox.getFootContactState(robotSide).inContact())
            copTrajectoryState.initializeStance(robotSide, bipedSupportPolygons.getFootPolygonsInSoleZUpFrame().get(robotSide), soleFrames.get(robotSide));
      }
      copTrajectoryForJumping.compute(copTrajectoryState);

      comTrajectoryPlanner.solveForTrajectory(copTrajectoryForJumping.getContactStateProviders());
      comTrajectoryPlanner.compute(timeInSupportSequence.getDoubleValue());

      // If this condition is false we are experiencing a late touchdown or a delayed liftoff. Do not advance the time in support sequence!
      timeInSupportSequence.add(controllerToolbox.getControlDT());

      comPlannerDone.set(timeInSupportSequence.getValue() >= currentStateDuration.getValue());

      plannerTimer.stopMeasurement();
   }

   public FramePoint3DReadOnly getDesiredDCM()
   {
      return yoDesiredDCM;
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return yoDesiredDCMVelocity;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return yoDesiredCoMVelocity;
   }

   public void initialize()
   {
      yoDesiredDCM.set(controllerToolbox.getCapturePoint());
      yoDesiredCoMPosition.setFromReferenceFrame(controllerToolbox.getCenterOfMassFrame());
      yoDesiredCoMPosition.setZ(comTrajectoryPlanner.getNominalCoMHeight());
      yoDesiredCoMVelocity.setToZero();

      yoPerfectVRP.set(bipedSupportPolygons.getSupportPolygonInWorld().getCentroid());
      copTrajectoryState.setInitialCoP(bipedSupportPolygons.getSupportPolygonInWorld().getCentroid());
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);
      timeInSupportSequence.set(0.0);
      currentStateDuration.set(Double.NaN);
      totalStateDuration.set(Double.NaN);

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(false);
   }

   public void initializeCoMPlanForStanding()
   {
      copTrajectoryState.setInitialCoP(yoPerfectVRP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(Double.POSITIVE_INFINITY);
      totalStateDuration.set(Double.POSITIVE_INFINITY);

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);

      comPlannerDone.set(false);
   }

   public void initializeCoMPlanForTransferToStanding()
   {
      copTrajectoryState.setInitialCoP(yoPerfectVRP);
      copTrajectoryState.initializeStance(bipedSupportPolygons.getFootPolygonsInSoleZUpFrame(), soleFrames);
      comTrajectoryPlanner.setInitialCenterOfMassState(yoDesiredCoMPosition, yoDesiredCoMVelocity);

      timeInSupportSequence.set(0.0);
      currentStateDuration.set(copTrajectoryState.getFinalTransferDuration());
      totalStateDuration.set(copTrajectoryState.getFinalTransferDuration());

      comTrajectoryPlanner.setMaintainInitialCoMVelocityContinuity(true);

      comPlannerDone.set(false);
   }


   public boolean isCoMPlanDone()
   {
      return comPlannerDone.getValue();
   }

   public void setFinalTransferTime(double finalTransferDuration)
   {
      copTrajectoryState.setFinalTransferDuration(finalTransferDuration);
   }

   public FramePoint3DReadOnly getCapturePoint()
   {
      return controllerToolbox.getCapturePoint();
   }

   public JumpingMomentumRateControlModuleInput getJumpingMomentumRateControlModuleInput()
   {
      return jumpingMomentumRateControlModuleInput;
   }
}
