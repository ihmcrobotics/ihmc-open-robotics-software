package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryCalculator;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryCalculatorVisualizer;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.PushRecoveryCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.PushRecoveryState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.DefaultPushRecoveryControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class PushRecoveryCoMTrajectoryPlannerVisualizer
{
   private static final double gravity = 9.81;
   //   private static final double nominalHeight = 0.75;
   private static final double nominalHeight = 9.81 / 9;

   private static final double finalTransferDuration = 1.0;
   private static final double stepWidth = 0.15;

   private static final double defaultMinSwingTime = 0.4;
   private static final double defaultMaxSwingTime = 0.8;
   private static final double defaultTransferTime = 0.05;

   private static final double simDt = 0.005;

   private static final int BUFFER_SIZE = 160000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;
   private final YoDouble timeInPhase;

   private final PushRecoveryState state;
   private final PushRecoveryCoPTrajectoryGenerator copPlanner;
   private final CoMTrajectoryPlanner comPlanner;

   private final ConvexPolygon2DBasics defaultSupportPolygon = new ConvexPolygon2D();

   private final FrameConvexPolygon2DBasics leftSupport;
   private final FrameConvexPolygon2DBasics rightSupport;
   private final YoFrameConvexPolygon2D nextFootPolygon;
   private final YoFrameConvexPolygon2D nextNextFootPolygon;
   private final YoFrameConvexPolygon2D nextNextNextFootPolygon;

   private final List<YoFrameConvexPolygon2D> nextFootPolygons = new ArrayList<>();

   private final YoBoolean shouldReplan;
   private final YoBoolean replan;
   private final YoFramePoint3D initialCoMPosition;
   private final YoFrameVector3D initialCoMVelocity;
   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFrameVector3D desiredGroundReactionForce;
   private final YoFramePoint3D desiredECMPPosition;

   private final YoDouble minSwingTime;
   private final YoDouble maxSwingTime;
   private final YoDouble transferTime;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;

   private final MultiStepPushRecoveryCalculator recoveryStepCalculator;
   private final MultiStepPushRecoveryCalculatorVisualizer recoveryStepCalculatorVisualizer;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2,
                                                                                                  getClass(),
                                                                                                  ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final YoDouble omega;

   public PushRecoveryCoMTrajectoryPlannerVisualizer()
   {
      YoRegistry registry = new YoRegistry("testJacobian");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

      initialCoMPosition = new YoFramePoint3D("initialCoMPosition", worldFrame, registry);
      initialCoMVelocity = new YoFrameVector3D("initialCoMVelocity", worldFrame, registry);
      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);
      desiredGroundReactionForce = new YoFrameVector3D("desiredGroundReactionForce", worldFrame, registry);
      desiredECMPPosition = new YoFramePoint3D("desiredECMPPosition", worldFrame, registry);

      omega = new YoDouble("omega", registry);
      omega.set(Math.sqrt(gravity / nominalHeight));

      minSwingTime = new YoDouble("minSwingTime", registry);
      maxSwingTime = new YoDouble("maxSwingTime", registry);
      transferTime = new YoDouble("transferTime", registry);

      dcmTrajectory = new BagOfBalls(100, 0.01, "dcmTrajectory", YoAppearance.Yellow(), YoGraphicPosition.GraphicType.SOLID_BALL, registry, graphicsListRegistry);
      comTrajectory = new BagOfBalls(100, 0.01, "comTrajectory", YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL, registry, graphicsListRegistry);
      vrpTrajectory = new BagOfBalls(100, 0.01, "vrpTrajectory", YoAppearance.Green(), YoGraphicPosition.GraphicType.SOLID_BALL, registry, graphicsListRegistry);

      yoTime = new YoDouble("timeToCheck", registry);
      timeInPhase = new YoDouble("timeInPhase", registry);

      defaultSupportPolygon.addVertex(0.1, 0.05);
      defaultSupportPolygon.addVertex(0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, 0.05);
      defaultSupportPolygon.update();
      state = new PushRecoveryState(registry);

      minSwingTime.set(defaultMinSwingTime);
      maxSwingTime.set(defaultMaxSwingTime);
      transferTime.set(defaultTransferTime);

      copPlanner = new PushRecoveryCoPTrajectoryGenerator(defaultSupportPolygon, registry);
      copPlanner.registerState(state);

      comPlanner = new CoMTrajectoryPlanner(gravity, nominalHeight, registry);
//      comPlanner.setCornerPointViewer(new CornerPointViewer(registry, graphicsListRegistry));
      comPlanner.setComContinuityCalculator(new CoMContinuousContinuityCalculator(gravity, omega, registry));
      comPlanner.setMaintainInitialCoMVelocityContinuity(true);

      FramePose3D leftStancePose = new FramePose3D();
      FramePose3D rightStancePose = new FramePose3D();
      leftStancePose.getPosition().setY(stepWidth);
      rightStancePose.getPosition().setY(-stepWidth);

      PoseReferenceFrame leftStepFrame = new PoseReferenceFrame("LeftStepFrame", worldFrame);
      PoseReferenceFrame rightStepFrame = new PoseReferenceFrame("RightStepFrame", worldFrame);
      leftStepFrame.setPoseAndUpdate(leftStancePose);
      rightStepFrame.setPoseAndUpdate(rightStancePose);

      leftSupport = new FrameConvexPolygon2D(leftStepFrame);
      rightSupport = new FrameConvexPolygon2D(rightStepFrame);
      leftSupport.set(defaultSupportPolygon);
      rightSupport.set(defaultSupportPolygon);

      state.initializeStance(RobotSide.LEFT, leftSupport, leftStepFrame);
      state.initializeStance(RobotSide.RIGHT, rightSupport, rightStepFrame);

      YoGraphicPosition initialDcmViz = new YoGraphicPosition("initialDCM",
                                                              initialCoMPosition,
                                                              0.02,
                                                              YoAppearance.Blue(),
                                                              YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition dcmViz = new YoGraphicPosition("desiredDCM",
                                                       desiredDCMPosition,
                                                       0.02,
                                                       YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.02, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition vrpViz = new YoGraphicPosition("desiredVRP",
                                                       desiredVRPPosition,
                                                       0.02,
                                                       YoAppearance.Purple(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      YoGraphicVector forceViz = new YoGraphicVector("desiredGRF", desiredECMPPosition, desiredGroundReactionForce, 0.05, YoAppearance.Red());

      YoFrameConvexPolygon2D leftFootPolygon = new YoFrameConvexPolygon2D("leftFootPolygon", worldFrame, 4, registry);
      YoFrameConvexPolygon2D rightFootPolygon = new YoFrameConvexPolygon2D("rightFootPolygon", worldFrame, 4, registry);
      nextFootPolygon = new YoFrameConvexPolygon2D("nextFootPolygon", worldFrame, 4, registry);
      nextNextFootPolygon = new YoFrameConvexPolygon2D("nextNextFootPolygon", worldFrame, 4, registry);
      nextNextNextFootPolygon = new YoFrameConvexPolygon2D("nextNextNextFootPolygon", worldFrame, 4, registry);

      nextFootPolygons.add(nextFootPolygon);
      nextFootPolygons.add(nextNextFootPolygon);
      nextFootPolygons.add(nextNextNextFootPolygon);

      YoArtifactPolygon leftFootPolygonArtifact = new YoArtifactPolygon("leftFootArtifact", leftFootPolygon, Color.green, false);
      YoArtifactPolygon rightFootPolygonArtifact = new YoArtifactPolygon("rightFootArtifact", rightFootPolygon, Color.green, false);

      YoArtifactPolygon nextFootPolygonArtifact = new YoArtifactPolygon("nextFootArtifact", nextFootPolygon, Color.blue, false);
      YoArtifactPolygon nextNextFootPolygonArtifact = new YoArtifactPolygon("nextNextFootArtifact", nextNextFootPolygon, Color.blue, false);
      YoArtifactPolygon nextNextNextFootPolygonArtifact = new YoArtifactPolygon("nextNextNextFootArtifact", nextNextNextFootPolygon, Color.blue, false);

      graphicsListRegistry.registerArtifact("dcmPlanner", leftFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("dcmPlanner", rightFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("dcmPlanner", nextFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("dcmPlanner", nextNextFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("dcmPlanner", nextNextNextFootPolygonArtifact);

      graphicsListRegistry.registerYoGraphic("dcmPlanner", forceViz);
      graphicsListRegistry.registerArtifact("dcmPlanner", initialDcmViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

      leftFootPolygon.setMatchingFrame(state.getFootPolygonInSole(RobotSide.LEFT), false);
      rightFootPolygon.setMatchingFrame(state.getFootPolygonInSole(RobotSide.RIGHT), false);

      double footWidth = 0.1;
      double kinematicsStepRange = 1.0;
      recoveryStepCalculator = new MultiStepPushRecoveryCalculator(() -> kinematicsStepRange,
                                                                   () -> footWidth,
                                                                   new DefaultPushRecoveryControllerParameters(),
                                                                   new SideDependentList<>(leftStepFrame, rightStepFrame),
                                                                   defaultSupportPolygon);
      recoveryStepCalculatorVisualizer = new MultiStepPushRecoveryCalculatorVisualizer("", 3, registry, graphicsListRegistry);

      shouldReplan = new YoBoolean("shouldReplan", registry);
      replan = new YoBoolean("replan", registry);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.75);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);

      addButton("replan", 1.0);


      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      initialCoMPosition.setToZero();
      initialCoMPosition.setZ(nominalHeight);
      initialCoMPosition.setX(0.25);
      initialCoMPosition.setY(-0.05);
      initialCoMVelocity.setToZero();

      replan.set(true);
      executorService.scheduleAtFixedRate(() ->
                                          {
                                             if (replan.getBooleanValue())
                                             {
                                                replan.set(false, false);
                                                shouldReplan.set(true);

                                                updateState();
                                                simulate();
                                             }
                                          }, 0, 100, TimeUnit.MILLISECONDS);
      scs.startOnAThread();
   }

   private void addButton(String yoVariableName, double newValue)
   {
      YoRegistry registry = scs.getRootRegistry();
      final YoVariable var = registry.findVariable(yoVariableName);
      final JButton button = new JButton(yoVariableName);
      scs.addButton(button);
      button.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            var.setValueFromDouble(newValue);
         }
      });
   }

   private final FramePoint2D icp = new FramePoint2D();
   private final PoseReferenceFrame newPoseFrame = new PoseReferenceFrame("newPose", worldFrame);

   private void updateState()
   {
      CapturePointTools.computeCapturePointPosition(new FramePoint2D(initialCoMPosition),
                                                    new FrameVector2D(initialCoMVelocity),
                                                    1.0 / omega.getDoubleValue(),
                                                    icp);

      LogTools.info("Computing recovery steps");

      recoveryStepCalculator.computeRecoverySteps(RobotSide.LEFT,
                                                  transferTime.getDoubleValue(),
                                                  minSwingTime.getDoubleValue(),
                                                  maxSwingTime.getDoubleValue(),
                                                  icp,
                                                  omega.getDoubleValue(),
                                                  rightSupport);
      recoveryStepCalculatorVisualizer.visualize(recoveryStepCalculator);

      LogTools.info("Updating CoP state");

      state.clear();
      state.setIcpAtStartOfState(icp);
      state.setFinalTransferDuration(1.0);
      state.setFinalTransferDuration(finalTransferDuration);
      for (int i = 0; i < recoveryStepCalculator.getNumberOfRecoverySteps(); i++)
      {
         state.addFootstep(recoveryStepCalculator.getRecoveryStep(i));
         state.addFootstepTiming(recoveryStepCalculator.getRecoveryStepTiming(i));
      }
      //      state.addFootstep(RobotSide.LEFT, new FramePose3D(worldFrame, new Point3D(stepLength, stepWidth, 0.0), new Quaternion()), null);
      //      state.addFootstepTiming(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      int i = 0;
      for (; i < state.getNumberOfFootsteps(); i++)
      {
         newPoseFrame.setPoseAndUpdate(state.getFootstep(i).getFootstepPose());

         FrameConvexPolygon2D newPolygon = new FrameConvexPolygon2D(newPoseFrame);
         newPolygon.set(defaultSupportPolygon);
         newPolygon.changeFrameAndProjectToXYPlane(worldFrame);
         nextFootPolygons.get(i).set(newPolygon);
      }
      for (; i < nextFootPolygons.size(); i++)
         nextFootPolygons.get(i).setToNaN();
   }

   private void simulate()
   {
      timeInPhase.set(0.0);
      shouldReplan.set(false);
      LogTools.info("Solving for CoP plan");
      copPlanner.compute(state);

      LogTools.info("Solving for CoM plan");
      comPlanner.setInitialCenterOfMassState(initialCoMPosition, initialCoMVelocity);
      comPlanner.solveForTrajectory(copPlanner.getContactStateProviders());

      LogTools.info("Visualizing CoM plan");

      while (timeInPhase.getDoubleValue() < copPlanner.getContactStateProviders()
                                                      .get(copPlanner.getContactStateProviders().size() - 1)
                                                      .getTimeInterval()
                                                      .getEndTime())
      {
         //         planner.solveForTrajectory();
         comPlanner.compute(timeInPhase.getDoubleValue());

         desiredCoMPosition.set(comPlanner.getDesiredCoMPosition());
         desiredCoMVelocity.set(comPlanner.getDesiredCoMVelocity());
         desiredCoMAcceleration.set(comPlanner.getDesiredCoMAcceleration());
         desiredDCMPosition.set(comPlanner.getDesiredDCMPosition());
         desiredDCMVelocity.set(comPlanner.getDesiredDCMVelocity());
         desiredVRPPosition.set(comPlanner.getDesiredVRPPosition());

         desiredGroundReactionForce.set(desiredCoMAcceleration);
         desiredGroundReactionForce.addZ(gravity);

         desiredECMPPosition.set(desiredVRPPosition);
         desiredECMPPosition.subZ(gravity / (omega.getDoubleValue() * omega.getDoubleValue()));

         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredVRPPosition);

         yoTime.add(simDt);
         timeInPhase.add(simDt);

         scs.tickAndUpdate();
      }

      LogTools.info("Finished!");

   }

   public static void main(String[] args)
   {
      PushRecoveryCoMTrajectoryPlannerVisualizer visualizer = new PushRecoveryCoMTrajectoryPlannerVisualizer();
   }
}
