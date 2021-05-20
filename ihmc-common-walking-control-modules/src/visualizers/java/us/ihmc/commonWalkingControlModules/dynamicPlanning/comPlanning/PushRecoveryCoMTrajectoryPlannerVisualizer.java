package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.PushRecoveryCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.PushRecoveryState;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;

public class PushRecoveryCoMTrajectoryPlannerVisualizer
{
   private static final double gravity = 9.81;
   //   private static final double nominalHeight = 0.75;
   private static final double nominalHeight = 9.81 / 9;

   private static final double finalTransferDuration = 1.0;
   private static final double stepLength = 0.5;
   private static final double stepWidth = 0.15;

   private static final double defaultSwingTime = 0.6;
   private static final double defaultTransferTime = 0.05;

   private static final double simDt = 1e-3;

   private double simDuration;

   private static final int BUFFER_SIZE = 160000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;
   private final YoDouble timeInPhase;

   private final PushRecoveryState state;
   private final PushRecoveryCoPTrajectoryGenerator copPlanner;
   private final CoMTrajectoryPlanner comPlanner;

   private final ConvexPolygon2DBasics defaultSupportPolygon = new ConvexPolygon2D();

   private final YoFrameConvexPolygon2D nextFootPolygon;

   private final YoBoolean replan;
   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFrameVector3D desiredGroundReactionForce;
   private final YoFramePoint3D desiredECMPPosition;

   private final YoDouble swingTime;
   private final YoDouble transferTime;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;

   private final YoDouble omega;

   public PushRecoveryCoMTrajectoryPlannerVisualizer()
   {
      YoRegistry registry = new YoRegistry("testJacobian");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

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

      swingTime = new YoDouble("swingTime", registry);
      transferTime = new YoDouble("transferTime", registry);

      dcmTrajectory = new BagOfBalls(50, 0.02, "dcmTrajectory", YoAppearance.Yellow(), registry, graphicsListRegistry);
      comTrajectory = new BagOfBalls(50, 0.02, "comTrajectory", YoAppearance.Black(), registry, graphicsListRegistry);
      vrpTrajectory = new BagOfBalls(50, 0.02, "vrpTrajectory", YoAppearance.Green(), registry, graphicsListRegistry);

      yoTime = new YoDouble("timeToCheck", registry);
      timeInPhase = new YoDouble("timeInPhase", registry);

      defaultSupportPolygon.addVertex(0.1, 0.05);
      defaultSupportPolygon.addVertex(0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, 0.05);
      defaultSupportPolygon.update();
      state = new PushRecoveryState(registry);

      swingTime.set(defaultSwingTime);
      transferTime.set(defaultTransferTime);

      copPlanner = new PushRecoveryCoPTrajectoryGenerator(defaultSupportPolygon, registry);
      copPlanner.registerState(state);

      comPlanner = new CoMTrajectoryPlanner(gravity, nominalHeight, registry);
      comPlanner.setCornerPointViewer(new CornerPointViewer(registry, graphicsListRegistry));
      comPlanner.setComContinuityCalculator(new CoMContinuousContinuityCalculator(gravity, omega, registry));

      FramePose3D leftStancePose = new FramePose3D();
      FramePose3D rightStancePose = new FramePose3D();
      leftStancePose.getPosition().setY(stepWidth);
      rightStancePose.getPosition().setY(-stepWidth);

      PoseReferenceFrame leftStepFrame = new PoseReferenceFrame("LeftStepFrame", worldFrame);
      PoseReferenceFrame rightStepFrame = new PoseReferenceFrame("RightStepFrame", worldFrame);
      leftStepFrame.setPoseAndUpdate(leftStancePose);
      rightStepFrame.setPoseAndUpdate(rightStancePose);

      FrameConvexPolygon2D leftSupport = new FrameConvexPolygon2D(leftStepFrame);
      FrameConvexPolygon2D rightSupport = new FrameConvexPolygon2D(rightStepFrame);
      leftSupport.set(defaultSupportPolygon);
      rightSupport.set(defaultSupportPolygon);

      state.initializeStance(RobotSide.LEFT, leftSupport, leftStepFrame);
      state.initializeStance(RobotSide.RIGHT, rightSupport, rightStepFrame);

      YoGraphicPosition dcmViz = new YoGraphicPosition("desiredDCM", desiredDCMPosition, 0.02, YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.02, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition vrpViz = new YoGraphicPosition("desiredVRP", desiredVRPPosition, 0.02, YoAppearance.Purple(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);

      YoGraphicVector forceViz = new YoGraphicVector("desiredGRF", desiredECMPPosition, desiredGroundReactionForce, 0.05, YoAppearance.Red());

      YoFrameConvexPolygon2D leftFootPolygon = new YoFrameConvexPolygon2D("leftFootPolygon", worldFrame, 4, registry);
      YoFrameConvexPolygon2D rightFootPolygon = new YoFrameConvexPolygon2D("rightFootPolygon", worldFrame, 4, registry);
      nextFootPolygon = new YoFrameConvexPolygon2D("nextFootPolygon", worldFrame, 4, registry);

      YoArtifactPolygon leftFootPolygonArtifact = new YoArtifactPolygon("leftFootArtifact", leftFootPolygon, Color.green, false);
      YoArtifactPolygon rightFootPolygonArtifact = new YoArtifactPolygon("rightFootArtifact", rightFootPolygon, Color.green, false);
      YoArtifactPolygon nextFootPolygonArtifact = new YoArtifactPolygon("nextFootArtifact", nextFootPolygon, Color.blue, false);

      graphicsListRegistry.registerArtifact("dcmPlanner", leftFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("dcmPlanner", rightFootPolygonArtifact);
      graphicsListRegistry.registerArtifact("dcmPlanner", nextFootPolygonArtifact);

      graphicsListRegistry.registerYoGraphic("dcmPlanner", forceViz);
      graphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      graphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

      leftFootPolygon.setMatchingFrame(state.getFootPolygonInSole(RobotSide.LEFT), false);
      rightFootPolygon.setMatchingFrame(state.getFootPolygonInSole(RobotSide.RIGHT), false);

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

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      desiredCoMPosition.setToZero();
      desiredCoMPosition.setZ(nominalHeight);
      desiredCoMPosition.setX(0.15);
      desiredCoMVelocity.setToZero();

      scs.startOnAThread();
      updateState();
      simulate();

      replan = new YoBoolean("replan", registry);
      replan.addListener(v -> {updateState(); simulate();});

      ThreadTools.sleepForever();
   }

   private final FramePoint2D icp = new FramePoint2D();
   private final PoseReferenceFrame newPoseFrame = new PoseReferenceFrame("newPose", worldFrame);

   private void updateState()
   {
      CapturePointTools.computeCapturePointPosition(new FramePoint2D(desiredCoMPosition), new FrameVector2D(desiredCoMVelocity), 1.0 / omega.getDoubleValue(), icp);

      state.setIcpAtStartOfState(icp);
      state.setFinalTransferDuration(1.0);
      state.addFootstep(RobotSide.LEFT, new FramePose3D(worldFrame, new Point3D(stepLength, stepWidth, 0.0), new Quaternion()), null);
      state.addFootstepTiming(swingTime.getDoubleValue(), transferTime.getDoubleValue());
      state.setFinalTransferDuration(finalTransferDuration);

      newPoseFrame.setPoseAndUpdate(state.getFootstep(0).getFootstepPose());

      FrameConvexPolygon2D newPolygon = new FrameConvexPolygon2D(newPoseFrame);
      newPolygon.set(defaultSupportPolygon);
      newPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      nextFootPolygon.set(newPolygon);
   }

   private void simulate()
   {
      copPlanner.compute(state);

      comPlanner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
      comPlanner.solveForTrajectory(copPlanner.getContactStateProviders());

      scs.tickAndUpdate();

      while (yoTime.getDoubleValue() < copPlanner.getContactStateProviders().get(copPlanner.getContactStateProviders().size() - 1).getTimeInterval().getEndTime())
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

   }

   public static void main(String[] args)
   {
      PushRecoveryCoMTrajectoryPlannerVisualizer visualizer = new PushRecoveryCoMTrajectoryPlannerVisualizer();
   }
}
