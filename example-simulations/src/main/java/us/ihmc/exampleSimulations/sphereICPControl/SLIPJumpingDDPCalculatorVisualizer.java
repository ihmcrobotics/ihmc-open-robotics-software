package us.ihmc.exampleSimulations.sphereICPControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMDDPCalculator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.ContinuousSLIPDynamics;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPJumpingDDPCalculator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.trajectoryOptimization.DiscreteOptimizationData;
import us.ihmc.trajectoryOptimization.DiscreteOptimizationTrajectory;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters.*;
import static us.ihmc.exampleSimulations.sphereICPControl.SphereICPPlannerVisualizer.defaultLeftColor;
import static us.ihmc.exampleSimulations.sphereICPControl.SphereICPPlannerVisualizer.defaultRightColor;
import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class SLIPJumpingDDPCalculatorVisualizer
{
   private static final int BUFFER_SIZE = 16000;
   private final double dt = 0.01;

   private static final double firstTransferDuration = 0.2;
   private static final double secondTransferDuration = 0.2;
   private static final double landingAngle = Math.toRadians(20);

   private static final double nominalComHeight = 1.0;
   private static final double length = 1.0;
   private static final double gravityZ = 9.81;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private final YoVariableRegistry registry = new YoVariableRegistry("ICPViz");

   private final YoFramePose yoNextFootstepPose = new YoFramePose("nextFootstepPose", worldFrame, registry);
   private final YoFramePose yoNextNextFootstepPose = new YoFramePose("nextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoNextFootstepPolygon = new YoFrameConvexPolygon2d("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2d yoNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextFootstep", "", worldFrame, 4, registry);

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final YoBoolean computeNextPass = new YoBoolean("computeNextPass", registry);
   private final YoInteger iterations = new YoInteger("iterations", registry);

   private final int simulatedTicksPerGraphicUpdate = 1;
   private final int numberOfBalls = (int) (2.0 / dt / simulatedTicksPerGraphicUpdate);

   private final BagOfBalls modifiedCopTrack;
   private final BagOfBalls comTrack;

   private final Footstep leftFoot;
   private final Footstep rightFoot;

   private final SLIPJumpingDDPCalculator ddp = new SLIPJumpingDDPCalculator(dt, 150.0, nominalComHeight, gravityZ);

   private final YoInteger updatesPerRequest = new YoInteger("updatesPerRequest", registry);
   private final YoDouble trajectoryDT = new YoDouble("trajectoryDT", registry);

   public SLIPJumpingDDPCalculatorVisualizer()
   {
      modifiedCopTrack = new BagOfBalls(numberOfBalls, 0.005, "ModifiedCoP", YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry);
      comTrack = new BagOfBalls(numberOfBalls, 0.005, "CoM", YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose startingPose = new FramePose(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         YoFramePose currentFootPose = new YoFramePose(sidePrefix + "FootPose", worldFrame, registry);

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPose, 1.0));
      }

      updatesPerRequest.set(1);
      trajectoryDT.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            ddp.setDeltaT(trajectoryDT.getDoubleValue());
         }
      });
      trajectoryDT.set(dt);

      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false));

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0));

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraPosition(-5.0, 5.0, 5.0);
      scs.setCameraFix(0.0, 0.0, 0.5);

      scs.startOnAThread();

      leftFoot = new Footstep(RobotSide.LEFT, new FramePose(new FramePoint3D(worldFrame, length, 0.1, 0.0), new FrameQuaternion()));
      rightFoot = new Footstep(RobotSide.RIGHT, new FramePose(new FramePoint3D(worldFrame, length, -0.1, 0.0), new FrameQuaternion()));

      simulate();
      ThreadTools.sleepForever();
   }


   private final FramePoint3D startPoint = new FramePoint3D();
   private final FramePoint3D endPoint = new FramePoint3D();
   private final FramePoint3D apexPoint = new FramePoint3D();

   private void simulate()
   {
      startPoint.set(0, 0.0, 0.0);
      endPoint.set(length, 0.0, 0.0);

      updateUpcomingFootstepsViz(leftFoot, rightFoot);

      DenseMatrix64F currentCoMState;
      currentCoMState = new DenseMatrix64F(SLIPState.stateVectorSize, 1);
      currentCoMState.set(2, 0, 1.0);

      double jumpLength = startPoint.distance(endPoint);
      double heightChange = endPoint.getZ() - startPoint.getZ();
      double flightDuration = Math.sqrt(2.0 * (heightChange + jumpLength * Math.tan(landingAngle)) / gravityZ);
      double apexHeight = 0.5 * Math.pow(jumpLength * Math.tan(landingAngle), 2.0) / (flightDuration * flightDuration * gravityZ);

      apexPoint.set(length / 2.0, 0.0, apexHeight + nominalComHeight);

      ddp.initialize(currentCoMState, startPoint, apexPoint, endPoint, firstTransferDuration, flightDuration, secondTransferDuration);
      plotCoMPlan();

      while(true)
      {
         if (computeNextPass.getBooleanValue())
         {
            computeNextPass.set(false);

            for (int i = 0; i < updatesPerRequest.getIntegerValue(); i++)
            {
               ddp.singleSolve();
            }

            plotCoMPlan();
         }

         scs.tickAndUpdate();
         yoTime.add(dt);
      }
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private void plotCoMPlan()
   {
      DiscreteOptimizationData trajectory = ddp.getOptimalSequence();

      comTrack.reset();

      for (int i = 0; i < trajectory.size(); i++)
      {
         DenseMatrix64F control = trajectory.getControl(i);
         DenseMatrix64F state = trajectory.getState(i);

         tempPoint.set(control.get(SLIPState.xF), control.get(SLIPState.yF), 0.0);
         modifiedCopTrack.setBallLoop(tempPoint);

         tempPoint.set(state.get(SLIPState.x), state.get(SLIPState.y), state.get(SLIPState.z));
         comTrack.setBallLoop(tempPoint);
      }
   }



   private void updateUpcomingFootstepsViz(Footstep nextFootstep, Footstep nextNextFootstep)
   {
      double polygonShrinkAmount = 0.005;
      updateFootstepViz(nextFootstep, yoNextFootstepPolygon, yoNextFootstepPose, polygonShrinkAmount);
      updateFootstepViz(nextNextFootstep, yoNextNextFootstepPolygon, yoNextNextFootstepPose, polygonShrinkAmount);
   }

   private final FrameConvexPolygon2d footstepPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempFootstepPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   private void updateFootstepViz(Footstep footstep, YoFrameConvexPolygon2d convexPolygon2d, YoFramePose framePose, double polygonShrinkAmount)
   {
      footstep.setPredictedContactPoints(contactableFeet.get(footstep.getRobotSide()).getContactPoints2d());
      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(footstep.getSoleReferenceFrame(), footstep.getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      convexPolygon2d.setFrameConvexPolygon2d(footstepPolygon);

      FramePose nextNextNextFootstepPose = new FramePose(footstep.getSoleReferenceFrame());
      framePose.setAndMatchFrame(nextNextNextFootstepPose);

   }

   public static void main(String[] args)
   {
      new SLIPJumpingDDPCalculatorVisualizer();
   }
}
