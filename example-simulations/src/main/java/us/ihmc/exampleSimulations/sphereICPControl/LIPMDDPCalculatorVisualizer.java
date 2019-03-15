package us.ihmc.exampleSimulations.sphereICPControl;

import static us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters.footLengthForControl;
import static us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters.footWidthForControl;
import static us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters.toeWidthForControl;
import static us.ihmc.exampleSimulations.sphereICPControl.SphereICPPlannerVisualizer.defaultLeftColor;
import static us.ihmc.exampleSimulations.sphereICPControl.SphereICPPlannerVisualizer.defaultRightColor;
import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.BasicCoPPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm.LIPMDDPCalculator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.trajectoryOptimization.DiscreteOptimizationTrajectory;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class LIPMDDPCalculatorVisualizer
{
   private static final int BUFFER_SIZE = 16000;
   private final double dt = 0.006;

   private static final double singleSupportDuration = 0.5; /// 0.7;
   private static final double doubleSupportDuration = 0.05; // 0.4; //0.25;

   private static final double nominalComHeight = 1.0;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();

   private final SideDependentList<YoFramePoseUsingYawPitchRoll> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private final YoVariableRegistry registry = new YoVariableRegistry("ICPViz");

   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextNextFootstep", "", worldFrame, 4, registry);

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final YoBoolean computeNextPass = new YoBoolean("computeNextPass", registry);
   private final YoInteger iterations = new YoInteger("iterations", registry);

   private final int simulatedTicksPerGraphicUpdate = 1;
   private final double trailingDuration = 4.0 * (singleSupportDuration + doubleSupportDuration);
   private final int numberOfBalls = (int) (trailingDuration / dt / simulatedTicksPerGraphicUpdate);

   private BipedSupportPolygons bipedSupportPolygons;
   private final BagOfBalls copTrack;
   private final BagOfBalls modifiedCopTrack;
   private final BagOfBalls comTrack;

   private final BasicCoPPlanner copPlanner;

   private static final boolean useSimple = false;
   //private final SimpleLIPMDDPCalculator ddp = new SimpleLIPMDDPCalculator(0.01, 1.0, 9.81);
   private final LIPMDDPCalculator ddp = new LIPMDDPCalculator(0.01, 150.0, 9.81);

   private final YoDouble lineSearchGain = new YoDouble("lineSearchGain", registry);
   private final YoInteger updatesPerRequest = new YoInteger("updatesPerRequest", registry);
   private final YoDouble trajectoryDT = new YoDouble("trajectoryDT", registry);

   public LIPMDDPCalculatorVisualizer()
   {
      copTrack = new BagOfBalls(numberOfBalls, 0.005, "CoP", YoAppearance.Purple(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry);
      modifiedCopTrack = new BagOfBalls(numberOfBalls, 0.005, "ModifiedCoP", YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry);
      comTrack = new BagOfBalls(numberOfBalls, 0.005, "CoM", YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL, registry, yoGraphicsListRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = new FramePose3D(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         currentFootPoses.put(robotSide, new YoFramePoseUsingYawPitchRoll(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      updatesPerRequest.set(1);
      trajectoryDT.set(0.01);
      trajectoryDT.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            ddp.setDeltaT(trajectoryDT.getDoubleValue());
         }
      });

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         soleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, contactableFoot.getSoleFrame(), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleFrames, registry, yoGraphicsListRegistry);

      FootstepTestHelper footstepTestHelper = new FootstepTestHelper(contactableFeet);
      List<FootstepTiming> timings = new ArrayList<>();
      List<Footstep> footsteps = footstepTestHelper.createFootsteps(0.25, 0.20, 20);
      for (int i = 0; i < footsteps.size(); i++)
         timings.add(new FootstepTiming(singleSupportDuration, doubleSupportDuration));

      copPlanner = new BasicCoPPlanner(contactableFeet, midFeetZUpFrame, registry);

      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false));

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<Point2D>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0));

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

      /*
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      */

      //computeNextPass.set(true);

      scs.startOnAThread();
      simulate(footsteps, timings);
      ThreadTools.sleepForever();
   }


   private void simulate(List<Footstep> footsteps, List<FootstepTiming> timings)
   {
      Footstep nextFootstep = footsteps.remove(0);
      FootstepTiming nextFootstepTiming = timings.remove(0);

      copPlanner.clearPlan();

      Footstep nextNextFootstep = footsteps.get(0);
      Footstep nextNextNextFootstep = footsteps.get(1);
      FootstepTiming nextNextFootstepTiming = timings.get(0);
      FootstepTiming nextNextNextFootstepTiming = timings.get(1);

      updateUpcomingFootstepsViz(nextFootstep, nextNextFootstep, nextNextNextFootstep);

      copPlanner.submitFootstep(nextFootstep, nextFootstepTiming);
      copPlanner.submitFootstep(nextNextFootstep, nextNextFootstepTiming);
      //copPlanner.submitFootstep(nextNextNextFootstep, nextNextNextFootstepTiming);
      copPlanner.initializeForTransfer(yoTime.getDoubleValue());

      double trajectoryTime = nextFootstepTiming.getStepTime() + nextNextFootstepTiming.getStepTime() + nextNextNextFootstepTiming.getStepTime();

      plotCoPPlan(trajectoryTime);

      DenseMatrix64F currentCoMState;
      if (useSimple)
      {
         currentCoMState = new DenseMatrix64F(4, 1);
      }
      else
      {
         currentCoMState = new DenseMatrix64F(6, 1);
         currentCoMState.set(2, 0, 1.0);
      }

      ddp.initialize(currentCoMState, copPlanner.getCoPTrajectory());
      plotCoMPlan();

      while(true)
      {
         if (computeNextPass.getBooleanValue())
         {
            computeNextPass.set(false);

            for (int i = 0; i < updatesPerRequest.getIntegerValue(); i++)
            {
               iterations.set(ddp.solve());
            }

            plotCoMPlan();
         }

         scs.tickAndUpdate();
         yoTime.add(dt);
      }
   }

   private int counter = 0;
   private final FramePoint3D desiredCoP = new FramePoint3D();
   private final FrameVector3D desiredCoPVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoPAcceleration = new FrameVector3D();

   private void plotCoPPlan(double trajectoryTime)
   {
      updateFootViz();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      double time = yoTime.getDoubleValue();

      while (time <= trajectoryTime)
      {
         copPlanner.compute(time);
         copPlanner.getDesiredCoPData(desiredCoP, desiredCoPVelocity, desiredCoPAcceleration);

         if (counter++ % simulatedTicksPerGraphicUpdate == 0)
         {
            copTrack.setBallLoop(desiredCoP);
         }
         time += dt;
      }
   }

   private final FramePoint3D tempPoint = new FramePoint3D();
   private void plotCoMPlan()
   {
      DiscreteOptimizationTrajectory trajectory = ddp.getOptimalTrajectory();

      comTrack.reset();

      for (int i = 0; i < trajectory.size(); i++)
      {
         DenseMatrix64F control = trajectory.getControl(i);
         DenseMatrix64F state = trajectory.getState(i);

         tempPoint.set(control.get(0), control.get(1), 0.0);
         modifiedCopTrack.setBallLoop(tempPoint);

         tempPoint.set(state.get(0), state.get(1), 1.0);
         comTrack.setBallLoop(tempPoint);
      }
   }



   private void updateFootViz()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D footPose = new FramePose3D(contactableFeet.get(robotSide).getSoleFrame());
         footPose.changeFrame(worldFrame);
         currentFootPoses.get(robotSide).set(footPose);
      }
   }

   private void updateUpcomingFootstepsViz(Footstep nextFootstep, Footstep nextNextFootstep, Footstep nextNextNextFootstep)
   {
      double polygonShrinkAmount = 0.005;
      updateFootstepViz(nextFootstep, yoNextFootstepPolygon, yoNextFootstepPose, polygonShrinkAmount);
      updateFootstepViz(nextNextFootstep, yoNextNextFootstepPolygon, yoNextNextFootstepPose, polygonShrinkAmount);
      updateFootstepViz(nextNextNextFootstep, yoNextNextNextFootstepPolygon, yoNextNextNextFootstepPose, polygonShrinkAmount);
   }

   private final FrameConvexPolygon2D footstepPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempFootstepPolygonForShrinking = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   private void updateFootstepViz(Footstep footstep, YoFrameConvexPolygon2D convexPolygon2d, YoFramePoseUsingYawPitchRoll framePose, double polygonShrinkAmount)
   {
      footstep.setPredictedContactPoints(contactableFeet.get(footstep.getRobotSide()).getContactPoints2d());
      tempFootstepPolygonForShrinking.setIncludingFrame(footstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(footstep.getPredictedContactPoints()));
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      convexPolygon2d.set(footstepPolygon);

      FramePose3D nextNextNextFootstepPose = new FramePose3D(footstep.getSoleReferenceFrame());
      framePose.setMatchingFrame(nextNextNextFootstepPose);

   }

   public static void main(String[] args)
   {
      new LIPMDDPCalculatorVisualizer();
   }
}
