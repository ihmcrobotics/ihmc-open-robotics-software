package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class BipedCoMTrajectoryPlannerVisualizer
{
   private static boolean visualize = true;

   private static final double stanceWidth = 0.5;
   private static final double gravity = 9.81;
   private static final double nominalHeight = 1.25;

   private static final double initialTransferTime = 1.0;
   private static final double swingDuration = 0.4;
   private static final double stanceDuration = 0.1;
   private static final double flightDuration = 0.1;

   private static final double startLength = 0.3;
   private static final double stepLength = 0.8;
   private static final double runLength = 1.5;
   private static final int numberOfWalkingSteps = 0;
   private static final int numberOfRunningSteps = 30;

   private static final double extraSimDuration = 0.5;

   private static final boolean includeFlight = true;

   private static final double footLengthForControl = 0.2;
   private static final double toeWidthForControl = 0.15;
   private static final double footWidthForControl = 0.15;

   private static final double simDt = 1e-3;

   private static final int BUFFER_SIZE = 160000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private final BipedCoMTrajectoryPlanner planner;

   private List<BipedTimedStep> steps;
   private final List<BipedTimedStep> stepsInProgress = new ArrayList<>();
   private final SideDependentList<TranslationMovingReferenceFrame> soleFramesForModifying = createSoleFrames();
   private final List<RobotSide> feetInContact = new ArrayList<>();

   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFrameVector3D desiredForce;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFramePoint3D desiredECMPPosition;
   private final YoFramePoint3D desiredCoPPosition;

   private final YoDouble omega;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;

   private final double simDuration;

   private final YoVariableRegistry registry = new YoVariableRegistry("test");

   private final YoFramePoseUsingYawPitchRoll leftFootPose = new YoFramePoseUsingYawPitchRoll("leftFootPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll rightFootPose = new YoFramePoseUsingYawPitchRoll("rightFootPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextFootstepPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextNextNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextNextNextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2D leftFoot = new YoFrameConvexPolygon2D("leftFoot", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D rightFoot = new YoFrameConvexPolygon2D("rightFoot", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D combinedFeet = new YoFrameConvexPolygon2D("combinedFeet", "", worldFrame, 8, registry);
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextFootstep", "", worldFrame, 4, registry);
   private final YoFrameConvexPolygon2D yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2D("nextNextNextFootstep", "", worldFrame, 4, registry);

   private final YoDouble springStiffness = new YoDouble("springStiffness", registry);
   private final YoDouble currentSpringLength = new YoDouble("currentSpringLength", registry);
   private final YoDouble restingSpringLength = new YoDouble("restingSpringLength", registry);
   private final YoDouble springDeflection = new YoDouble("springDeflection", registry);
   private final YoFrameVector3D springAcceleration = new YoFrameVector3D("springAcceleration", worldFrame, registry);
   private final YoFrameVector3D springDirection = new YoFrameVector3D("springDirection", worldFrame, registry);

   private final List<YoFramePoseUsingYawPitchRoll> nextFootstepPoses = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> nextFootstepPolygons = new ArrayList<>();

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   private final Graphics3DObject worldGraphics = new Graphics3DObject();

   private final SideDependentList<MovingReferenceFrame> soleFrames;

   public BipedCoMTrajectoryPlannerVisualizer(StepGetter stepGetter)
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      visualize &= !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      soleFrames = new SideDependentList<>();
      soleFrames.putAll(soleFramesForModifying);

      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredForce = new YoFrameVector3D("desiredForce", worldFrame, registry);
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);
      desiredECMPPosition = new YoFramePoint3D("desiredECMPPosition", worldFrame, registry);
      desiredCoPPosition = new YoFramePoint3D("desiredCoPPosition", worldFrame, registry);

      omega = new YoDouble("omega", registry);
      omega.set(Math.sqrt(gravity / nominalHeight));

      dcmTrajectory = new BagOfBalls(50, 0.02, "dcmTrajectory", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      comTrajectory = new BagOfBalls(50, 0.02, "comTrajectory", YoAppearance.Black(), registry, yoGraphicsListRegistry);
      vrpTrajectory = new BagOfBalls(50, 0.02, "vrpTrajectory", YoAppearance.Green(), registry, yoGraphicsListRegistry);

      yoTime = new YoDouble("time", registry);

      YoGraphicPosition dcmViz = new YoGraphicPosition("desiredDCM", desiredDCMPosition, 0.02, YoAppearance.Yellow(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition comViz = new YoGraphicPosition("desiredCoM", desiredCoMPosition, 0.02, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition vrpViz = new YoGraphicPosition("desiredVRP", desiredVRPPosition, 0.02, YoAppearance.Purple(),
                                                       YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS);
      YoGraphicVector forceVector = new YoGraphicVector("desiredForce", desiredECMPPosition, desiredForce, 0.1, YoAppearance.Red());

      yoGraphicsListRegistry.registerYoGraphic("dcmPlanner", forceVector);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("leftFoot", leftFoot, Color.green, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("rightFoot", rightFoot, Color.green, false));
      //      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("combinedFeet", combinedFeet, Color.red, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry
            .registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false));

      List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
      contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
      contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
      contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
      contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));

      contactPointsInSoleFrame.forEach(footPolygon::addVertex);
      footPolygon.update();

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      Graphics3DObject stanceFootGraphics = new Graphics3DObject();
      footstepGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, YoAppearance.Color(Color.blue));
      stanceFootGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, YoAppearance.Color(Color.green));

      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("leftFootPose", stanceFootGraphics, leftFootPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("rightFootPose", stanceFootGraphics, rightFootPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0));
      yoGraphicsListRegistry
            .registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0));

      nextFootstepPoses.add(yoNextFootstepPose);
      nextFootstepPoses.add(yoNextNextFootstepPose);
      nextFootstepPoses.add(yoNextNextNextFootstepPose);

      nextFootstepPolygons.add(yoNextFootstepPolygon);
      nextFootstepPolygons.add(yoNextNextFootstepPolygon);
      nextFootstepPolygons.add(yoNextNextNextFootstepPolygon);

      planner = new BipedCoMTrajectoryPlanner(soleFrames, omega, gravity, nominalHeight, registry, yoGraphicsListRegistry);
      steps = stepGetter.getSteps(soleFrames, worldGraphics);
      //      steps = createSteps(soleFrames);
      //      steps = createFancySteps(soleFrames);
      //      steps = createSkippingSteps(soleFrames);

      springStiffness.set(computeStiffness());
      //      restingSpringLength.set(computeSpringRestingLength(nominalHeight, springStiffness.getDoubleValue()));
      restingSpringLength.set(1.4);

      simDuration = steps.get(steps.size() - 1).getTimeInterval().getEndTime() + extraSimDuration;

      SimulationConstructionSetParameters scsParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      scsParameters.setDataBufferSize(BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      for (RobotSide robotSide : RobotSide.values)
         feetInContact.add(robotSide);

      scsParameters.setShowWindows(visualize);
      scsParameters.setCreateGUI(visualize);

      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(1.0);
      worldGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(worldGraphics);
      scs.setCameraFix(7.5, 0.0, 0.5);
      scs.setCameraPosition(-2.0, 9.5, 6.0);

      if (scsParameters.getCreateGUI())
      {
         SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
         simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
         simulationOverheadPlotterFactory.createOverheadPlotter();
      }

      scs.startOnAThread();
      simulate();
      ThreadTools.sleepForever();
   }

   private static double computeStiffness()
   {
      double stanceTime = swingDuration - 2.0 * flightDuration;
      double oscillationPeriod = 2.0 * stanceTime;
      double oscillationFrequency = 2.0 * Math.PI / oscillationPeriod;
      return MathTools.square(oscillationFrequency);
   }

   private static double computeSpringRestingLength(double nominalHeight, double stiffness)
   {
      double requiredCompression = gravity / stiffness;
      return nominalHeight + requiredCompression;
   }

   private static SideDependentList<TranslationMovingReferenceFrame> createSoleFrames()
   {
      SideDependentList<TranslationMovingReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         TranslationMovingReferenceFrame soleFrame = new TranslationMovingReferenceFrame(robotSide + "SoleFrame", worldFrame);
         Vector3D translation = new Vector3D();
         translation.setY(robotSide.negateIfRightSide(stanceWidth / 2.0));
         soleFrame.updateTranslation(translation);

         soleFrames.put(robotSide, soleFrame);
      }

      return soleFrames;
   }

   public SideDependentList<MovingReferenceFrame> getSoleFrames()
   {
      return soleFrames;
   }

   public Graphics3DObject getWorldGraphics()
   {
      return worldGraphics;
   }

   static List<BipedTimedStep> createSteps(SideDependentList<MovingReferenceFrame> soleFrames, Graphics3DObject worldGraphics)
   {
      List<BipedTimedStep> steps = new ArrayList<>();

      FramePose3D stepPose = new FramePose3D();
      stepPose.setToZero(soleFrames.get(RobotSide.LEFT));
      stepPose.changeFrame(worldFrame);

      double stepStartTime = initialTransferTime;
      RobotSide currentSide = RobotSide.LEFT;
      for (int i = 0; i < numberOfWalkingSteps; i++)
      {
         double alpha = i / (numberOfWalkingSteps - 1.0);
         double length = InterpolationTools.linearInterpolate(startLength, stepLength, alpha);
         stepPose.getPosition().addX(length);
         stepPose.getPosition().setY(currentSide.negateIfRightSide(stanceWidth / 2.0));

         BipedTimedStep step = new BipedTimedStep();

         step.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
         step.setRobotSide(currentSide);
         step.setGoalPose(stepPose);
         steps.add(step);

         currentSide = currentSide.getOppositeSide();
         stepStartTime += swingDuration + stanceDuration;
      }

      if (includeFlight)
      {
         for (int i = 0; i < numberOfRunningSteps; i++)
         {
            double alpha = i / (numberOfRunningSteps - 1.0);
            double length = InterpolationTools.linearInterpolate(stepLength, runLength, alpha);
            stepPose.getPosition().addX(length);
            stepPose.getPosition().setY(currentSide.negateIfRightSide(stanceWidth / 2.0));

            BipedTimedStep step = new BipedTimedStep();

            step.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
            step.setRobotSide(currentSide);
            step.setGoalPose(stepPose);
            steps.add(step);

            currentSide = currentSide.getOppositeSide();
            stepStartTime += swingDuration - flightDuration;
         }
      }

      return steps;
   }

   static List<BipedTimedStep> createSkippingSteps(SideDependentList<MovingReferenceFrame> soleFrames, Graphics3DObject worldGraphics)
   {
      List<BipedTimedStep> steps = new ArrayList<>();

      FramePose3D stepPose = new FramePose3D();
      stepPose.setToZero(soleFrames.get(RobotSide.LEFT));
      stepPose.changeFrame(worldFrame);

      double stepStartTime = initialTransferTime;
      double currentPosition = 0.0;
      double currentHeight = 0.0;

      BipedTimedStep step1 = new BipedTimedStep();
      step1.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step1.setRobotSide(RobotSide.LEFT);
      step1.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.5, stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step1);

      stepStartTime += swingDuration + stanceDuration;
      currentPosition += 0.3;

      BipedTimedStep step2 = new BipedTimedStep();
      step2.getTimeInterval().setInterval(stepStartTime, stepStartTime + 2.0 * swingDuration);
      step2.setRobotSide(RobotSide.RIGHT);
      step2.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.7, -stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step2);

      stepStartTime += 0.5 * (2.0 * swingDuration - flightDuration);
      //      currentPosition += 0.7;

      BipedTimedStep step3 = new BipedTimedStep();
      step3.getTimeInterval().setInterval(stepStartTime, stepStartTime + flightDuration);
      step3.setRobotSide(RobotSide.LEFT);
      step3.setGoalPose(new FramePoint3D(worldFrame, currentPosition, stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step3);

      return steps;
   }

   static List<BipedTimedStep> createFancySteps(SideDependentList<MovingReferenceFrame> soleFrames, Graphics3DObject worldGraphics)
   {
      List<BipedTimedStep> steps = new ArrayList<>();

      FramePose3D stepPose = new FramePose3D();
      stepPose.setToZero(soleFrames.get(RobotSide.LEFT));
      stepPose.changeFrame(worldFrame);

      double stepStartTime = initialTransferTime;
      double currentPosition = 0.0;
      double currentHeight = 0.0;

      worldGraphics.identity();
      worldGraphics.translate(8.25, -0.3, 0.0);
      worldGraphics.addCube(0.5, 0.6, 0.4, YoAppearance.Gray());

      worldGraphics.identity();
      worldGraphics.translate(8.75, 0.3, 0.0);
      worldGraphics.addCube(0.5, 0.6, 0.8, YoAppearance.Gray());

      worldGraphics.identity();
      worldGraphics.translate(9.25 + 3.0, 0.0, 0.0);
      worldGraphics.addCube(6.5, 1.5, 1.0, YoAppearance.Gray());

      worldGraphics.identity();
      worldGraphics.translate(11.45, 0.0, 1.0);
      worldGraphics.addCube(0.2, 1.5, 0.3, YoAppearance.Gray());

      BipedTimedStep step1 = new BipedTimedStep();
      step1.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step1.setRobotSide(RobotSide.LEFT);
      step1.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.3, stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step1);

      stepStartTime += swingDuration + stanceDuration;
      currentPosition += 0.3;

      BipedTimedStep step2 = new BipedTimedStep();
      step2.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step2.setRobotSide(RobotSide.RIGHT);
      step2.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.7, -stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step2);

      stepStartTime += swingDuration + stanceDuration;
      currentPosition += 0.7;

      BipedTimedStep step3 = new BipedTimedStep();
      step3.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step3.setRobotSide(RobotSide.LEFT);
      step3.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.0, stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step3);

      stepStartTime += swingDuration + stanceDuration;
      currentPosition += 1.0;

      BipedTimedStep step4 = new BipedTimedStep();
      step4.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step4.setRobotSide(RobotSide.RIGHT);
      step4.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.2, -stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step4);

      stepStartTime += swingDuration - flightDuration;
      currentPosition += 1.2;

      BipedTimedStep step5 = new BipedTimedStep();
      step5.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step5.setRobotSide(RobotSide.LEFT);
      step5.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.5, stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step5);

      stepStartTime += swingDuration - flightDuration;
      currentPosition += 1.5;

      BipedTimedStep step6 = new BipedTimedStep();
      step6.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step6.setRobotSide(RobotSide.RIGHT);
      step6.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.5, -stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step6);

      stepStartTime += swingDuration - flightDuration;
      currentPosition += 1.5;

      BipedTimedStep step7 = new BipedTimedStep();
      step7.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step7.setRobotSide(RobotSide.LEFT);
      step7.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.5, stanceWidth / 2.0, currentHeight), new FrameQuaternion());

      steps.add(step7);
      stepStartTime += swingDuration - flightDuration;
      currentPosition += 1.5;

      BipedTimedStep step8 = new BipedTimedStep();
      step8.getTimeInterval().setInterval(stepStartTime, stepStartTime + 2.0 * swingDuration);
      step8.setRobotSide(RobotSide.RIGHT);
      step8.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.5, -0.75 * stanceWidth, currentHeight + 0.4), new FrameQuaternion());

      steps.add(step8);
      stepStartTime += 2.0 * swingDuration - 2.0 * flightDuration;
      currentPosition += 0.5;
      currentHeight += 0.4;

      BipedTimedStep step9 = new BipedTimedStep();
      step9.getTimeInterval().setInterval(stepStartTime, stepStartTime + 2.0 * swingDuration);
      step9.setRobotSide(RobotSide.LEFT);
      step9.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.5, 0.75 * stanceWidth, currentHeight + 0.4), new FrameQuaternion());

      steps.add(step9);
      stepStartTime += 2.0 * swingDuration - 2.0 * flightDuration;
      currentPosition += 0.5;
      currentHeight += 0.4;

      BipedTimedStep step10 = new BipedTimedStep();
      step10.getTimeInterval().setInterval(stepStartTime, stepStartTime + 2.0 * swingDuration);
      step10.setRobotSide(RobotSide.RIGHT);
      step10.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.5, -0.5 * stanceWidth, currentHeight + 0.2), new FrameQuaternion());

      steps.add(step10);

      stepStartTime += 2.0 * swingDuration - flightDuration;
      currentPosition += 0.5;
      currentHeight += 0.2;

      BipedTimedStep step11 = new BipedTimedStep();
      step11.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step11.setRobotSide(RobotSide.LEFT);
      step11.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.75, 0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step11);

      stepStartTime += swingDuration - flightDuration; // slightly longer flight
      currentPosition += 0.75;

      /* double jump set up, where we align both feet */
      BipedTimedStep step12 = new BipedTimedStep();
      step12.getTimeInterval().setInterval(stepStartTime, stepStartTime + 1.25 * swingDuration);
      step12.setRobotSide(RobotSide.RIGHT);
      step12.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.0, -0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step12);

      stepStartTime += 0.1;

      BipedTimedStep step13 = new BipedTimedStep();
      step13.getTimeInterval().setInterval(stepStartTime + 0.5 * swingDuration, stepStartTime + 1.25 * swingDuration);
      step13.setRobotSide(RobotSide.LEFT);
      step13.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.0, 0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step13);

      stepStartTime += 1.25 * swingDuration + 2.0 * stanceDuration;
      currentPosition += 1.0;


      /* double jump flight */
      BipedTimedStep step14 = new BipedTimedStep();
      step14.getTimeInterval().setInterval(stepStartTime, stepStartTime + 1.15 * swingDuration);
      step14.setRobotSide(RobotSide.RIGHT);
      step14.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.0, -0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step14);

      BipedTimedStep step15 = new BipedTimedStep();
      step15.getTimeInterval().setInterval(stepStartTime, stepStartTime + 1.15 * swingDuration);
      step15.setRobotSide(RobotSide.LEFT);
      step15.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.0, 0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step15);

      stepStartTime += 1.15 * swingDuration + 2.0 * stanceDuration;
      currentPosition += 1.0;

      BipedTimedStep step16 = new BipedTimedStep();
      step16.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step16.setRobotSide(RobotSide.RIGHT);
      step16.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.5, -0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step16);

      stepStartTime += swingDuration + stanceDuration;
      currentPosition += 0.5;

      BipedTimedStep step17 = new BipedTimedStep();
      step17.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step17.setRobotSide(RobotSide.LEFT);
      step17.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.0, 0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step17);

      stepStartTime += swingDuration - 0.1;
      currentPosition += 1.0;

      BipedTimedStep step18 = new BipedTimedStep();
      step18.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step18.setRobotSide(RobotSide.RIGHT);
      step18.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 1.0, -0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step18);

      stepStartTime += swingDuration + stanceDuration;
      currentPosition += 1.0;

      BipedTimedStep step19 = new BipedTimedStep();
      step19.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step19.setRobotSide(RobotSide.LEFT);
      step19.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.7, 0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step19);

      stepStartTime += swingDuration + stanceDuration;

      BipedTimedStep step20 = new BipedTimedStep();
      step20.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
      step20.setRobotSide(RobotSide.RIGHT);
      step20.setGoalPose(new FramePoint3D(worldFrame, currentPosition + 0.7, -0.5 * stanceWidth, currentHeight), new FrameQuaternion());

      steps.add(step20);

      return steps;
   }

   private void simulate()
   {
      desiredCoMPosition.setZ(nominalHeight);
      desiredCoMVelocity.setToZero();
      planner.initialize();
      planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);

      while (simDuration > yoTime.getDoubleValue())
      {
         planner.clearStepSequence();
         steps.forEach(planner::addStepToSequence);
         planner.computeSetpoints(yoTime.getDoubleValue(), feetInContact);

         desiredCoMPosition.set(planner.getDesiredCoMPosition());
         desiredCoMVelocity.set(planner.getDesiredCoMVelocity());
         desiredCoMAcceleration.set(planner.getDesiredCoMAcceleration());
         desiredDCMPosition.set(planner.getDesiredDCMPosition());
         desiredDCMVelocity.set(planner.getDesiredDCMVelocity());
         desiredVRPPosition.set(planner.getDesiredVRPPosition());

         desiredECMPPosition.set(desiredVRPPosition);
         desiredECMPPosition.subZ(gravity / MathTools.square(omega.getDoubleValue()));

         desiredCoPPosition.set(desiredECMPPosition);
         desiredCoPPosition.setZ(0.0);

         desiredForce.set(desiredCoMAcceleration);
         desiredForce.addZ(gravity);

         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredVRPPosition);

         assertDCMDynamicsHold();
         assertCoMDynamicsHold();

         computeStiffnessThatBestMeetsAcceleration();

         yoTime.add(simDt);
         updateFeetStates(yoTime.getDoubleValue());

         scs.tickAndUpdate();
      }
   }

   private static final double epsilon = 1e-8;

   private void assertDCMDynamicsHold()
   {
      FramePoint3D constructedDCM = new FramePoint3D();
      constructedDCM.set(desiredCoMVelocity);
      constructedDCM.scale(1 / omega.getDoubleValue());
      constructedDCM.add(desiredCoMPosition);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(constructedDCM, desiredDCMPosition, epsilon);

      FrameVector3D constructedDCMVelocity = new FrameVector3D();
      constructedDCMVelocity.sub(constructedDCM, desiredVRPPosition);
      constructedDCMVelocity.scale(omega.getDoubleValue());

      EuclidCoreTestTools.assertVector3DGeometricallyEquals(constructedDCMVelocity, desiredDCMVelocity, epsilon);
   }

   private void assertCoMDynamicsHold()
   {
      FrameVector3D constructedCoMAcceleration = new FrameVector3D();
      constructedCoMAcceleration.sub(desiredCoMPosition, desiredVRPPosition);
      constructedCoMAcceleration.scale(MathTools.square(omega.getDoubleValue()));

      EuclidCoreTestTools.assertVector3DGeometricallyEquals(constructedCoMAcceleration, desiredCoMAcceleration, epsilon);
   }

   private void computeStiffnessThatBestMeetsAcceleration()
   {
      if (MathTools.epsilonEquals(desiredForce.length(), 0.0, 1e-1))
      {
         springDirection.setToZero();
         springAcceleration.setToZero();
         springAcceleration.setZ(-gravity);
         return;
      }

      springDirection.sub(desiredCoMPosition, desiredCoPPosition);
      currentSpringLength.set(springDirection.length());
      springDeflection.set(restingSpringLength.getDoubleValue() - currentSpringLength.getDoubleValue());

      springAcceleration.set(springDirection);
      springAcceleration.normalize();
      springAcceleration.scale(springStiffness.getDoubleValue() * springDeflection.getDoubleValue());
      springAcceleration.subZ(gravity);
   }

   private final PoseReferenceFrame stepPoseFrame = new PoseReferenceFrame("stepPoseFrame", worldFrame);

   private void updateFeetStates(double currentTime)
   {
      feetInContact.clear();
      for (RobotSide robotSide : RobotSide.values)
         feetInContact.add(robotSide);

      int stepNumber = 0;
      while (stepNumber < steps.size())
      {
         BipedTimedStep step = steps.get(stepNumber);
         if (currentTime > step.getTimeInterval().getEndTime() && steps.size() > 1)
         {
            steps.remove(stepNumber);
            stepsInProgress.remove(step);
            planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
         }
         else
         {
            stepNumber++;
         }

         if (currentTime > step.getTimeInterval().getStartTime() && !stepsInProgress.contains(step))
         {
            stepsInProgress.add(step);
            planner.setInitialCenterOfMassState(desiredCoMPosition, desiredCoMVelocity);
         }

      }

      for (int i = 0; i < steps.size(); i++)
      {
         BipedTimedStep step = steps.get(i);

         if (step.getTimeInterval().intervalContains(currentTime))
         {
            soleFramesForModifying.get(step.getRobotSide()).updateTranslation(step.getGoalPose().getPosition());
            feetInContact.remove(step.getRobotSide());
         }
      }

      int nextStepIndex = 0;

      int stepIndex = 0;
      while (stepIndex < steps.size() && nextStepIndex < nextFootstepPoses.size() && nextStepIndex < nextFootstepPolygons.size())
      {
         BipedTimedStep step = steps.get(stepIndex);
         nextFootstepPoses.get(nextStepIndex).set(step.getGoalPose());

         stepPoseFrame.setPoseAndUpdate(step.getGoalPose());
         FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
         tempPolygon.setReferenceFrame(stepPoseFrame);
         tempPolygon.set(footPolygon);
         tempPolygon.changeFrame(worldFrame);
         nextFootstepPolygons.get(nextStepIndex).set(tempPolygon);

         stepIndex++;
         nextStepIndex++;
      }
      while (nextStepIndex < nextFootstepPoses.size() && nextStepIndex < nextFootstepPolygons.size())
      {
         nextFootstepPoses.get(nextStepIndex).setToNaN();
         nextFootstepPolygons.get(nextStepIndex).setToNaN();
         nextStepIndex++;
      }

      leftFootPose.setToNaN();
      rightFootPose.setToNaN();
      leftFoot.clearAndUpdate();
      rightFoot.clearAndUpdate();
      combinedFeet.clear();
      if (feetInContact.contains(RobotSide.LEFT))
      {
         FramePose3D pose = new FramePose3D(soleFramesForModifying.get(RobotSide.LEFT));
         pose.changeFrame(worldFrame);
         stepPoseFrame.setPoseAndUpdate(pose);
         FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
         tempPolygon.setReferenceFrame(stepPoseFrame);
         tempPolygon.set(footPolygon);
         tempPolygon.changeFrame(worldFrame);

         leftFoot.set(tempPolygon);
         leftFootPose.setMatchingFrame(pose);
         combinedFeet.addVertices(leftFoot);
      }
      if (feetInContact.contains(RobotSide.RIGHT))
      {
         FramePose3D pose = new FramePose3D(soleFramesForModifying.get(RobotSide.RIGHT));
         pose.changeFrame(worldFrame);
         stepPoseFrame.setPoseAndUpdate(pose);
         FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
         tempPolygon.setReferenceFrame(stepPoseFrame);
         tempPolygon.set(footPolygon);
         tempPolygon.changeFrame(worldFrame);

         rightFoot.set(tempPolygon);
         rightFootPose.setMatchingFrame(pose);
         combinedFeet.addVertices(rightFoot);
      }
      combinedFeet.update();
   }

   interface StepGetter
   {
      List<BipedTimedStep> getSteps(SideDependentList<MovingReferenceFrame> soleFrames, Graphics3DObject graphics3DObject);
   }

   public static void main(String[] args)
   {
      BipedCoMTrajectoryPlannerVisualizer visualizer = new BipedCoMTrajectoryPlannerVisualizer(BipedCoMTrajectoryPlannerVisualizer::createFancySteps);
   }
}
