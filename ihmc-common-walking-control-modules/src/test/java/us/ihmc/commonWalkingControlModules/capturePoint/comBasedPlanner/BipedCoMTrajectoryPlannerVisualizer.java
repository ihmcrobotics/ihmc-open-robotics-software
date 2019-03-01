package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class BipedCoMTrajectoryPlannerVisualizer
{
   private static final double stanceWidth = 0.5;
   private static final double gravity = 9.81;
   private static final double nominalHeight = 1.25;

   private static final double initialTransferTime = 1.0;
   private static final double swingDuration = 0.4;
   private static final double stanceDuration = 0.1;
   private static final double flightDuration = 0.1;

   private static final double startLength = 0.3;
   private static final double stepLength = 0.4;
   private static final double runLength = 0.5;
   private static final int numberOfWalkingSteps = 0;
   private static final int numberOfRunningSteps = 5;

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

   private final List<YoFramePoseUsingYawPitchRoll> nextFootstepPoses = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> nextFootstepPolygons = new ArrayList<>();

   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();

   public BipedCoMTrajectoryPlannerVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      soleFrames.putAll(soleFramesForModifying);

      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredForce = new YoFrameVector3D("desiredForce", worldFrame, registry);
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);
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
      YoGraphicVector forceVector = new YoGraphicVector("desiredForce", desiredCoPPosition, desiredForce, 0.1, YoAppearance.Red());

      yoGraphicsListRegistry.registerYoGraphic("dcmPlanner", forceVector);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("leftFoot", leftFoot, Color.green, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("rightFoot", rightFoot, Color.green, false));
//      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("combinedFeet", combinedFeet, Color.red, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false));
      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false));

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
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0));

      nextFootstepPoses.add(yoNextFootstepPose);
      nextFootstepPoses.add(yoNextNextFootstepPose);
      nextFootstepPoses.add(yoNextNextNextFootstepPose);

      nextFootstepPolygons.add(yoNextFootstepPolygon);
      nextFootstepPolygons.add(yoNextNextFootstepPolygon);
      nextFootstepPolygons.add(yoNextNextNextFootstepPolygon);

      planner = new BipedCoMTrajectoryPlanner(soleFrames, omega, gravity, nominalHeight, registry, yoGraphicsListRegistry);
      steps = createSteps(soleFrames);

      simDuration = steps.get(steps.size() - 1).getTimeInterval().getEndTime() + 5.0;

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      for (RobotSide robotSide : RobotSide.values)
         feetInContact.add(robotSide);

      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.25);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      simulate();
      ThreadTools.sleepForever();
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

   private List<BipedTimedStep> createSteps(SideDependentList<MovingReferenceFrame> soleFrames)
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

         desiredCoPPosition.set(desiredVRPPosition);
         desiredCoPPosition.subZ(gravity / MathTools.square(omega.getDoubleValue()));

         desiredForce.set(desiredCoMAcceleration);
         desiredForce.addZ(gravity);


         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredVRPPosition);

         yoTime.add(simDt);
         updateFeetStates(yoTime.getDoubleValue());

         scs.tickAndUpdate();
      }
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

   public static void main(String[] args)
   {
      BipedCoMTrajectoryPlannerVisualizer visualizer = new BipedCoMTrajectoryPlannerVisualizer();
   }
}
