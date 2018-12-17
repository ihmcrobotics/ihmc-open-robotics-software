package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.ArrayList;
import java.util.List;

public class BipedCoMTrajectoryPlannerVisualizer
{
   private static final double stanceWidth = 0.5;
   private static final double gravity = 9.81;
   private static final double nominalHeight = 1.25;

   private static final double initialTransferTime = 1.0;
   private static final double swingDuration = 0.4;
   private static final double stanceDuration = 0.2;
   private static final double stepLength = 0.5;
   private static final int numberOfSteps = 5;

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
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;

   private final double simDuration;

   public BipedCoMTrajectoryPlannerVisualizer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      soleFrames.putAll(soleFramesForModifying);

      desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector3D("desiredCoMAcceleration", worldFrame, registry);
      desiredDCMPosition = new YoFramePoint3D("desiredDCMPosition", worldFrame, registry);
      desiredDCMVelocity = new YoFrameVector3D("desiredDCMVelocity", worldFrame, registry);
      desiredVRPPosition = new YoFramePoint3D("desiredVRPPosition", worldFrame, registry);

      YoDouble omega = new YoDouble("omega", registry);
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

      yoGraphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

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
      for (int i = 0; i < numberOfSteps; i++)
      {
         stepPose.getPosition().addX(stepLength);
         stepPose.getPosition().setY(currentSide.negateIfRightSide(stanceWidth / 2.0));

         BipedTimedStep step = new BipedTimedStep();

         step.getTimeInterval().setInterval(stepStartTime, stepStartTime + swingDuration);
         step.setRobotSide(currentSide);
         step.setGoalPose(stepPose);
         steps.add(step);

         currentSide = currentSide.getOppositeSide();
         stepStartTime += swingDuration + stanceDuration;
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

         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredVRPPosition);

         yoTime.add(simDt);
         updateFeetStates(yoTime.getDoubleValue());

         scs.tickAndUpdate();
      }
   }

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
   }

   public static void main(String[] args)
   {
      BipedCoMTrajectoryPlannerVisualizer visualizer = new BipedCoMTrajectoryPlannerVisualizer();
   }
}
