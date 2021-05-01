package us.ihmc.quadrupedRobotics.planning.comPlanning;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.TranslationMovingReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedCoMTrajectoryPlannerVisualizer
{
   private static final double stanceLength = 1.0;
   private static final double stanceWidth = 0.5;
   private static final double gravity = 9.81;
   private static final double nominalHeight = 0.5;

   private static final double finalExtraTime = 1.5;

   private static final double initialTransferTime = 1.0;
   private static final double stepDuration = 0.4;
   private static final double stanceDuration = 0.2;
   private static final double flightTime = 0.1;
   private static final double stepLength = 0.5;
   private static final int numberOfSteps = 6;

   private final TDoubleArrayList changeOfStateEvents = new TDoubleArrayList();

   private static final boolean doTrot = true;
   private static final boolean planForFlight = true;

   private static final double mass = 1.0;
   private static final double simDt = 0.01;

   private static final int BUFFER_SIZE = 160000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;
   private final YoDouble omega;

   private final QuadrupedCoMTrajectoryPlanner planner;

   private List<QuadrupedTimedStep> steps;
   private final QuadrantDependentList<TranslationMovingReferenceFrame> soleFramesForModifying = createSoleFrames();
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final YoFrameConvexPolygon2D supportPolygonViz;


   private final YoFramePoint3D desiredCoMPosition;
   private final YoFrameVector3D desiredCoMVelocity;
   private final YoFrameVector3D desiredCoMAcceleration;
   private final YoFrameVector3D desiredForce;
   private final YoFramePoint3D desiredDCMPosition;
   private final YoFrameVector3D desiredDCMVelocity;
   private final YoFramePoint3D desiredVRPPosition;
   private final YoFramePoint3D desiredCoPPosition;

   private final BagOfBalls dcmTrajectory;
   private final BagOfBalls comTrajectory;
   private final BagOfBalls vrpTrajectory;

   private final double simDuration;

   public QuadrupedCoMTrajectoryPlannerVisualizer(StepGetter stepGetter)
   {
      YoRegistry registry = new YoRegistry("test");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      QuadrantDependentList<MovingReferenceFrame> soleFrames = new QuadrantDependentList<>();
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

      YoGraphicVector forceVector = new YoGraphicVector("desiredForce", desiredCoPPosition, desiredForce, 0.05, YoAppearance.Red());

      yoGraphicsListRegistry.registerYoGraphic("dcmPlanner", forceVector);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", dcmViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", comViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", vrpViz.createArtifact());

      planner = new QuadrupedCoMTrajectoryPlanner(new TestDCMPlannerParameters(), soleFrames, gravity, nominalHeight, registry, yoGraphicsListRegistry);
      Pair<List<QuadrupedTimedStep>, TDoubleArrayList> stepPair = stepGetter.getSteps(soleFrames);

      steps = stepPair.getLeft();
      changeOfStateEvents.addAll(stepPair.getRight());

      supportPolygonViz = new YoFrameConvexPolygon2D("combinedPolygon", "", worldFrame, 4 ,
                                                     registry);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", new YoArtifactPolygon("Combined Polygon", supportPolygonViz, Color.pink, false));

      simDuration = steps.get(steps.size() - 1).getTimeInterval().getEndTime() + finalExtraTime;

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         feetInContact.add(robotQuadrant);

      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setDT(simDt, 1);
      scs.addYoRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.5);
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

   private static QuadrantDependentList<TranslationMovingReferenceFrame> createSoleFrames()
   {
      QuadrantDependentList<TranslationMovingReferenceFrame> soleFrames = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         TranslationMovingReferenceFrame soleFrame = new TranslationMovingReferenceFrame(robotQuadrant + "SoleFrame", worldFrame);
         Vector3D translation = new Vector3D();
         translation.setX(robotQuadrant.getEnd().negateIfHindEnd(stanceLength / 2.0));
         translation.setY(robotQuadrant.getSide().negateIfRightSide(stanceWidth / 2.0));
         soleFrame.updateTranslation(translation);

         soleFrames.put(robotQuadrant, soleFrame);
      }

      return soleFrames;
   }

   static Pair<List<QuadrupedTimedStep>, TDoubleArrayList> createTrotSteps(QuadrantDependentList<MovingReferenceFrame> soleFrames)
   {
      TDoubleArrayList changeOfStateEvents = new TDoubleArrayList();
      int stepInterval = 0;
      List<QuadrupedTimedStep> steps = new ArrayList<>();

      FramePoint3D frontLeftStep = new FramePoint3D();
      FramePoint3D frontRightStep = new FramePoint3D();
      FramePoint3D hindLeftStep = new FramePoint3D();
      FramePoint3D hindRightStep = new FramePoint3D();

      frontLeftStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_LEFT));
      frontRightStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_RIGHT));
      hindLeftStep.setToZero(soleFrames.get(RobotQuadrant.HIND_LEFT));
      hindRightStep.setToZero(soleFrames.get(RobotQuadrant.HIND_RIGHT));

      frontLeftStep.changeFrame(worldFrame);
      frontRightStep.changeFrame(worldFrame);
      hindLeftStep.changeFrame(worldFrame);
      hindRightStep.changeFrame(worldFrame);

      double currentTime = initialTransferTime;
      for (int i = 0; i < numberOfSteps; i++)
      {
         frontLeftStep.addX(stepLength);
         frontRightStep.addX(stepLength);
         hindLeftStep.addX(stepLength);
         hindRightStep.addX(stepLength);

         QuadrupedTimedStep step1 = new QuadrupedTimedStep();
         QuadrupedTimedStep step2 = new QuadrupedTimedStep();

         double stepStart = currentTime;
         double stepEnd = stepStart + stepDuration;

         step1.getTimeInterval().setInterval(stepStart, stepEnd);
         step2.getTimeInterval().setInterval(stepStart, stepEnd);

         changeOfStateEvents.add(stepStart);
         changeOfStateEvents.add(stepEnd);

         if (stepInterval == 0)
         {
            step1.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
            step1.setGoalPosition(frontLeftStep);
            step2.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
            step2.setGoalPosition(hindRightStep);

            stepInterval = 1;
         }
         else
         {
            step1.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
            step1.setGoalPosition(frontRightStep);
            step2.setRobotQuadrant(RobotQuadrant.HIND_LEFT);
            step2.setGoalPosition(hindLeftStep);

            stepInterval = 0;
         }

         steps.add(step1);
         steps.add(step2);

         currentTime += stepDuration + stanceDuration;
      }

      changeOfStateEvents.sort();
      int index = 0;
      while (index < changeOfStateEvents.size() - 1)
      {
         if (MathTools.epsilonEquals(changeOfStateEvents.get(index), changeOfStateEvents.get(index + 1), 1e-4))
            changeOfStateEvents.removeAt(index + 1);
         else
            index++;
      }

      return new ImmutablePair<>(steps, changeOfStateEvents);
   }

   static Pair<List<QuadrupedTimedStep>, TDoubleArrayList> createFlyingTrotSteps(QuadrantDependentList<MovingReferenceFrame> soleFrames)
   {
      TDoubleArrayList changeOfStateEvents = new TDoubleArrayList();

      int stepInterval = 0;
      List<QuadrupedTimedStep> steps = new ArrayList<>();

      FramePoint3D frontLeftStep = new FramePoint3D();
      FramePoint3D frontRightStep = new FramePoint3D();
      FramePoint3D hindLeftStep = new FramePoint3D();
      FramePoint3D hindRightStep = new FramePoint3D();

      frontLeftStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_LEFT));
      frontRightStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_RIGHT));
      hindLeftStep.setToZero(soleFrames.get(RobotQuadrant.HIND_LEFT));
      hindRightStep.setToZero(soleFrames.get(RobotQuadrant.HIND_RIGHT));

      frontLeftStep.changeFrame(worldFrame);
      frontRightStep.changeFrame(worldFrame);
      hindLeftStep.changeFrame(worldFrame);
      hindRightStep.changeFrame(worldFrame);

      double currentTime = initialTransferTime;
      for (int i = 0; i < numberOfSteps; i++)
      {
         frontLeftStep.addX(stepLength);
         frontRightStep.addX(stepLength);
         hindLeftStep.addX(stepLength);
         hindRightStep.addX(stepLength);

         QuadrupedTimedStep step1 = new QuadrupedTimedStep();
         QuadrupedTimedStep step2 = new QuadrupedTimedStep();

         double stepStart = currentTime;
         double stepEnd = stepStart + stepDuration;

         step1.getTimeInterval().setInterval(stepStart, stepEnd);
         step2.getTimeInterval().setInterval(stepStart, stepEnd);

         changeOfStateEvents.add(stepStart);
         changeOfStateEvents.add(stepEnd);

         if (stepInterval == 0)
         {
            step1.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
            step1.setGoalPosition(frontLeftStep);
            step2.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
            step2.setGoalPosition(hindRightStep);

            stepInterval = 1;
         }
         else
         {
            step1.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
            step1.setGoalPosition(frontRightStep);
            step2.setRobotQuadrant(RobotQuadrant.HIND_LEFT);
            step2.setGoalPosition(hindLeftStep);

            stepInterval = 0;
         }

         steps.add(step1);
         steps.add(step2);

         currentTime += stepDuration - flightTime;
      }

      changeOfStateEvents.sort();
      int index = 0;
      while (index < changeOfStateEvents.size() - 1)
      {
         if (MathTools.epsilonEquals(changeOfStateEvents.get(index), changeOfStateEvents.get(index + 1), 1e-4))
            changeOfStateEvents.removeAt(index + 1);
         else
            index++;
      }

      return new ImmutablePair<>(steps, changeOfStateEvents);
   }



   static Pair<List<QuadrupedTimedStep>, TDoubleArrayList> createAmbleSteps(QuadrantDependentList<MovingReferenceFrame> soleFrames)
   {
      TDoubleArrayList changeOfStateEvents = new TDoubleArrayList();
      int stepInterval = 0;
      List<QuadrupedTimedStep> steps = new ArrayList<>();

      FramePoint3D frontLeftStep = new FramePoint3D();
      FramePoint3D frontRightStep = new FramePoint3D();
      FramePoint3D hindLeftStep = new FramePoint3D();
      FramePoint3D hindRightStep = new FramePoint3D();

      frontLeftStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_LEFT));
      frontRightStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_RIGHT));
      hindLeftStep.setToZero(soleFrames.get(RobotQuadrant.HIND_LEFT));
      hindRightStep.setToZero(soleFrames.get(RobotQuadrant.HIND_RIGHT));

      frontLeftStep.changeFrame(worldFrame);
      frontRightStep.changeFrame(worldFrame);
      hindLeftStep.changeFrame(worldFrame);
      hindRightStep.changeFrame(worldFrame);

      double currentTime = initialTransferTime;
      for (int i = 0; i < numberOfSteps; i++)
      {
         frontLeftStep.addX(stepLength);
         frontRightStep.addX(stepLength);
         hindLeftStep.addX(stepLength);
         hindRightStep.addX(stepLength);

         QuadrupedTimedStep step1 = new QuadrupedTimedStep();
         QuadrupedTimedStep step2 = new QuadrupedTimedStep();

         double step1Start = currentTime;
         double step1End = step1Start + stepDuration;
         double step2Start = currentTime + 0.5 * stepDuration;
         double step2End = step2Start + stepDuration;

         step1.getTimeInterval().setInterval(step1Start, step1End);
         step2.getTimeInterval().setInterval(step2Start, step2End);

         changeOfStateEvents.add(step1Start);
         changeOfStateEvents.add(step1End);
         changeOfStateEvents.add(step2Start);
         changeOfStateEvents.add(step2End);

         if (stepInterval == 0)
         {
            step1.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
            step1.setGoalPosition(frontLeftStep);
            step2.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
            step2.setGoalPosition(hindRightStep);

            stepInterval = 1;
         }
         else
         {
            step1.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
            step1.setGoalPosition(frontRightStep);
            step2.setRobotQuadrant(RobotQuadrant.HIND_LEFT);
            step2.setGoalPosition(hindLeftStep);

            stepInterval = 0;
         }

         steps.add(step1);
         steps.add(step2);

         currentTime += stepDuration + stanceDuration;
      }

      changeOfStateEvents.sort();
      int index = 0;
      while (index < changeOfStateEvents.size() - 1)
      {
         if (MathTools.epsilonEquals(changeOfStateEvents.get(index), changeOfStateEvents.get(index + 1), 1e-4))
            changeOfStateEvents.removeAt(index + 1);
         else
            index++;
      }

      return new ImmutablePair<>(steps, changeOfStateEvents);
   }

   private void simulate()
   {
      desiredCoMPosition.setZ(nominalHeight);
      desiredCoMVelocity.setToZero();
      desiredCoPPosition.setToZero();
      planner.initialize();
      planner.setInitialState(yoTime.getDoubleValue(), desiredCoMPosition, desiredCoMVelocity, desiredCoPPosition);

      while (simDuration > yoTime.getDoubleValue())
      {
         updateFeetStates(yoTime.getDoubleValue());
         //         steps = steps.stream().filter(step -> step.getTimeInterval().getEndTime() > yoTime.getDoubleValue()).collect(Collectors.toList());
         planner.computeSetpoints(yoTime.getDoubleValue(), steps, feetInContact);

         desiredCoMPosition.set(planner.getDesiredCoMPosition());
         desiredCoMVelocity.set(planner.getDesiredCoMVelocity());
         desiredCoMAcceleration.set(planner.getDesiredCoMAcceleration());
         desiredDCMPosition.set(planner.getDesiredDCMPosition());
         desiredDCMVelocity.set(planner.getDesiredDCMVelocity());
         desiredVRPPosition.set(planner.getDesiredVRPPosition());
         desiredCoPPosition.set(desiredVRPPosition);
         desiredCoPPosition.setZ(0.0);

         desiredForce.set(desiredCoMAcceleration);
         desiredForce.addZ(gravity);
         desiredForce.scale(mass);

         dcmTrajectory.setBallLoop(desiredDCMPosition);
         comTrajectory.setBallLoop(desiredCoMPosition);
         vrpTrajectory.setBallLoop(desiredVRPPosition);

         assertDCMDynamicsHold();
         assertCoMDynamicsHold();

         scs.tickAndUpdate();

         yoTime.add(simDt);
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

   private final FramePoint3D tempPoint = new FramePoint3D();

   private void updateFeetStates(double currentTime)
   {
      feetInContact.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         feetInContact.add(robotQuadrant);

      if (changeOfStateEvents.size() > 0 && (changeOfStateEvents.get(0) <= currentTime))
      {
         planner.setInitialState(yoTime.getDoubleValue(), desiredCoMPosition, desiredCoMVelocity, desiredCoPPosition);
         changeOfStateEvents.removeAt(0);
      }

      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedStep step = steps.get(i);
         if (step.getTimeInterval().intervalContains(currentTime))
         {
            soleFramesForModifying.get(step.getRobotQuadrant()).updateTranslation(step.getGoalPosition());
            feetInContact.remove(step.getRobotQuadrant());
         }
      }

      supportPolygonViz.clear();
      for (int i = 0; i < feetInContact.size(); i++)
      {
         tempPoint.setToZero(soleFramesForModifying.get(feetInContact.get(i)));
         tempPoint.changeFrame(worldFrame);
         supportPolygonViz.addVertex(tempPoint);
      }
   }

   interface StepGetter
   {
      Pair<List<QuadrupedTimedStep>, TDoubleArrayList> getSteps(QuadrantDependentList<MovingReferenceFrame> soleFrames);
   }

   public static void main(String[] args)
   {
      if (planForFlight)
         new QuadrupedCoMTrajectoryPlannerVisualizer(QuadrupedCoMTrajectoryPlannerVisualizer::createFlyingTrotSteps);
      else if (doTrot)
         new QuadrupedCoMTrajectoryPlannerVisualizer(QuadrupedCoMTrajectoryPlannerVisualizer::createTrotSteps);
      else
         new QuadrupedCoMTrajectoryPlannerVisualizer(QuadrupedCoMTrajectoryPlannerVisualizer::createAmbleSteps);
   }
}
