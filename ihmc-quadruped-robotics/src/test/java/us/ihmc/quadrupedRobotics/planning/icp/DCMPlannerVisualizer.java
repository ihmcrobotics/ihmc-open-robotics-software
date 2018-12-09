package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
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

public class DCMPlannerVisualizer
{
   private static final double stanceLength = 1.0;
   private static final double stanceWidth = 0.5;
   private static final double gravity = 9.81;
   private static final double nominalHeight = 0.75;

   private static final int BUFFER_SIZE = 16000;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private final DCMBasedCoMPlanner planner;

   private List<QuadrupedTimedStep> steps;
   private final QuadrantDependentList<ContactState> contactStates = new QuadrantDependentList<>();

   private final YoFramePoint3D desiredICPPosition;
   private final YoFrameVector3D desiredICPVelocity;

   public DCMPlannerVisualizer()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      QuadrantDependentList<ReferenceFrame> soleFrames = createSoleFrames();
      planner = new DCMBasedCoMPlanner(soleFrames, gravity, nominalHeight, registry);
      steps = createSteps(soleFrames);

      desiredICPPosition = new YoFramePoint3D("desiredICPPosition", worldFrame, registry);
      desiredICPVelocity = new YoFrameVector3D("desiredICPVelocity", worldFrame, registry);

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.put(robotQuadrant, ContactState.IN_CONTACT);

      yoTime = new YoDouble("timeToCheck", registry);
      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.addYoVariableRegistry(registry);
      //      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      //      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      simulate();
      ThreadTools.sleepForever();
   }

   private QuadrantDependentList<ReferenceFrame> createSoleFrames()
   {
      QuadrantDependentList<ReferenceFrame> soleFrames = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         TranslationReferenceFrame soleFrame = new TranslationReferenceFrame(robotQuadrant + "SoleFrame", worldFrame);
         Vector3D translation = new Vector3D();
         translation.setX(robotQuadrant.getEnd().negateIfHindEnd(stanceLength / 2.0));
         translation.setY(robotQuadrant.getSide().negateIfRightSide(stanceWidth / 2.0));
         soleFrame.updateTranslation(translation);

         soleFrames.put(robotQuadrant, soleFrame);
      }

      return soleFrames;
   }

   private final double stepStartTime = 1.0;
   private final double stepDuration = 0.4;
   private final double stanceDuration = 0.2;
   private final double stepLength = 0.5;
   private final int numberOfSteps = 5;

   private final FramePoint3D frontLeftStep = new FramePoint3D();
   private final FramePoint3D frontRightStep = new FramePoint3D();
   private final FramePoint3D hindLeftStep = new FramePoint3D();
   private final FramePoint3D hindRightStep = new FramePoint3D();

   private List<QuadrupedTimedStep> createSteps(QuadrantDependentList<ReferenceFrame> soleFrames)
   {
      int stepInterval = 0;
      List<QuadrupedTimedStep> steps = new ArrayList<>();

      frontLeftStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_LEFT));
      frontRightStep.setToZero(soleFrames.get(RobotQuadrant.FRONT_RIGHT));
      hindLeftStep.setToZero(soleFrames.get(RobotQuadrant.HIND_LEFT));
      hindRightStep.setToZero(soleFrames.get(RobotQuadrant.HIND_RIGHT));

      frontLeftStep.changeFrame(worldFrame);
      frontRightStep.changeFrame(worldFrame);
      hindLeftStep.changeFrame(worldFrame);
      hindRightStep.changeFrame(worldFrame);

      double currentTime = stepStartTime;
      for (int i = 0; i < numberOfSteps; i++)
      {
         frontLeftStep.addX(stepLength);
         frontRightStep.addX(stepLength);
         hindLeftStep.addX(stepLength);
         hindRightStep.addX(stepLength);

         QuadrupedTimedStep step1 = new QuadrupedTimedStep();
         QuadrupedTimedStep step2 = new QuadrupedTimedStep();

         step1.getTimeInterval().setStartTime(currentTime);
         step1.getTimeInterval().setEndTime(currentTime + stepDuration);
         step2.getTimeInterval().setStartTime(currentTime);
         step2.getTimeInterval().setEndTime(currentTime + stepDuration);

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

      return steps;
   }

   private void simulate()
   {
      while (true)
      {
         planner.clearStepSequence();
         for (int i = 0; i < steps.size(); i++)
            planner.addStepToSequence(steps.get(i));

         planner.initialize();
         double currentTime = yoTime.getDoubleValue();
         updateContactState(currentTime);
         planner.computeSetpoints(currentTime, contactStates, desiredICPPosition, desiredICPVelocity);

         scs.tickAndUpdate();
      }
   }

   private void updateContactState(double currentTime)
   {
      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedStep step = steps.get(i);
         if (step.getTimeInterval().intervalContains(currentTime))
         {
            contactStates.put(step.getRobotQuadrant(), ContactState.NO_CONTACT);
         }
         else
         {
            contactStates.put(step.getRobotQuadrant(), ContactState.IN_CONTACT);
         }
      }
   }

   public static void main(String[] args)
   {
      DCMPlannerVisualizer visualizer = new DCMPlannerVisualizer();
   }
}
