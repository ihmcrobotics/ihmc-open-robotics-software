package us.ihmc.quadrupedRobotics.planning.comPlanning;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.TranslationMovingReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedCoMTrajectoryPlannerTest
{
   private static final double epsilon = 1e-6;

   private final static double gravityZ = 9.81;
   private final static double omegaValue = 3.0;
   private static final double nominalHeight = gravityZ / MathTools.square(omegaValue);
   private static final double nominalLength = 1.0;
   private static final double nominalWidth = 0.5;

   private YoRegistry registry;
   private YoDouble omega;
   private YoDouble time;
   private QuadrantDependentList<MovingReferenceFrame> soleFrames;
   private QuadrantDependentList<YoEnum<ContactState>> contactStates;
   private QuadrupedCoMTrajectoryPlanner planner;

   @BeforeEach
   public void construct()
   {
      registry = new YoRegistry("test");
      omega = new YoDouble("omega", registry);
      omega.set(omegaValue);

      time = new YoDouble("time", registry);

      soleFrames = DCMPlanningTestTools.createSimpleSoleFrames(nominalLength, nominalWidth);
      contactStates = new QuadrantDependentList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         contactStates.put(quadrant, new YoEnum<>(quadrant.getShortName() + "ContactState", registry, ContactState.class));
      planner = new QuadrupedCoMTrajectoryPlanner(new TestDCMPlannerParameters(), soleFrames, gravityZ, nominalHeight, registry);
   }

   @AfterEach
   public void destroy()
   {
      registry = null;
      omega = null;
      time = null;

      soleFrames = null;
      contactStates = null;
      planner = null;
   }

   @Test
   public void testStanding()
   {
      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setStartECMPPosition(new FramePoint3D());

      List<RobotQuadrant> feetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         feetInContact.add(quadrant);

      planner.initialize();

      FramePoint3D expectedDesiredDCM = new FramePoint3D();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D solePosition = new FramePoint3D(soleFrames.get(robotQuadrant));
         solePosition.changeFrame(ReferenceFrame.getWorldFrame());
         expectedDesiredDCM.add(solePosition);
      }
      expectedDesiredDCM.scale(0.25);
      expectedDesiredDCM.addZ(nominalHeight);

      for (double timeInState = 0.0; timeInState < 5000; timeInState += 0.5)
      {
         planner.computeSetpoints(timeInState, new ArrayList<>(), feetInContact);

         EuclidFrameTestTools.assertGeometricallyEquals("Position failed at t = " + timeInState, expectedDesiredDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("Velocity failed at t = " + timeInState, new FrameVector3D(), planner.getDesiredDCMVelocity(), epsilon);
      }
   }

   @Test
   public void testOneStep()
   {
      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setStartECMPPosition(new FramePoint3D());

      List<RobotQuadrant> feetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         feetInContact.add(quadrant);

      planner.initialize();
      planner.setInitialState(0.0, new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, nominalHeight), new FrameVector3D(), new FramePoint3D());

      FramePoint3D expectedDesiredDCM = new FramePoint3D();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D solePosition = new FramePoint3D(soleFrames.get(robotQuadrant));
         solePosition.changeFrame(ReferenceFrame.getWorldFrame());
         expectedDesiredDCM.add(solePosition);
      }
      expectedDesiredDCM.scale(0.25);
      expectedDesiredDCM.addZ(nominalHeight);

      List<QuadrupedTimedStep> steps = new ArrayList<>();

      for (double timeInState = 0.0; timeInState < 0.5; timeInState += 0.5)
      {
         planner.computeSetpoints(timeInState, steps, feetInContact);

         EuclidFrameTestTools.assertGeometricallyEquals("Position failed at t = " + timeInState, expectedDesiredDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("Velocity failed at t = " + timeInState, new FrameVector3D(), planner.getDesiredDCMVelocity(), epsilon);
      }

      time.set(0.3);

      QuadrupedTimedStep step = new QuadrupedTimedStep();
      step.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      step.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), nominalLength / 2 + 0.2, nominalWidth / 2.0, 0.0));
      step.getTimeInterval().setInterval(0.5, 1.5);

      steps.add(step);

      contactStates.get(RobotQuadrant.FRONT_LEFT).set(ContactState.IN_CONTACT);
      contactStates.get(RobotQuadrant.FRONT_RIGHT).set(ContactState.IN_CONTACT);
      contactStates.get(RobotQuadrant.HIND_RIGHT).set(ContactState.IN_CONTACT);
      contactStates.get(RobotQuadrant.HIND_LEFT).set(ContactState.IN_CONTACT);

      // add the step in, and do the initial transfer
      planner.initialize();
      planner.setInitialState(0.0, planner.getDesiredCoMPosition(), planner.getDesiredCoMVelocity(), planner.getDesiredECMPPosition());

      for (; time.getDoubleValue() < 0.5; time.add(0.05))
      {
         planner.computeSetpoints(time.getDoubleValue(), steps, feetInContact);
      }

      // remove the front left foot, since it is the one that's stepping, and compute the step trajectory
      feetInContact.remove(RobotQuadrant.FRONT_LEFT);

      for (; time.getDoubleValue() < 1.5; time.add(0.05))
      {
         planner.computeSetpoints(time.getDoubleValue(), steps, feetInContact);
      }

      // update the foot positions, since we just finished the step
      MovingReferenceFrame newSoleFrame = new TranslationMovingReferenceFrame("newSoleFrame", ReferenceFrame.getWorldFrame());
      ((TranslationMovingReferenceFrame) newSoleFrame).updateTranslation(step.getGoalPosition());
      soleFrames.put(RobotQuadrant.FRONT_LEFT, newSoleFrame);

      feetInContact.add(RobotQuadrant.FRONT_LEFT);

      FramePoint3D finalDCM = new FramePoint3D();

      FramePoint3D solePosition = new FramePoint3D(soleFrames.get(RobotQuadrant.FRONT_RIGHT));
      solePosition.changeFrame(ReferenceFrame.getWorldFrame());
      finalDCM.add(solePosition);

      solePosition = new FramePoint3D(soleFrames.get(RobotQuadrant.HIND_LEFT));
      solePosition.changeFrame(ReferenceFrame.getWorldFrame());
      finalDCM.add(solePosition);

      solePosition = new FramePoint3D(soleFrames.get(RobotQuadrant.HIND_RIGHT));
      solePosition.changeFrame(ReferenceFrame.getWorldFrame());
      finalDCM.add(solePosition);

      finalDCM.add(step.getGoalPosition());
      finalDCM.scale(0.25);
      finalDCM.addZ(nominalHeight);
      FrameVector3DReadOnly zero = new FrameVector3D();

      planner.initialize();
      planner.setInitialState(0.0, planner.getDesiredCoMPosition(), planner.getDesiredCoMVelocity(), planner.getDesiredECMPPosition());

      for (; time.getDoubleValue() < 100.0; time.add(0.5))
      {
         planner.computeSetpoints(time.getDoubleValue(), steps, feetInContact);

         EuclidFrameTestTools.assertGeometricallyEquals("time = " + time.getDoubleValue(), finalDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("time = " + time.getDoubleValue(), zero, planner.getDesiredDCMVelocity(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals("time = " + time.getDoubleValue(), finalDCM, planner.getDesiredVRPPosition(), epsilon);

         // give a bunch of extra convergence time before starting to check this
         if (time.getDoubleValue() > 3.0)
         {
            EuclidFrameTestTools.assertGeometricallyEquals("time = " + time.getDoubleValue(), finalDCM, planner.getDesiredCoMPosition(), 0.01);
            EuclidFrameTestTools.assertGeometricallyEquals("time = " + time.getDoubleValue(), zero, planner.getDesiredCoMVelocity(), 0.01);
            EuclidFrameTestTools.assertGeometricallyEquals("time = " + time.getDoubleValue(), zero, planner.getDesiredCoMAcceleration(), 0.01);
         }
      }
   }

   @Test
   public void testFlyingTrot()
   {
      new QuadrupedCoMTrajectoryPlannerVisualizer(QuadrupedCoMTrajectoryPlannerVisualizer::createFlyingTrotSteps);
   }

   @Test
   public void testTrot()
   {
      new QuadrupedCoMTrajectoryPlannerVisualizer(QuadrupedCoMTrajectoryPlannerVisualizer::createTrotSteps);
   }

   @Test
   public void testAmble()
   {
      new QuadrupedCoMTrajectoryPlannerVisualizer(QuadrupedCoMTrajectoryPlannerVisualizer::createAmbleSteps);
   }
}
