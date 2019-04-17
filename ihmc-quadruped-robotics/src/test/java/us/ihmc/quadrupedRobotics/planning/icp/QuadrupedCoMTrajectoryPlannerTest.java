package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedCoMTrajectoryPlannerTest
{
   private static final double epsilon = 1e-6;

   @Test
   public void testStanding()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;
      double nominalLength = 1.0;
      double nominalWidth = 0.5;

      YoDouble time = new YoDouble("time", registry);

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      QuadrantDependentList<MovingReferenceFrame> soleFrames = DCMPlanningTestTools.createSimpleSoleFrames(nominalLength, nominalWidth);
      QuadrupedCoMTrajectoryPlanner planner = new QuadrupedCoMTrajectoryPlanner(soleFrames, time, omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setStartCopPosition(new FramePoint3D());

      contactSequence.add(firstContact);

      List<RobotQuadrant> feetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         feetInContact.add(quadrant);

      planner.initializeForStanding();

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
         planner.computeSetpoints(timeInState, feetInContact);

         EuclidFrameTestTools
               .assertFramePoint3DGeometricallyEquals("Position failed at t = " + timeInState, expectedDesiredDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools
               .assertFrameVector3DGeometricallyEquals("Velocity failed at t = " + timeInState, new FrameVector3D(), planner.getDesiredDCMVelocity(), epsilon);
      }
   }

   @Test
   public void testOneStep()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;
      double nominalLength = 1.0;
      double nominalWidth = 0.5;

      YoDouble time = new YoDouble("time", registry);

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      QuadrantDependentList<MovingReferenceFrame> soleFrames = DCMPlanningTestTools.createSimpleSoleFrames(nominalLength, nominalWidth);
      QuadrantDependentList<YoEnum<ContactState>> contactStates = new QuadrantDependentList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         contactStates.put(quadrant, new YoEnum<>(quadrant.getShortName() + "ContactState", registry, ContactState.class));
      QuadrupedCoMTrajectoryPlanner planner = new QuadrupedCoMTrajectoryPlanner(soleFrames, time, omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setStartCopPosition(new FramePoint3D());

      contactSequence.add(firstContact);

      List<RobotQuadrant> feetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         feetInContact.add(quadrant);

      planner.initializeForStanding();
      planner.setInitialCenterOfMassState(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, nominalHeight), new FrameVector3D());


      FramePoint3D expectedDesiredDCM = new FramePoint3D();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D solePosition = new FramePoint3D(soleFrames.get(robotQuadrant));
         solePosition.changeFrame(ReferenceFrame.getWorldFrame());
         expectedDesiredDCM.add(solePosition);
      }
      expectedDesiredDCM.scale(0.25);
      expectedDesiredDCM.addZ(nominalHeight);

      for (double timeInState = 0.0; timeInState < 0.5; timeInState += 0.5)
      {
         planner.computeSetpoints(timeInState, feetInContact);

         EuclidFrameTestTools
               .assertFramePoint3DGeometricallyEquals("Position failed at t = " + timeInState, expectedDesiredDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools
               .assertFrameVector3DGeometricallyEquals("Velocity failed at t = " + timeInState, new FrameVector3D(), planner.getDesiredDCMVelocity(), epsilon);
      }

      time.set(0.3);

      QuadrupedTimedStep step = new QuadrupedTimedStep();
      step.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      step.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), nominalLength / 2 + 0.2, nominalWidth / 2.0, 0.0));
      step.getTimeInterval().setInterval(0.5, 1.5);

      contactStates.get(RobotQuadrant.FRONT_LEFT).set(ContactState.IN_CONTACT);
      contactStates.get(RobotQuadrant.FRONT_RIGHT).set(ContactState.IN_CONTACT);
      contactStates.get(RobotQuadrant.HIND_RIGHT).set(ContactState.IN_CONTACT);
      contactStates.get(RobotQuadrant.HIND_LEFT).set(ContactState.IN_CONTACT);

      // add the step in, and do the initial transfer
      planner.addStepToSequence(step);
      planner.initializeForStepping(contactStates);
      planner.setInitialCenterOfMassState(planner.getDesiredCoMPosition(), planner.getDesiredCoMVelocity());

      for (; time.getDoubleValue() < 0.5; time.add(0.05))
      {
         planner.computeSetpoints(time.getDoubleValue(), feetInContact);
      }

      // remove the front left foot, since it is the one that's stepping, and compute the step trajectory
      feetInContact.remove(RobotQuadrant.FRONT_LEFT);

      for (; time.getDoubleValue() < 1.5; time.add(0.05))
      {
         planner.computeSetpoints(time.getDoubleValue(), feetInContact);
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

      planner.initializeForStanding();
      planner.setInitialCenterOfMassState(planner.getDesiredCoMPosition(), planner.getDesiredCoMVelocity());

      for (; time.getDoubleValue() < 100.0; time.add(0.5))
      {
         planner.computeSetpoints(time.getDoubleValue(), feetInContact);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("time = " + time.getDoubleValue(), finalDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("time = " + time.getDoubleValue(), zero, planner.getDesiredDCMVelocity(), epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("time = " + time.getDoubleValue(), finalDCM, planner.getDesiredVRPPosition(), epsilon);

         // give a bunch of extra convergence time before starting to check this
         if (time.getDoubleValue() > 3.0)
         {
            EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("time = " + time.getDoubleValue(), finalDCM, planner.getDesiredCoMPosition(), 0.01);
            EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("time = " + time.getDoubleValue(), zero, planner.getDesiredCoMVelocity(), 0.01);
            EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("time = " + time.getDoubleValue(), zero, planner.getDesiredCoMAcceleration(), 0.01);
         }
      }
   }

}
