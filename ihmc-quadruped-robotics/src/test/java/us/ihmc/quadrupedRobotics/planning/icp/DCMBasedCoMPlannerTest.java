package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class DCMBasedCoMPlannerTest
{
   private static final double epsilon = 1e-6;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
      DCMBasedCoMPlanner planner = new DCMBasedCoMPlanner(soleFrames, time, omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setCopPosition(new FramePoint3D());

      contactSequence.add(firstContact);

      List<RobotQuadrant> feetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         feetInContact.add(quadrant);

      FramePoint3D desiredDCM = new FramePoint3D();
      FrameVector3D desiredDCMVelocity = new FrameVector3D();
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
         planner.computeSetpoints(timeInState, feetInContact, desiredDCM, desiredDCMVelocity);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("Position failed at t = " + timeInState, expectedDesiredDCM, desiredDCM, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Velocity failed at t = " + timeInState, new FrameVector3D(), desiredDCMVelocity, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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
      DCMBasedCoMPlanner planner = new DCMBasedCoMPlanner(soleFrames, time, omega, gravityZ, nominalHeight, registry);

      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setCopPosition(new FramePoint3D());

      contactSequence.add(firstContact);

      List<RobotQuadrant> feetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         feetInContact.add(quadrant);

      FramePoint3D desiredDCM = new FramePoint3D();
      FrameVector3D desiredDCMVelocity = new FrameVector3D();
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

      for (double timeInState = 0.0; timeInState < 0.5; timeInState += 0.5)
      {
         planner.computeSetpoints(timeInState, feetInContact, desiredDCM, desiredDCMVelocity);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("Position failed at t = " + timeInState, expectedDesiredDCM, desiredDCM, epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("Velocity failed at t = " + timeInState, new FrameVector3D(), desiredDCMVelocity, epsilon);
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

      planner.addStepToSequence(step);
      planner.initializeForStepping(contactStates, null);

      for (; time.getDoubleValue() < 0.5; time.add(0.05))
      {
         planner.computeSetpoints(time.getDoubleValue(), feetInContact, desiredDCM, desiredDCMVelocity);
      }

      feetInContact.remove(RobotQuadrant.FRONT_LEFT);

      for (; time.getDoubleValue() < 1.5; time.add(0.05))
         ;
      {
         planner.computeSetpoints(time.getDoubleValue(), feetInContact, desiredDCM, desiredDCMVelocity);
      }

      MovingReferenceFrame newSoleFrame = new TranslationMovingReferenceFrame("newSOleFrame", ReferenceFrame.getWorldFrame());
      ((TranslationMovingReferenceFrame) newSoleFrame).updateTranslation(step.getGoalPositionProvider());
      soleFrames.put(RobotQuadrant.FRONT_LEFT, newSoleFrame);

      feetInContact.add(RobotQuadrant.FRONT_LEFT);
      for (; time.getDoubleValue() < 1.0; time.add(0.05))
         ;
      {
         planner.computeSetpoints(time.getDoubleValue(), feetInContact, desiredDCM, desiredDCMVelocity);
      }

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

      for (; time.getDoubleValue() < 100.0; time.add(0.5))
      {
         planner.computeSetpoints(time.getDoubleValue(), feetInContact, desiredDCM, desiredDCMVelocity);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(finalDCM, desiredDCM, epsilon);
      }
   }

}
