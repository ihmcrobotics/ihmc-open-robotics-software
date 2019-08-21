package us.ihmc.quadrupedRobotics.controlModules;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.*;

public class QuadrupedBalanceBasedStepDelayerTest
{
   private QuadrantDependentList<MovingReferenceFrame> soleFrames;
   private ReferenceFrame midFeetZUpFrame;
   private DoubleProvider omega;
   private QuadrantDependentList<DummyContactState> contactStates;
   private static final double controlDt = 0.01;
   private YoVariableRegistry registry;

   private static final double stanceLength = 1.0;
   private static final double stanceWidth = 0.5;

   private QuadrupedBalanceBasedStepDelayer stepDelayer;

   @BeforeEach
   public void setupTest()
   {
      soleFrames = new QuadrantDependentList<>();
      contactStates = new QuadrantDependentList<>();
      midFeetZUpFrame = ReferenceFrame.getWorldFrame();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(0.5 * robotQuadrant.getEnd().negateIfHindEnd(stanceLength) , 0.5 * robotQuadrant.getSide().negateIfRightSide(stanceWidth), 0.0);
         MovingReferenceFrame soleFrame = MovingReferenceFrame.constructFrameFixedInParent(robotQuadrant.getShortName() + "SoleFrame", ReferenceFrame.getWorldFrame(), transform);

         soleFrames.put(robotQuadrant, soleFrame);
         contactStates.put(robotQuadrant, new DummyContactState());
      }
      omega = () -> 3.0;
      registry = new YoVariableRegistry("testRegistry");
      stepDelayer = new QuadrupedBalanceBasedStepDelayer(soleFrames, midFeetZUpFrame, omega, contactStates, controlDt, registry, null);

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);
   }

   @AfterEach
   public void tearDownTest()
   {
      soleFrames = null;
      contactStates = null;
      midFeetZUpFrame = null;
      omega = null;
      registry = null;
      stepDelayer = null;
   }

   @Test
   public void testNoDelay() throws Exception
   {
      BooleanParameter allowDelayingSteps = (BooleanParameter) getParameter("allowingDelayingSteps");
      BooleanParameter delayAllSubsequentSteps = (BooleanParameter) getParameter("delayAllSubsequentSteps");
      BooleanParameter requireTwoFeetInContact = (BooleanParameter) getParameter("requireTwoFeetInContact");
      BooleanParameter requireFootOnEachEnd = (BooleanParameter) getParameter("requireFootOnEachEnd");

      setValueOfBooleanParameter(allowDelayingSteps, true);
      setValueOfBooleanParameter(delayAllSubsequentSteps, true);
      setValueOfBooleanParameter(requireTwoFeetInContact, true);
      setValueOfBooleanParameter(requireFootOnEachEnd, true);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);

      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep frontLeftStep = new QuadrupedTimedStep();
      frontLeftStep.getTimeInterval().setInterval(0.5, 1.0);
      frontLeftStep.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      frontLeftStep.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));

      activeSteps.add(frontLeftStep);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.05, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.04, 0.0);

      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);

      assertEquals(1, updatedActiveSteps.size());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         assertFalse(stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }

   @Test
   public void testDelayBecauseICPOutside() throws Exception
   {
      RobotQuadrant quadrantToStepWith = RobotQuadrant.FRONT_LEFT;

      BooleanParameter allowDelayingSteps = (BooleanParameter) getParameter("allowingDelayingSteps");
      BooleanParameter requireTwoFeetInContact = (BooleanParameter) getParameter("requireTwoFeetInContact");
      BooleanParameter requireFootOnEachEnd = (BooleanParameter) getParameter("requireFootOnEachEnd");

      setValueOfBooleanParameter(allowDelayingSteps, true);
      setValueOfBooleanParameter(requireTwoFeetInContact, false);
      setValueOfBooleanParameter(requireFootOnEachEnd, false);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);

      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep frontLeftStep = new QuadrupedTimedStep();
      frontLeftStep.getTimeInterval().setInterval(0.5, 1.0);
      frontLeftStep.setRobotQuadrant(quadrantToStepWith);
      frontLeftStep.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));



      activeSteps.add(frontLeftStep);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.05, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.06, 0.0);

      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);

      assertEquals(0, updatedActiveSteps.size());
      assertTrue(stepDelayer.getStepWasDelayed(quadrantToStepWith));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == quadrantToStepWith)
            continue;
         assertFalse(stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }

   @Test
   public void testPreventingLessThanTwoFeet() throws Exception
   {
      BooleanParameter allowDelayingSteps = (BooleanParameter) getParameter("allowingDelayingSteps");
      BooleanParameter requireTwoFeetInContact = (BooleanParameter) getParameter("requireTwoFeetInContact");
      BooleanParameter requireFootOnEachEnd = (BooleanParameter) getParameter("requireFootOnEachEnd");

      setValueOfBooleanParameter(allowDelayingSteps, false);
      setValueOfBooleanParameter(requireTwoFeetInContact, true);
      setValueOfBooleanParameter(requireFootOnEachEnd, false);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);

      contactStates.get(RobotQuadrant.FRONT_LEFT).setInContact(false);
      contactStates.get(RobotQuadrant.HIND_RIGHT).setInContact(false);


      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep frontLeftStep = new QuadrupedTimedStep();
      frontLeftStep.getTimeInterval().setInterval(0.5, 1.0);
      frontLeftStep.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      frontLeftStep.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));

      activeSteps.add(frontLeftStep);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);

      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 0.0);

      assertEquals(0, updatedActiveSteps.size());
      assertTrue(stepDelayer.getStepWasDelayed(RobotQuadrant.FRONT_RIGHT));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == RobotQuadrant.FRONT_RIGHT)
            continue;
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }

   @Test
   public void testRequireFootOnEachEnd() throws Exception
   {
      BooleanParameter allowDelayingSteps = (BooleanParameter) getParameter("allowingDelayingSteps");
      BooleanParameter requireTwoFeetInContact = (BooleanParameter) getParameter("requireTwoFeetInContact");
      BooleanParameter requireFootOnEachEnd = (BooleanParameter) getParameter("requireFootOnEachEnd");

      setValueOfBooleanParameter(allowDelayingSteps, false);
      setValueOfBooleanParameter(requireTwoFeetInContact, false);
      setValueOfBooleanParameter(requireFootOnEachEnd, true);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);

      contactStates.get(RobotQuadrant.FRONT_LEFT).setInContact(false);


      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep frontLeftStep = new QuadrupedTimedStep();
      frontLeftStep.getTimeInterval().setInterval(0.5, 1.0);
      frontLeftStep.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      frontLeftStep.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));

      activeSteps.add(frontLeftStep);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);

      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 0.0);

      assertEquals(0, updatedActiveSteps.size());
      assertTrue(stepDelayer.getStepWasDelayed(RobotQuadrant.FRONT_RIGHT));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == RobotQuadrant.FRONT_RIGHT)
            continue;
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }

   @Test
   public void testFootIsHelpingToPush() throws Exception
   {
      RobotQuadrant quadrantToStepWith = RobotQuadrant.HIND_LEFT;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);

      FrameConvexPolygon2D polygonAfterStep = new FrameConvexPolygon2D();
      FrameConvexPolygon2D scaledPolygonAfterStep = new FrameConvexPolygon2D();


      double distanceInsideToNotDelay = 0.05;
      DoubleParameter distance = (DoubleParameter) getParameter("minimumICPDistanceFromEdgeForNotNeeded");
      BooleanParameter delayFootIfItsHelpingButNotNeeded = (BooleanParameter) getParameter("delayFootIfItsHelpingButNotNeeded");
      setValueOfDoubleParameter(distance, distanceInsideToNotDelay);
      setValueOfBooleanParameter(delayFootIfItsHelpingButNotNeeded, true);

      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep step = new QuadrupedTimedStep();
      step.getTimeInterval().setInterval(0.5, 1.0);
      step.setRobotQuadrant(quadrantToStepWith);
      step.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));

      activeSteps.add(step);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.06, 0.0);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!contactStates.get(robotQuadrant).inContact() || quadrantToStepWith == robotQuadrant)
            continue;
         FramePoint3D footPoint = new FramePoint3D(soleFrames.get(robotQuadrant));
         polygonAfterStep.addVertexMatchingFrame(footPoint);
      }
      polygonAfterStep.update();
      ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
      polygonScaler.scaleConvexPolygon(polygonAfterStep, distanceInsideToNotDelay, scaledPolygonAfterStep);


      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);



      assertEquals(0, updatedActiveSteps.size());
      assertTrue(stepDelayer.getStepWasDelayed(quadrantToStepWith));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == quadrantToStepWith)
            continue;
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }

      // project the current icp to the threshold, meaning it should still delay
      FramePoint2D currentICP2D = new FramePoint2D(currentICP);
      scaledPolygonAfterStep.orthogonalProjection(currentICP2D);
      currentICP.set(currentICP2D);

      updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);
      assertEquals(0, updatedActiveSteps.size());
      assertTrue(stepDelayer.getStepWasDelayed(quadrantToStepWith));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == quadrantToStepWith)
            continue;
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }

      // project it just past the threshold, which should mean it doesn't delay
      polygonScaler.scaleConvexPolygon(polygonAfterStep, distanceInsideToNotDelay + 1e-4, scaledPolygonAfterStep);

      currentICP2D = new FramePoint2D(currentICP);
      scaledPolygonAfterStep.orthogonalProjection(currentICP2D);
      currentICP.set(currentICP2D);

      updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);
      assertEquals(1, updatedActiveSteps.size());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }

      setValueOfDoubleParameter(distance, 0.15);

      currentICP2D = new FramePoint2D(currentICP);
      scaledPolygonAfterStep.orthogonalProjection(currentICP2D);
      currentICP.set(currentICP2D);

      updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);
      assertEquals(0, updatedActiveSteps.size());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == quadrantToStepWith)
            continue;

         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }

      setValueOfBooleanParameter(delayFootIfItsHelpingButNotNeeded, false);

      updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);
      assertEquals(1, updatedActiveSteps.size());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }


   @Test
   public void testReallyDynamicWalkButNotTheRightStep() throws Exception
   {
      RobotQuadrant quadrantToStepWith = RobotQuadrant.HIND_LEFT;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);
      contactStates.get(RobotQuadrant.FRONT_RIGHT).setInContact(false);

      FrameConvexPolygon2D polygonAfterStep = new FrameConvexPolygon2D();
      FrameConvexPolygon2D scaledPolygonAfterStep = new FrameConvexPolygon2D();


      double distanceInsideToNotDelay = 0.05;
      DoubleParameter distance = (DoubleParameter) getParameter("minimumICPDistanceFromEdgeForNotNeeded");
      BooleanParameter delayFootIfItsHelpingButNotNeeded = (BooleanParameter) getParameter("delayFootIfItsHelpingButNotNeeded");
      setValueOfDoubleParameter(distance, distanceInsideToNotDelay);
      setValueOfBooleanParameter(delayFootIfItsHelpingButNotNeeded, true);

      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep step = new QuadrupedTimedStep();
      step.getTimeInterval().setInterval(0.5, 1.0);
      step.setRobotQuadrant(quadrantToStepWith);
      step.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));

      activeSteps.add(step);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.05, -0.04, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.06, 0.0);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!contactStates.get(robotQuadrant).inContact() || quadrantToStepWith == robotQuadrant)
            continue;
         FramePoint3D footPoint = new FramePoint3D(soleFrames.get(robotQuadrant));
         polygonAfterStep.addVertexMatchingFrame(footPoint);
      }
      polygonAfterStep.update();
      ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
      polygonScaler.scaleConvexPolygon(polygonAfterStep, distanceInsideToNotDelay, scaledPolygonAfterStep);


      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);

      assertEquals(1, updatedActiveSteps.size());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }

   @Test
   public void testReallyDynamicWalkWithTheRightStep() throws Exception
   {
      RobotQuadrant quadrantToStepWith = RobotQuadrant.HIND_RIGHT;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);
      contactStates.get(RobotQuadrant.FRONT_RIGHT).setInContact(false);

      FrameConvexPolygon2D polygonAfterStep = new FrameConvexPolygon2D();
      FrameConvexPolygon2D scaledPolygonAfterStep = new FrameConvexPolygon2D();


      double distanceInsideToNotDelay = 0.05;
      DoubleParameter distance = (DoubleParameter) getParameter("minimumICPDistanceFromEdgeForNotNeeded");
      BooleanParameter delayFootIfItsHelpingButNotNeeded = (BooleanParameter) getParameter("delayFootIfItsHelpingButNotNeeded");
      setValueOfDoubleParameter(distance, distanceInsideToNotDelay);
      setValueOfBooleanParameter(delayFootIfItsHelpingButNotNeeded, true);

      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep step = new QuadrupedTimedStep();
      step.getTimeInterval().setInterval(0.5, 1.0);
      step.setRobotQuadrant(quadrantToStepWith);
      step.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));

      activeSteps.add(step);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.05, -0.04, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.1, -0.06, 0.0);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!contactStates.get(robotQuadrant).inContact() || quadrantToStepWith == robotQuadrant)
            continue;
         FramePoint3D footPoint = new FramePoint3D(soleFrames.get(robotQuadrant));
         polygonAfterStep.addVertexMatchingFrame(footPoint);
      }
      polygonAfterStep.update();
      ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
      polygonScaler.scaleConvexPolygon(polygonAfterStep, distanceInsideToNotDelay, scaledPolygonAfterStep);


      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);

      assertEquals(0, updatedActiveSteps.size());
      assertTrue(quadrantToStepWith.getShortName() + " was not delayed.", stepDelayer.getStepWasDelayed(quadrantToStepWith));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (quadrantToStepWith == robotQuadrant)
            continue;
         assertFalse(robotQuadrant.getShortName() + " was delayed.", stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }

   private YoParameter<?> getParameter(String name)
   {
      for (YoParameter<?> parameter : registry.getAllParameters())
      {
         if (parameter.getName().equals(name))
            return parameter;
      }

      throw new RuntimeException("Variable doesn't exist.");
   }

   private void setValueOfDoubleParameter(DoubleParameter parameter, double value) throws Exception
   {
      Method method = parameter.getClass().getDeclaredMethod("getVariable", null);
      method.setAccessible(true);
      ((YoDouble) method.invoke(parameter)).set(value);
   }

   private void setValueOfBooleanParameter(BooleanParameter parameter, boolean value) throws Exception
   {
      Method method = parameter.getClass().getDeclaredMethod("getVariable", null);
      method.setAccessible(true);
      ((YoBoolean) method.invoke(parameter)).set(value);
   }

   public class DummyContactState implements PlaneContactState
   {
      boolean inContact = true;

      public void setInContact(boolean inContact)
      {
         this.inContact = inContact;
      }

      @Override
      public RigidBodyBasics getRigidBody()
      {
         return null;
      }

      @Override
      public ReferenceFrame getFrameAfterParentJoint()
      {
         return null;
      }

      @Override
      public ReferenceFrame getPlaneFrame()
      {
         return null;
      }

      @Override
      public boolean inContact()
      {
         return inContact;
      }

      @Override
      public FrameVector3D getContactNormalFrameVectorCopy()
      {
         return null;
      }

      @Override
      public void getContactNormalFrameVector(FrameVector3D frameVectorToPack)
      {

      }

      @Override
      public List<FramePoint3D> getContactFramePointsInContactCopy()
      {
         return null;
      }

      @Override
      public void getContactFramePointsInContact(List<FramePoint3D> contactPointListToPack)
      {

      }

      @Override
      public List<FramePoint2D> getContactFramePoints2dInContactCopy()
      {
         return null;
      }

      @Override
      public double getCoefficientOfFriction()
      {
         return 0;
      }

      @Override
      public int getNumberOfContactPointsInContact()
      {
         return 0;
      }

      @Override
      public int getTotalNumberOfContactPoints()
      {
         return 0;
      }

      @Override
      public List<? extends ContactPointInterface> getContactPoints()
      {
         return null;
      }

      @Override
      public void updateFromPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommand)
      {

      }

      @Override
      public void getPlaneContactStateCommand(PlaneContactStateCommand planeContactStateCommandToPack)
      {

      }

      @Override
      public void notifyContactStateHasChanged()
      {

      }

      @Override
      public boolean pollContactHasChangedNotification()
      {
         return false;
      }

      @Override
      public boolean peekContactHasChangedNotification()
      {
         return false;
      }
   }

}
