package us.ihmc.quadrupedRobotics.controlModules;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointInterface;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
   public void testNoDelay()
   {
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
   public void testDelayBecauseICPOutside()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         contactStates.get(robotQuadrant).setInContact(true);

      List<QuadrupedTimedStep> activeSteps = new ArrayList<>();
      List<QuadrupedTimedStep> otherSteps = new ArrayList<>();
      QuadrupedTimedStep frontLeftStep = new QuadrupedTimedStep();
      frontLeftStep.getTimeInterval().setInterval(0.5, 1.0);
      frontLeftStep.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      frontLeftStep.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.5 * stanceLength, 0.5 * stanceWidth, 0.0));

      activeSteps.add(frontLeftStep);

      FramePoint3D currentICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, 0.05, 0.0);
      FramePoint3D desiredICP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, -0.06, 0.0);

      List<? extends QuadrupedTimedStep> updatedActiveSteps = stepDelayer.delayStepsIfNecessary(activeSteps, otherSteps, desiredICP, currentICP, 10.0);

      assertEquals(0, updatedActiveSteps.size());
      assertTrue(stepDelayer.getStepWasDelayed(RobotQuadrant.FRONT_LEFT));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == RobotQuadrant.FRONT_LEFT)
            continue;
         assertFalse(stepDelayer.getStepWasDelayed(robotQuadrant));
      }
   }

   @Test
   public void testPreventingLessThanTwoFeet()
   {
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
   public void testRequireFootOnEachEnd()
   {
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
