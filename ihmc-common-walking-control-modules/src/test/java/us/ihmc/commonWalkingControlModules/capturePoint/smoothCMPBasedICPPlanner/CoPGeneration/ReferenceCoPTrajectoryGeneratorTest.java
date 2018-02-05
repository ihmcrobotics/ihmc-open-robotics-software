package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.CoPSupportPolygonNames;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ReferenceCoPTrajectoryGeneratorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final int numberOfContactPoints = 4;
   private final double soleFrameYDisplacement = 0.2;
   private final double ankleFrameZDisplacement = 0.05;
   private final double frontContactX = 0.07;
   private final double rearContactX = -0.07;
   private final double side1ContactY = 0.03;
   private final double side2ContactY = -0.03;
   private final double swingTime = 1;
   private final double transferTime = 0.1;
   private final double stepLength = 0.3;
   private final double stepWidth = soleFrameYDisplacement;
   private final double EPSILON = 5e-4;

   ReferenceCoPTrajectoryGenerator testCoPGenerator;
   SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   SideDependentList<ContactableFoot> contactableFeet = new SideDependentList<>();
   SideDependentList<RigidBody> feetBodies = new SideDependentList<>();
   YoVariableRegistry parentRegistry = new YoVariableRegistry("TestRegistry");
   MidFootZUpGroundFrame midFeetZUpFrame;
   SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   SmoothCMPPlannerParameters plannerParameters;
   YoInteger numberOfFootstepsToConsider = new YoInteger("numberOfFootstepsToConsider", parentRegistry);
   private final ArrayList<YoDouble> swingDurations = new ArrayList<>();
   private final ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();
   private final ArrayList<YoDouble> swingDurationShiftFractions = new ArrayList<>();
   private final ArrayList<YoDouble> transferDurations = new ArrayList<>();
   private final ArrayList<YoDouble> touchdownDurations = new ArrayList<>();
   private final ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();

   @Before
   public void setUp()
   {
      for (RobotSide side : RobotSide.values)
      {
         ZUpFrame soleFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(),
                                           new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.0, side.negateIfRightSide(soleFrameYDisplacement), 0.0),
                                           "DummyRobot" + side.toString() + "FootSoleFrame");
         soleZUpFrames.put(side, soleFrame);
         ZUpFrame ankleFrame = new ZUpFrame(soleFrame, new FramePoint3D(soleFrame, 0.0, 0.0, ankleFrameZDisplacement),
                                            "DummyRobot" + side.toString() + "AnkleSoleFrame");
         ankleZUpFrames.put(side, ankleFrame);
         List<Point2D> contactPoints = new ArrayList<>(numberOfContactPoints);
         Point2D point1 = new Point2D(frontContactX, side.negateIfRightSide(side1ContactY));
         contactPoints.add(point1);
         Point2D point2 = new Point2D(frontContactX, side.negateIfRightSide(side2ContactY));
         contactPoints.add(point2);
         Point2D point3 = new Point2D(rearContactX, side.negateIfRightSide(side1ContactY));
         contactPoints.add(point3);
         Point2D point4 = new Point2D(rearContactX, side.negateIfRightSide(side2ContactY));
         contactPoints.add(point4);
         ContactableFoot contactableFoot = new ListOfPointsContactableFoot(null, soleFrame, contactPoints, point1,
                                                                           new LineSegment2D(point1, point2));
         contactableFeet.put(side, contactableFoot);
         RigidBody feetBody = new RigidBody(side.toString() + "Feet", ReferenceFrame.getWorldFrame());
         feetBodies.put(side, feetBody);
         List<FramePoint2D> contactFramePoints = new ArrayList<>();
         contactFramePoints.add(new FramePoint2D(soleFrame, point1));
         contactFramePoints.add(new FramePoint2D(soleFrame, point2));
         contactFramePoints.add(new FramePoint2D(soleFrame, point3));
         contactFramePoints.add(new FramePoint2D(soleFrame, point4));

         YoPlaneContactState contactState = new YoPlaneContactState("DummyRobot" + side.toString() + "FootContactState", feetBody, soleFrame,
                                                                    contactFramePoints, 10, parentRegistry);
         contactStates.put(side, contactState);
      }
      midFeetZUpFrame = new MidFootZUpGroundFrame("DummyRobotMidFootZUpFrame", soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, parentRegistry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      plannerParameters = new TestSmoothCMPPlannerParameters();
      numberOfFootstepsToConsider.set(plannerParameters.getNumberOfFootstepsToConsider());

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         YoDouble swingDuration = new YoDouble("swingDuration" + i, parentRegistry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, parentRegistry);
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, parentRegistry);
         YoDouble transferDuration = new YoDouble("transferDuration" + i, parentRegistry);
         YoDouble touchdownDuration = new YoDouble("touchdownDuration" + i, parentRegistry);
         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, parentRegistry);

         swingDurations.add(swingDuration);
         swingSplitFractions.add(swingSplitFraction);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
         transferDurations.add(transferDuration);
         touchdownDurations.add(touchdownDuration);
         transferSplitFractions.add(transferSplitFraction);
      }
      YoDouble transferDuration = new YoDouble("transferDuration" + numberOfFootstepsToConsider.getIntegerValue(), parentRegistry);
      YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + numberOfFootstepsToConsider.getIntegerValue(), parentRegistry);
      transferDurations.add(transferDuration);
      transferSplitFractions.add(transferSplitFraction);

      int numberOfPointsInFoot = plannerParameters.getNumberOfCoPWayPointsPerFoot();
      int maxNumberOfFootstepsToConsider = plannerParameters.getNumberOfFootstepsToConsider();
      testCoPGenerator = new ReferenceCoPTrajectoryGenerator("TestCoPPlanner", maxNumberOfFootstepsToConsider, bipedSupportPolygons,
                                                             contactableFeet, numberOfFootstepsToConsider, swingDurations, transferDurations, touchdownDurations,
                                                             swingSplitFractions, swingDurationShiftFractions, transferSplitFractions, parentRegistry);
      testCoPGenerator.initializeParameters(plannerParameters);
      assertTrue("Object not initialized", testCoPGenerator != null);
   }

   @After
   public void clearAllVariables()
   {
      parentRegistry.clear();
      testCoPGenerator = null;
      soleZUpFrames.clear();
      ankleZUpFrames.clear();
      contactableFeet.clear();
      feetBodies.clear();
      midFeetZUpFrame = null;
      contactStates.clear();
      plannerParameters = null;
   }

   public void sendFootStepMessages(int numberOfFootstepsToPlan)
   {
      RobotSide robotSide = RobotSide.LEFT;
      FramePoint3D footstepLocation = new FramePoint3D();
      FrameQuaternion footstepOrientation = new FrameQuaternion();
      for (int i = 1; i < numberOfFootstepsToPlan + 1; i++)
      {
         Footstep footstep = new Footstep(robotSide);
         footstepLocation.set(i * stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);
         footstep.setPose(footstepLocation, footstepOrientation);
         FootstepTiming timing = new FootstepTiming(swingTime, transferTime);
         testCoPGenerator.addFootstepToPlan(footstep, timing);
         robotSide = robotSide.getOppositeSide();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDoubleSupportFootstepPlanFromRest()
   {
      int numberOfFootsteps = 3;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepsRegistered() == numberOfFootsteps);
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(true, RobotSide.RIGHT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();

      CoPPointName exitCoPName = plannerParameters.getExitCoPName();
      //  Check first step
      CoPPointsInFoot zeroStep = copList.get(0);
      {
         CoPTrajectoryPoint startCoP = zeroStep.get(0);
         assertTrue(startCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, 0.0), EPSILON));
         assertTrue(zeroStep.getCoPPointList().get(0) == CoPPointName.START_COP);
      }

      CoPPointsInFoot firstStep = copList.get(1);
      {
         CoPTrajectoryPoint midStanceCoP = firstStep.get(0);
         assertTrue(midStanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, 0.0), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = firstStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, -0.205), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = firstStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0375, -0.190), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = firstStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.06, -0.180), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = firstStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.06, -0.180), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check second step
      CoPPointsInFoot secondStep = copList.get(2);
      {
         CoPTrajectoryPoint midstanceCoP = secondStep.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.150, 0.0), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = secondStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.260, 0.205), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = secondStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.3375, 0.190), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = secondStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.360, 0.180), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = secondStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.360, 0.180), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check third step
      CoPPointsInFoot thirdStep = copList.get(3);
      {
         CoPTrajectoryPoint midstanceCoP = thirdStep.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.450, 0.0), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = thirdStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.560, -0.205), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = thirdStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.6375, -0.190), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = thirdStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.660, -0.180), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = thirdStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.660,-0.180), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check final transfer
      CoPPointsInFoot finalTransfer = copList.get(4);
      {
         CoPTrajectoryPoint midstanceCoP = finalTransfer.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.750, 0.0), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
      }

      testCoPGenerator.clear();
      assertTrue("Planned footsteps not removed", testCoPGenerator.getNumberOfFootstepsRegistered() == 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDoubleSupportFootstepPlanMoving()
   {
      sendFootStepMessages(10);
      //testCoPGenerator.setInitialCoPPosition(new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.1));
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(false, RobotSide.RIGHT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();
      CoPPointName exitCoPName = plannerParameters.getExitCoPName();

      CoPPointsInFoot zeroStep = copList.get(0);
      {
         CoPTrajectoryPoint startCoP = zeroStep.get(0);
         assertTrue(startCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, 0.180), EPSILON));
         assertTrue(zeroStep.getCoPPointList().get(0) == exitCoPName);
      }

      // check first step
      CoPPointsInFoot firstStep = copList.get(1);
      {
         CoPTrajectoryPoint midStanceCoP = firstStep.get(0);
         assertTrue(midStanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, 0.0), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = firstStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, -0.205), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = firstStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0375, -0.190), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = firstStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.06, -0.180), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = firstStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.06, -0.180), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check second step
      CoPPointsInFoot secondStep = copList.get(2);
      {
         CoPTrajectoryPoint midstanceCoP = secondStep.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.150, 0.0), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = secondStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.260, 0.205), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = secondStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.3375, 0.190), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = secondStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.360, 0.180), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = secondStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.360, 0.180), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check third step
      CoPPointsInFoot thirdStep = copList.get(3);
      {
         CoPTrajectoryPoint midstanceCoP = thirdStep.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.450, 0.0), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = thirdStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.560, -0.205), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = thirdStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.6375, -0.190), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = thirdStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.660, -0.180), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = thirdStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.660,-0.180), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check final transfer
      CoPPointsInFoot finalTransfer = copList.get(4);
      {
         CoPTrajectoryPoint midstanceCoP = finalTransfer.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.750, 0.0), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSingleSupportFootstepPlan()
   {
      int numberOfFootsteps = 10;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepsRegistered() == numberOfFootsteps);
      //testCoPGenerator.setInitialCoPPosition(initialCoPPosition);
      testCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(RobotSide.RIGHT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();
      CoPPointName exitCoPName = plannerParameters.getExitCoPName();

      CoPPointsInFoot zeroStep = copList.get(0);
      {
         CoPTrajectoryPoint startCoP = zeroStep.get(0);
         assertTrue(startCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, 0.180), EPSILON));
         assertTrue(zeroStep.getCoPPointList().get(0) == exitCoPName);
      }

      // check first step
      CoPPointsInFoot firstStep = copList.get(1);
      {
         CoPTrajectoryPoint midstanceCoP = firstStep.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, 0.0), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = firstStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0, -0.205), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = firstStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.0375, -0.190), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = firstStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.06, -0.180), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = firstStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.06, -0.180), EPSILON));
         assertTrue(firstStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check second step
      CoPPointsInFoot secondStep = copList.get(2);
      {
         CoPTrajectoryPoint midstanceCoP = secondStep.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.150, 0.0), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = secondStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.260, 0.205), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = secondStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.3375, 0.190), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = secondStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.360, 0.180), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = secondStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.360, 0.180), EPSILON));
         assertTrue(secondStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check third step
      CoPPointsInFoot thirdStep = copList.get(3);
      {
         CoPTrajectoryPoint midstanceCoP = thirdStep.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.450, 0.0), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         CoPTrajectoryPoint entryCoP = thirdStep.get(1);
         assertTrue(entryCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.560, -0.205), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(1) == CoPPointName.HEEL_COP);
         CoPTrajectoryPoint ballCoP = thirdStep.get(2);
         assertTrue(ballCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.6375, -0.190), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(2) == CoPPointName.BALL_COP);
         CoPTrajectoryPoint toeCoP = thirdStep.get(3);
         assertTrue(toeCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.660, -0.180), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(3) == CoPPointName.TOE_COP);
         CoPTrajectoryPoint exitCoP = thirdStep.get(4);
         assertTrue(exitCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.660,-0.180), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check final transfer
      CoPPointsInFoot finalTransfer = copList.get(4);
      {
         CoPTrajectoryPoint midstanceCoP = finalTransfer.get(0);
         assertTrue(midstanceCoP.epsilonEquals(new FramePoint2D(worldFrame, 0.750, 0.0), EPSILON));
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
      }
   }

   private class TestSmoothCMPPlannerParameters extends SmoothCMPPlannerParameters
   {
      public TestSmoothCMPPlannerParameters()
      {
         super();
         endCoPName = CoPPointName.MIDFEET_COP;
         entryCoPName = CoPPointName.HEEL_COP;
         exitCoPName = CoPPointName.TOE_COP;
         swingCopPointsToPlan = new CoPPointName[]{CoPPointName.BALL_COP, CoPPointName.TOE_COP};
         transferCoPPointsToPlan = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.HEEL_COP};
         copOffsetFrameNames.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
         copOffsetFrameNames.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
         copOffsetFrameNames.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.SUPPORT_FOOT_POLYGON);
         copOffsetFrameNames.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.INITIAL_DOUBLE_SUPPORT_POLYGON);

         stepLengthOffsetPolygon.put(CoPPointName.MIDFEET_COP, CoPSupportPolygonNames.NULL);
         stepLengthOffsetPolygon.put(CoPPointName.HEEL_COP, CoPSupportPolygonNames.INITIAL_SWING_POLYGON);
         stepLengthOffsetPolygon.put(CoPPointName.BALL_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);
         stepLengthOffsetPolygon.put(CoPPointName.TOE_COP, CoPSupportPolygonNames.FINAL_SWING_POLYGON);

         constrainToMinMax.put(CoPPointName.MIDFEET_COP, false);
         constrainToMinMax.put(CoPPointName.HEEL_COP, true);
         constrainToMinMax.put(CoPPointName.BALL_COP, true);
         constrainToMinMax.put(CoPPointName.TOE_COP, true);

         constrainToSupportPolygon.put(CoPPointName.MIDFEET_COP, false);
         constrainToSupportPolygon.put(CoPPointName.HEEL_COP, true);
         constrainToSupportPolygon.put(CoPPointName.BALL_COP, true);
         constrainToSupportPolygon.put(CoPPointName.TOE_COP, true);

         stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.HEEL_COP, 1.0 / 3.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.BALL_COP, 1.0 / 8.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.TOE_COP, 1.0 / 3.0);

         copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
         copOffsetsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(0.0, -0.005));
         copOffsetsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.01));
         copOffsetsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.025));

         copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
         copOffsetBoundsInFootFrame.put(CoPPointName.HEEL_COP, new Vector2D(-0.04, 0.03));
         copOffsetBoundsInFootFrame.put(CoPPointName.BALL_COP, new Vector2D(0.0, 0.055));
         copOffsetBoundsInFootFrame.put(CoPPointName.TOE_COP, new Vector2D(0.0, 0.08));
      }
   }
}
