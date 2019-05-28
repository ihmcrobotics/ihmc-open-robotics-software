package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

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

   private ReferenceCoPTrajectoryGenerator testCoPGenerator;
   private MidFootZUpGroundFrame midFeetZUpFrame;

   private final SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private final SideDependentList<ContactableFoot> contactableFeet = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> feetBodies = new SideDependentList<>();
   private final YoVariableRegistry parentRegistry = new YoVariableRegistry("TestRegistry");
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   private final SmoothCMPPlannerParameters plannerParameters = new TestSmoothCMPPlannerParameters();;
   private final YoInteger numberOfFootstepsToConsider = new YoInteger("numberOfFootstepsToConsider", parentRegistry);
   private final YoInteger numberOfUpcomingFootsteps = new YoInteger("NumberOfUpcomingFootsteps", parentRegistry);
   private final ArrayList<YoDouble> swingDurations = new ArrayList<>();
   private final ArrayList<YoDouble> swingSplitFractions = new ArrayList<>();
   private final ArrayList<YoDouble> swingDurationShiftFractions = new ArrayList<>();
   private final ArrayList<YoDouble> transferDurations = new ArrayList<>();
   private final ArrayList<YoDouble> transferSplitFractions = new ArrayList<>();
   private final ArrayList<FootstepData> upcomingFootstepsData = new ArrayList<>();

   @BeforeEach
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
         RigidBodyBasics feetBody = new RigidBody(side.toString() + "Feet", ReferenceFrame.getWorldFrame());
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
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleZUpFrames, parentRegistry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      numberOfFootstepsToConsider.set(plannerParameters.getNumberOfFootstepsToConsider());

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         YoDouble swingDuration = new YoDouble("swingDuration" + i, parentRegistry);
         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, parentRegistry);
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, parentRegistry);
         YoDouble transferDuration = new YoDouble("transferDuration" + i, parentRegistry);
         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, parentRegistry);

         swingDurations.add(swingDuration);
         swingSplitFractions.add(swingSplitFraction);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
         transferDurations.add(transferDuration);
         transferSplitFractions.add(transferSplitFraction);
      }
      YoDouble transferDuration = new YoDouble("transferDuration" + numberOfFootstepsToConsider.getIntegerValue(), parentRegistry);
      YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + numberOfFootstepsToConsider.getIntegerValue(), parentRegistry);
      transferDurations.add(transferDuration);
      transferSplitFractions.add(transferSplitFraction);

      int numberOfPointsInFoot = plannerParameters.getNumberOfCoPWayPointsPerFoot();
      int maxNumberOfFootstepsToConsider = plannerParameters.getNumberOfFootstepsToConsider();
      testCoPGenerator = new ReferenceCoPTrajectoryGenerator("TestCoPPlanner", maxNumberOfFootstepsToConsider, bipedSupportPolygons, contactableFeet,
                                                             numberOfFootstepsToConsider, swingDurations, transferDurations, swingSplitFractions,
                                                             swingDurationShiftFractions, transferSplitFractions, numberOfUpcomingFootsteps,
                                                             upcomingFootstepsData, soleZUpFrames, parentRegistry);
      testCoPGenerator.initializeParameters(plannerParameters);
      assertTrue("Object not initialized", testCoPGenerator != null);
   }

   @AfterEach
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
      numberOfUpcomingFootsteps.set(0);
      upcomingFootstepsData.clear();
      ReferenceFrameTools.clearWorldFrameTree();
   }

   public void sendFootStepMessages(int numberOfFootstepsToPlan)
   {
      RobotSide robotSide = RobotSide.LEFT;
      FramePoint3D footstepLocation = new FramePoint3D();
      FrameQuaternion footstepOrientation = new FrameQuaternion();
      upcomingFootstepsData.clear();

      for (int i = 1; i < numberOfFootstepsToPlan + 1; i++)
      {
         Footstep footstep = new Footstep(robotSide);
         footstepLocation.set(i * stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);
         footstep.setPose(footstepLocation, footstepOrientation);
         FootstepTiming timing = new FootstepTiming(swingTime, transferTime);
         upcomingFootstepsData.add(new FootstepData(footstep, timing));
         robotSide = robotSide.getOppositeSide();
      }
      numberOfUpcomingFootsteps.set(upcomingFootstepsData.size());
   }

   @Test
   public void testDoubleSupportFootstepPlanFromRest()
   {
      int numberOfFootsteps = 3;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepsRegistered() == numberOfFootsteps);
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(true, RobotSide.RIGHT, null);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();

      CoPPointName exitCoPName = plannerParameters.getExitCoPName();
      //  Check first step
      CoPPointsInFoot zeroStep = copList.get(0);
      {
         YoFrameEuclideanTrajectoryPoint startCoP = zeroStep.get(0);
         assertEquals(new FramePoint3D(worldFrame, 0.0, 0.0, 0.0), startCoP, EPSILON);
         assertTrue(zeroStep.getCoPPointList().get(0) == CoPPointName.START_COP);
      }

      CoPPointsInFoot firstStep = copList.get(1);
      {
         YoFrameEuclideanTrajectoryPoint midStanceCoP = firstStep.get(0);
         assertEquals(new FramePoint3D(worldFrame, 0.0, 0.0, 0.0), midStanceCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = firstStep.get(1);
         assertEquals(new FramePoint3D(worldFrame, 0.0, -0.205, 0.0), entryCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = firstStep.get(2);
         assertEquals(new FramePoint3D(worldFrame, 0.0375, -0.190, 0.0), ballCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = firstStep.get(3);
         assertEquals(new FramePoint3D(worldFrame, 0.06, -0.180, 0.0), toeCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = firstStep.get(4);
         assertEquals(new FramePoint3D(worldFrame, 0.06, -0.180, 0.0), exitCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check second step
      CoPPointsInFoot secondStep = copList.get(2);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = secondStep.get(0);
         assertEquals(new FramePoint3D(worldFrame, 0.150, 0.0, 0.0), midstanceCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = secondStep.get(1);
         assertEquals(new FramePoint3D(worldFrame, 0.260, 0.205, 0.0), entryCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = secondStep.get(2);
         assertEquals(new FramePoint3D(worldFrame, 0.3375, 0.190, 0.0), ballCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = secondStep.get(3);
         assertEquals(new FramePoint3D(worldFrame, 0.360, 0.180, 0.0), toeCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = secondStep.get(4);
         assertEquals(new FramePoint3D(worldFrame, 0.360, 0.180, 0.0), exitCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check third step
      CoPPointsInFoot thirdStep = copList.get(3);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = thirdStep.get(0);
         assertEquals(new FramePoint3D(worldFrame, 0.450, 0.0, 0.0), midstanceCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = thirdStep.get(1);
         assertEquals(new FramePoint3D(worldFrame, 0.560, -0.205, 0.0), entryCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = thirdStep.get(2);
         assertEquals(new FramePoint3D(worldFrame, 0.6375, -0.190, 0.0), ballCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = thirdStep.get(3);
         assertEquals(new FramePoint3D(worldFrame, 0.660, -0.180, 0.0), toeCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = thirdStep.get(4);
         assertEquals(new FramePoint3D(worldFrame, 0.660,-0.180, 0.0), exitCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check final transfer
      CoPPointsInFoot finalTransfer = copList.get(4);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = finalTransfer.get(0);
         assertEquals(new FramePoint3D(worldFrame, 0.750, 0.0, 0.0), midstanceCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
      }

      testCoPGenerator.clear();
      upcomingFootstepsData.clear();
      numberOfUpcomingFootsteps.set(0);
      assertTrue("Planned footsteps not removed", testCoPGenerator.getNumberOfFootstepsRegistered() == 0);
   }

   @Test
   public void testDoubleSupportFootstepPlanMoving()
   {
      sendFootStepMessages(10);
      //testCoPGenerator.setInitialCoPPosition(new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.1));
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(false, RobotSide.RIGHT, RobotSide.LEFT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();
      CoPPointName exitCoPName = plannerParameters.getExitCoPName();

      CoPPointsInFoot zeroStep = copList.get(0);
      {
         YoFrameEuclideanTrajectoryPoint startCoP = zeroStep.get(0);
         assertEquals(new FramePoint3D(worldFrame, 0.0, 0.180, 0.0), startCoP, EPSILON);
         assertTrue(zeroStep.getCoPPointList().get(0) == exitCoPName);
      }

      // check first step
      CoPPointsInFoot firstStep = copList.get(1);
      {
         YoFrameEuclideanTrajectoryPoint midStanceCoP = firstStep.get(0);
         assertEquals(new FramePoint3D(worldFrame, 0.0, 0.0, 0.0), midStanceCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = firstStep.get(1);
         assertEquals(new FramePoint2D(worldFrame, 0.0, -0.205), entryCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = firstStep.get(2);
         assertEquals(new FramePoint2D(worldFrame, 0.0375, -0.190), ballCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = firstStep.get(3);
         assertEquals(new FramePoint2D(worldFrame, 0.06, -0.180), toeCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = firstStep.get(4);
         assertEquals(new FramePoint2D(worldFrame, 0.06, -0.180), exitCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check second step
      CoPPointsInFoot secondStep = copList.get(2);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = secondStep.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.150, 0.0), midstanceCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = secondStep.get(1);
         assertEquals(new FramePoint2D(worldFrame, 0.260, 0.205), entryCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = secondStep.get(2);
         assertEquals(new FramePoint2D(worldFrame, 0.3375, 0.190), ballCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = secondStep.get(3);
         assertEquals(new FramePoint2D(worldFrame, 0.360, 0.180), toeCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = secondStep.get(4);
         assertEquals(new FramePoint2D(worldFrame, 0.360, 0.180), exitCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check third step
      CoPPointsInFoot thirdStep = copList.get(3);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = thirdStep.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.450, 0.0), midstanceCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = thirdStep.get(1);
         assertEquals(new FramePoint3D(worldFrame, 0.560, -0.205, 0.0), entryCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = thirdStep.get(2);
         assertEquals(new FramePoint2D(worldFrame, 0.6375, -0.190), ballCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = thirdStep.get(3);
         assertEquals(new FramePoint2D(worldFrame, 0.660, -0.180), toeCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = thirdStep.get(4);
         assertEquals(new FramePoint2D(worldFrame, 0.660,-0.180), exitCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check final transfer
      CoPPointsInFoot finalTransfer = copList.get(4);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = finalTransfer.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.750, 0.0), midstanceCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
      }
   }

   @Test
   public void testSingleSupportFootstepPlan()
   {
      int numberOfFootsteps = 10;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepsRegistered() == numberOfFootsteps);
      //testCoPGenerator.setInitialCoPPosition(initialCoPPosition);
      testCoPGenerator.initializeForSwing();
      testCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(RobotSide.RIGHT);
      List<CoPPointsInFoot> copList = testCoPGenerator.getWaypoints();
      CoPPointName exitCoPName = plannerParameters.getExitCoPName();

      CoPPointsInFoot zeroStep = copList.get(0);
      {
         YoFrameEuclideanTrajectoryPoint startCoP = zeroStep.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.0, 0.180), startCoP, EPSILON);
         assertTrue(zeroStep.getCoPPointList().get(0) == exitCoPName);
      }

      // check first step
      CoPPointsInFoot firstStep = copList.get(1);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = firstStep.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.0, 0.0), midstanceCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = firstStep.get(1);
         assertEquals(new FramePoint2D(worldFrame, 0.0, -0.205), entryCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = firstStep.get(2);
         assertEquals(new FramePoint2D(worldFrame, 0.0375, -0.190), ballCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = firstStep.get(3);
         assertEquals(new FramePoint2D(worldFrame, 0.06, -0.180), toeCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = firstStep.get(4);
         assertEquals(new FramePoint2D(worldFrame, 0.06, -0.180), exitCoP, EPSILON);
         assertTrue(firstStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check second step
      CoPPointsInFoot secondStep = copList.get(2);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = secondStep.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.150, 0.0), midstanceCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = secondStep.get(1);
         assertEquals(new FramePoint2D(worldFrame, 0.260, 0.205), entryCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = secondStep.get(2);
         assertEquals(new FramePoint2D(worldFrame, 0.3375, 0.190), ballCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = secondStep.get(3);
         assertEquals(new FramePoint2D(worldFrame, 0.360, 0.180), toeCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = secondStep.get(4);
         assertEquals(new FramePoint2D(worldFrame, 0.360, 0.180), exitCoP, EPSILON);
         assertTrue(secondStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check third step
      CoPPointsInFoot thirdStep = copList.get(3);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = thirdStep.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.450, 0.0), midstanceCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
         YoFrameEuclideanTrajectoryPoint entryCoP = thirdStep.get(1);
         assertEquals(new FramePoint2D(worldFrame, 0.560, -0.205), entryCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(1) == CoPPointName.ENTRY_COP);
         YoFrameEuclideanTrajectoryPoint ballCoP = thirdStep.get(2);
         assertEquals(new FramePoint2D(worldFrame, 0.6375, -0.190), ballCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(2) == CoPPointName.MIDFOOT_COP);
         YoFrameEuclideanTrajectoryPoint toeCoP = thirdStep.get(3);
         assertEquals(new FramePoint2D(worldFrame, 0.660, -0.180), toeCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(3) == CoPPointName.EXIT_COP);
         YoFrameEuclideanTrajectoryPoint exitCoP = thirdStep.get(4);
         assertEquals(new FramePoint2D(worldFrame, 0.660,-0.180), exitCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(4) == exitCoPName);
      }

      //  Check final transfer
      CoPPointsInFoot finalTransfer = copList.get(4);
      {
         YoFrameEuclideanTrajectoryPoint midstanceCoP = finalTransfer.get(0);
         assertEquals(new FramePoint2D(worldFrame, 0.750, 0.0), midstanceCoP, EPSILON);
         assertTrue(thirdStep.getCoPPointList().get(0) == CoPPointName.MIDFEET_COP);
      }
   }

   private class TestSmoothCMPPlannerParameters extends SmoothCMPPlannerParameters
   {
      public TestSmoothCMPPlannerParameters()
      {
         super();
         endCoPName = CoPPointName.MIDFEET_COP;
         entryCoPName = CoPPointName.ENTRY_COP;
         exitCoPName = CoPPointName.EXIT_COP;
         swingCopPointsToPlan = new CoPPointName[]{CoPPointName.MIDFOOT_COP, CoPPointName.EXIT_COP};
         transferCoPPointsToPlan = new CoPPointName[]{CoPPointName.MIDFEET_COP, CoPPointName.ENTRY_COP};

         stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFEET_COP, 0.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.ENTRY_COP, 1.0 / 3.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.MIDFOOT_COP, 1.0 / 8.0);
         stepLengthToCoPOffsetFactor.put(CoPPointName.EXIT_COP, 1.0 / 3.0);

         copOffsetsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(0.0, 0.0));
         copOffsetsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(0.0, -0.005));
         copOffsetsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.01));
         copOffsetsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.025));

         copOffsetBoundsInFootFrame.put(CoPPointName.MIDFEET_COP, new Vector2D(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
         copOffsetBoundsInFootFrame.put(CoPPointName.ENTRY_COP, new Vector2D(-0.04, 0.03));
         copOffsetBoundsInFootFrame.put(CoPPointName.MIDFOOT_COP, new Vector2D(0.0, 0.055));
         copOffsetBoundsInFootFrame.put(CoPPointName.EXIT_COP, new Vector2D(0.0, 0.08));
      }
   }

   private static void assertEquals(YoFrameEuclideanTrajectoryPoint pointA, YoFrameEuclideanTrajectoryPoint pointB, double epsilon)
   {
      pointA.checkReferenceFrameMatch(pointB);
      Assert.assertEquals(pointA.getTime(), pointB.getTime(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(pointA.getPosition(), pointB.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(pointA.getLinearVelocity(), pointB.getLinearVelocity(), epsilon);
   }

   private static void assertEquals(FramePoint2DReadOnly pointA, YoFrameEuclideanTrajectoryPoint pointB, double epsilon)
   {
      assertEquals(new FramePoint3D(pointA.getReferenceFrame(), pointA.getX(), pointA.getY(), 0.0), pointB, epsilon);
   }

   private static void assertEquals(FramePoint3DReadOnly pointA, YoFrameEuclideanTrajectoryPoint pointB, double epsilon)
   {
      pointA.checkReferenceFrameMatch(pointB);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(pointA, pointB.getPosition(), epsilon);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(new FrameVector3D(pointB.getReferenceFrame()), pointB.getLinearVelocity(), epsilon);
   }
}
