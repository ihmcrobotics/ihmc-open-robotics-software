package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.DummyCoPParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.ReferenceCenterOfPressureTrajectoryCalculator;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class ReferenceCenterOfPressureTrajectoryCalculatorTest
{
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
   private final double EPSILON = 10e-5;

   ReferenceCenterOfPressureTrajectoryCalculator testCoPGenerator;
   SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   SideDependentList<ContactableFoot> contactableFeet = new SideDependentList<>();
   SideDependentList<RigidBody> feetBodies = new SideDependentList<>();
   YoVariableRegistry parentRegistry = new YoVariableRegistry("TestRegistry");
   MidFootZUpGroundFrame midFeetZUpFrame;
   SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   DummyCoPParameters icpPlannerParameters;
   YoInteger numberOfFootstepsToConsider = new YoInteger("numberOfFootstepsToConsider", parentRegistry);

   @Before
   public void setUp()
   {
      for (RobotSide side : RobotSide.values)
      {
         ZUpFrame soleFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(),
                                           new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, side.negateIfRightSide(soleFrameYDisplacement), 0.0),
                                           "DummyRobot" + side.toString() + "FootSoleFrame");
         soleZUpFrames.put(side, soleFrame);
         ZUpFrame ankleFrame = new ZUpFrame(soleFrame, new FramePoint(soleFrame, 0.0, 0.0, ankleFrameZDisplacement),
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
         ContactableFoot contactableFoot = new ListOfPointsContactableFoot(null, (ReferenceFrame) soleFrame, contactPoints, point1,
                                                                           new LineSegment2D(point1, point2));
         contactableFeet.put(side, contactableFoot);
         RigidBody feetBody = new RigidBody(side.toString() + "Feet", ReferenceFrame.getWorldFrame());
         feetBodies.put(side, feetBody);
         List<FramePoint2d> contactFramePoints = new ArrayList<>();
         contactFramePoints.add(new FramePoint2d(soleFrame, point1));
         contactFramePoints.add(new FramePoint2d(soleFrame, point2));
         contactFramePoints.add(new FramePoint2d(soleFrame, point3));
         contactFramePoints.add(new FramePoint2d(soleFrame, point4));

         YoPlaneContactState contactState = new YoPlaneContactState("DummyRobot" + side.toString() + "FootContactState", feetBody, soleFrame,
                                                                    contactFramePoints, 10, parentRegistry);
         contactStates.put(side, contactState);
      }
      midFeetZUpFrame = new MidFootZUpGroundFrame("DummyRobotMidFootZUpFrame", soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, parentRegistry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      icpPlannerParameters = new DummyCoPParameters();
      numberOfFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());
      testCoPGenerator = new ReferenceCenterOfPressureTrajectoryCalculator("TestCoPPlanClass", icpPlannerParameters, bipedSupportPolygons,
                                                                           contactableFeet, numberOfFootstepsToConsider, parentRegistry);
      assertTrue("Object not initialized", testCoPGenerator != null);
   }

   @After
   public void clearAllVariables()
   {
      testCoPGenerator = null;
      soleZUpFrames.clear();
      ankleZUpFrames.clear();
      contactableFeet.clear();
      feetBodies.clear();
      midFeetZUpFrame = null;
      contactStates.clear();
      icpPlannerParameters = null;
   }
   
   public void sendFootStepMessages(int numberOfFootstepsToPlan)
   {
      RobotSide robotSide = RobotSide.LEFT;
      FramePoint footstepLocation = new FramePoint();
      FrameOrientation footstepOrientation = new FrameOrientation();
      for (int i = 1; i < numberOfFootstepsToPlan + 1; i++)
      {
         Footstep footstep = new Footstep(feetBodies.get(robotSide), robotSide);
         footstepLocation.set(i * stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);
         footstep.setPose(footstepLocation, footstepOrientation);
         FootstepTiming timing = new FootstepTiming(swingTime, transferTime);
         testCoPGenerator.addFootstepToPlan(footstep, timing);
         robotSide = robotSide.getOppositeSide();
      }
   }

   @Test
   public void testDoubleSupportFootstepPlan()
   {
      int numberOfFootsteps = 3;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepRegistered() == numberOfFootsteps);
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(true);
      List<FramePoint> coPList = testCoPGenerator.getWaypoints();
      assertTrue("Incorrect number of CoP way points generated", coPList.size() == numberOfFootsteps * 3 + 2);
      assertTrue(coPList.get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0), EPSILON));
      assertTrue(coPList.get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), -0.05, -0.2), EPSILON));
      assertTrue(coPList.get(2).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, -0.2), EPSILON));
      assertTrue(coPList.get(3).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.06, -0.2), EPSILON));
      assertTrue(coPList.get(4).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.25, 0.2), EPSILON));
      assertTrue(coPList.get(5).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.3, 0.2), EPSILON));
      assertTrue(coPList.get(6).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.36, 0.2), EPSILON));
      assertTrue(coPList.get(7).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.55, -0.2), EPSILON));
      assertTrue(coPList.get(8).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.6, -0.2), EPSILON));
      assertTrue(coPList.get(9).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.66, -0.2), EPSILON));
      assertTrue(coPList.get(10).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.85, 0.1), EPSILON));
      testCoPGenerator.clearPlan();
      assertTrue("Plan not cleared", testCoPGenerator.getWaypoints().isEmpty());
      assertTrue("Footsteps cleared on clearing plan", testCoPGenerator.getNumberOfFootstepRegistered() == 3);
      testCoPGenerator.clear();
      assertTrue("Planned footsteps not removed", testCoPGenerator.getNumberOfFootstepRegistered() == 0);

      sendFootStepMessages(10);
      testCoPGenerator.setInitialCoPPosition(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.1));
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(false);
      coPList = testCoPGenerator.getWaypoints();
      assertTrue("Incorrect number of CoP way points generated", coPList.size() == 3 * 3 + 1);
      assertTrue(coPList.get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.1), EPSILON));
      assertTrue(coPList.get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), -0.05, -0.2), EPSILON));
      assertTrue(coPList.get(2).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, -0.2), EPSILON));
      assertTrue(coPList.get(3).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.06, -0.2), EPSILON));
      assertTrue(coPList.get(4).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.25, 0.2), EPSILON));
      assertTrue(coPList.get(5).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.3, 0.2), EPSILON));
      assertTrue(coPList.get(6).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.36, 0.2), EPSILON));
      assertTrue(coPList.get(7).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.55, -0.2), EPSILON));
      assertTrue(coPList.get(8).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.6, -0.2), EPSILON));
      assertTrue(coPList.get(9).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.66, -0.2), EPSILON));
   }

   @Test
   public void testSingleSupportFootstepPlan()
   {
      int numberOfFootsteps = 10;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepRegistered() == numberOfFootsteps);
      FramePoint2d initialCoPPosition = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.2);
      testCoPGenerator.setInitialCoPPosition(initialCoPPosition);
      testCoPGenerator.computeReferenceCoPsStartingFromSingleSupport();
      List<FramePoint> coPList = testCoPGenerator.getWaypoints();
      assertTrue("Incorrect number of CoP way points generated", coPList.size() == 3 * 3);
      assertTrue(coPList.get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.2), EPSILON));
      assertTrue(coPList.get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, -0.2), EPSILON));
      assertTrue(coPList.get(2).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.06, -0.2), EPSILON));
      assertTrue(coPList.get(3).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.25, 0.2), EPSILON));
      assertTrue(coPList.get(4).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.3, 0.2), EPSILON));
      assertTrue(coPList.get(5).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.36, 0.2), EPSILON));
      assertTrue(coPList.get(6).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.55, -0.2), EPSILON));
      assertTrue(coPList.get(7).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.6, -0.2), EPSILON));
      assertTrue(coPList.get(8).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.66, -0.2), EPSILON));

      testCoPGenerator.clearPlan();
      coPList = testCoPGenerator.getWaypoints();
      assertTrue("Unable to clear plan", coPList.isEmpty());
      testCoPGenerator.setInitialCoPPosition(initialCoPPosition);
      testCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(0.6);
      coPList = testCoPGenerator.getWaypoints();
      assertTrue("Incorrect number of CoP way points generated", coPList.size() == 3 * 3 - 1);      
      assertTrue(coPList.get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.2), EPSILON));
      assertTrue(coPList.get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.06, -0.2), EPSILON));
      assertTrue(coPList.get(2).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.25, 0.2), EPSILON));
      assertTrue(coPList.get(3).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.3, 0.2), EPSILON));
      assertTrue(coPList.get(4).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.36, 0.2), EPSILON));
      assertTrue(coPList.get(5).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.55, -0.2), EPSILON));
      assertTrue(coPList.get(6).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.6, -0.2), EPSILON));
      assertTrue(coPList.get(7).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.66, -0.2), EPSILON));
   }
   
   @Test
   public void testTransferFootstepPlan()
   {
      testCoPGenerator.computeReferenceCoPsForTransfer(RobotSide.LEFT);
      List<FramePoint> coPList = testCoPGenerator.getWaypoints();
      assertTrue("Incorrect number of CoP way points generated", coPList.size() == 2);
      assertTrue(coPList.get(0).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0), EPSILON));
      assertTrue(coPList.get(1).epsilonEquals(new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.2), EPSILON));      
   }
}
