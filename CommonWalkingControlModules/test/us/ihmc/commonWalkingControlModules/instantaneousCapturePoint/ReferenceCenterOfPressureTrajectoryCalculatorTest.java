package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.DummyExtendedCapturePointPlannerParameters;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;

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
   
   ReferenceCenterOfPressureTrajectoryCalculator testCoPGenerator;
   SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   SideDependentList<ContactableFoot> contactableFeet = new SideDependentList<>();
   SideDependentList<RigidBody> feetBodies = new SideDependentList<>(); 
   YoVariableRegistry parentRegistry = new YoVariableRegistry("TestRegistry");
   MidFootZUpGroundFrame midFeetZUpFrame;
   SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   DummyExtendedCapturePointPlannerParameters icpPlannerParameters;
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
         
         YoPlaneContactState contactState = new YoPlaneContactState("DummyRobot" + side.toString() + "FootContactState", feetBody, soleFrame, contactFramePoints, 10, parentRegistry);
         contactStates.put(side, contactState);
      }
      midFeetZUpFrame = new MidFootZUpGroundFrame("DummyRobotMidFootZUpFrame", soleZUpFrames.get(RobotSide.LEFT),
                                                                        soleZUpFrames.get(RobotSide.RIGHT));
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, parentRegistry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);
      testCoPGenerator = new ReferenceCenterOfPressureTrajectoryCalculator("TestCoPPlanClass");
      assertTrue("Object not initialized", testCoPGenerator != null);
      icpPlannerParameters = new DummyExtendedCapturePointPlannerParameters();
      testCoPGenerator.initializeParameters(icpPlannerParameters, bipedSupportPolygons, contactableFeet, null);
   }
  
   public void sendFootStepMessages(int numberOfFootstepsToPlan)
   {
      RobotSide robotSide = RobotSide.LEFT;
      FramePoint footstepLocation = new FramePoint();
      FrameOrientation footstepOrientation = new FrameOrientation();
      for(int i = 1; i < numberOfFootstepsToPlan + 1; i++)
      {
         Footstep footstep = new Footstep(feetBodies.get(robotSide), robotSide);
         footstepLocation.set(i*stepLength, robotSide.negateIfRightSide(stepWidth), 0.0);
         footstep.setPose(footstepLocation, footstepOrientation);
         FootstepTiming timing = new FootstepTiming(swingTime, transferTime);         
         testCoPGenerator.addFootstepToPlan(footstep, timing);
         robotSide = robotSide.getOppositeSide();
      }
   }
   
   @Test
   public void testDoubleSupportFootstepPlan()   
   {
      setUp();
      int numberOfFootsteps = 3;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepRegistered() == numberOfFootsteps);
      testCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(true);
      testCoPGenerator.clearPlan();
      FramePoint2d initialCoPPosition = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      testCoPGenerator.setInitialCoPPosition(initialCoPPosition);
      testCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(0.0);
      List<FramePoint> coPList = testCoPGenerator.getCoPs();
      for (int i = 0; i < coPList.size(); i++)
      {
         System.out.println(coPList.get(i).toString());
      }
   }

   @Test
   public void testSingleSupportFootstepPlan()   
   {
      setUp();
      int numberOfFootsteps = 10;
      sendFootStepMessages(numberOfFootsteps);
      assertTrue("Footstep registration error", testCoPGenerator.getNumberOfFootstepRegistered() == numberOfFootsteps);
      FramePoint2d initialCoPPosition = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      testCoPGenerator.setInitialCoPPosition(initialCoPPosition);
      testCoPGenerator.computeReferenceCoPsStartingFromSingleSupport();
      List<FramePoint> coPList = testCoPGenerator.getCoPs();
      for (int i = 0; i < coPList.size(); i++)
      {
         System.out.println(coPList.get(i).toString());
      }
   }


}
