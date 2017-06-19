package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.configurations.DummyExtendedCapturePointPlannerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ReferenceCenterOfPressureTrajectoryCalculatorTest
{
   private final int numberOfContactPoints = 4;
   private final double soleFrameYDisplacement = 0.2;
   private final double ankleFrameZDisplacement = 0.05;
   private final double frontContactX = 0.07;
   private final double rearContactX = -0.07;
   private final double side1ContactY = 0.03;
   private final double side2ContactY = -0.03;
   
   ReferenceCenterOfPressureTrajectoryCalculator testCoPGenerator;
   
   public void setUp()
   {
      SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
      SideDependentList<ContactableFoot> contactableFeet = new SideDependentList<>();
      
      for (RobotSide side : RobotSide.values)
      {
         ZUpFrame soleFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), new FramePoint(ReferenceFrame.getWorldFrame(), 0.0, side.negateIfRightSide(soleFrameYDisplacement), 0.0),
                                           "DummyRobot" + side.toString() + "FootSoleFrame");
         soleZUpFrames.put(side, soleFrame);
         ZUpFrame ankleFrame = new ZUpFrame(soleFrame, new FramePoint(soleFrame, 0.0, 0.0, ankleFrameZDisplacement), "DummyRobot" + side.toString() + "AnkleSoleFrame");
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
         ContactableFoot contactableFoot = new ListOfPointsContactableFoot(null, (ReferenceFrame) soleFrame, contactPoints, point1, new LineSegment2D(point1, point2));
         contactableFeet.put(side, contactableFoot);
      }      
      MidFootZUpGroundFrame midFeetZUpFrame = new MidFootZUpGroundFrame("DummyRobotMidFootZUpFrame", soleZUpFrames.get(RobotSide.LEFT),
                                                                        soleZUpFrames.get(RobotSide.RIGHT));
      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, null, null);
      testCoPGenerator = new ReferenceCenterOfPressureTrajectoryCalculator("TestCoPPlanClass");
      assertTrue("Object not initialized", testCoPGenerator != null);
      DummyExtendedCapturePointPlannerParameters icpPlannerParameters = new DummyExtendedCapturePointPlannerParameters();
      testCoPGenerator.initializeParameters(icpPlannerParameters, bipedSupportPolygons, contactableFeet, null);
   }
      
   public void testCoPWayPointGeneration(int numberOfFootstepsToPlan)
   {
      setUp();      
   }   
   
   
   
}
