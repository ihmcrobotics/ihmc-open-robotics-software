package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class CoMTrajectoryPlannerTest
{
   @Test
   public void testOneSegment()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint2D comPosition = new FramePoint2D();
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), 1e-5);
   }

   @Test
   public void testTwoSegments()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();
      QuadrupedContactPhase thirdContact= new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0));
      thirdContact.setTimeInterval(new TimeInterval(2.0, 3.0));
      thirdContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint2D comPosition = new FramePoint2D();
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), 1e-5);
   }
}
