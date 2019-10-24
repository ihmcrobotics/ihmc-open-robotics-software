package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

public class FrameEuclideanTrajectoryPointListTest
{
   @Test
   public void testSetReferenceFrame()
   {
      FrameEuclideanTrajectoryPointList frameEuclideanTrajectoryPointList = new FrameEuclideanTrajectoryPointList();

      ReferenceFrame referenceFrameNotWorld = ReferenceFrameTools.constructARootFrame("testFrame");
      frameEuclideanTrajectoryPointList.setReferenceFrame(referenceFrameNotWorld);

      assertEquals(referenceFrameNotWorld, frameEuclideanTrajectoryPointList.getReferenceFrame());
   }

   @Test
   public void testSetIncludingFrame()
   {
      FrameEuclideanTrajectoryPointList frameEuclideanTrajectoryPointList = new FrameEuclideanTrajectoryPointList();

      ReferenceFrame referenceFrameNotWorld = ReferenceFrameTools.constructARootFrame("testFrame");
      frameEuclideanTrajectoryPointList.setReferenceFrame(referenceFrameNotWorld);

      FrameSE3TrajectoryPointList frameSE3TrajectoryPointList = new FrameSE3TrajectoryPointList();
      FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(ReferenceFrame.getWorldFrame());
      frameSE3TrajectoryPointList.addPositionTrajectoryPoint(frameEuclideanTrajectoryPoint);

      frameEuclideanTrajectoryPointList.setIncludingFrame(frameSE3TrajectoryPointList);

      assertThrows(ReferenceFrameMismatchException.class, () -> frameEuclideanTrajectoryPointList.getReferenceFrame().checkReferenceFrameMatch(referenceFrameNotWorld));
   }
}
