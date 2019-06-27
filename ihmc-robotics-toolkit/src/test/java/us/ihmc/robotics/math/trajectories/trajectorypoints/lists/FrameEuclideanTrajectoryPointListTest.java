package us.ihmc.robotics.math.trajectories.trajectorypoints.lists;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.generators.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;

public class FrameEuclideanTrajectoryPointListTest
{
    @Test
    public void testChangeReferenceFrame()
    {
        FrameEuclideanTrajectoryPointList frameEuclideanTrajectoryPointList = new FrameEuclideanTrajectoryPointList();

        ReferenceFrame referenceFrameNotWorld = new ReferenceFrame("testFrame") {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {

            }
        };

        frameEuclideanTrajectoryPointList.setReferenceFrame(referenceFrameNotWorld);


        try
        {
            frameEuclideanTrajectoryPointList.addTrajectoryPoint(0.0, new Point3D(0.0, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0));
            assert(true);
        }
        catch(ReferenceFrameMismatchException e)
        {
            assert(false);
        }
    }

    @Test
    public void testSetIncludingFrame()
    {
        FrameEuclideanTrajectoryPointList frameEuclideanTrajectoryPointList = new FrameEuclideanTrajectoryPointList();

        ReferenceFrame referenceFrameNotWorld = new ReferenceFrame("testFrame") {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {

            }
        };

        frameEuclideanTrajectoryPointList.setReferenceFrame(referenceFrameNotWorld);

        FrameSE3TrajectoryPointList frameSE3TrajectoryPointList = new FrameSE3TrajectoryPointList();
        FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(ReferenceFrame.getWorldFrame());
        frameSE3TrajectoryPointList.addPositionTrajectoryPoint(frameEuclideanTrajectoryPoint);

        frameEuclideanTrajectoryPointList.setIncludingFrame(frameSE3TrajectoryPointList);
        try
        {
            frameEuclideanTrajectoryPointList.getReferenceFrame().checkReferenceFrameMatch(referenceFrameNotWorld);
            assert(false);
        }
        catch (ReferenceFrameMismatchException e)
        {
            assert(true);
        }

    }

}
