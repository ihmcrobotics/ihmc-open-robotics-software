package us.ihmc.robotics.math.trajectories.generators;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class EuclideanTrajectoryPointCalculatorTest {
    @Test
    public void testChangeReferenceFrame()
    {
        EuclideanTrajectoryPointCalculator euclideanTrajectoryPointCalculator = new EuclideanTrajectoryPointCalculator();

        ReferenceFrame referenceFrameNotWorld = new ReferenceFrame("testFrame") {
            @Override
            protected void updateTransformToParent(RigidBodyTransform transformToParent)
            {

            }
        };

        euclideanTrajectoryPointCalculator.changeFrame(referenceFrameNotWorld);

        try
        {
            euclideanTrajectoryPointCalculator.appendTrajectoryPoint(0.0, new Point3D(0.0, 0.0, 0.0));
            assert(true);
        }
        catch(ReferenceFrameMismatchException e)
        {
            assert(false);
        }
    }
}
