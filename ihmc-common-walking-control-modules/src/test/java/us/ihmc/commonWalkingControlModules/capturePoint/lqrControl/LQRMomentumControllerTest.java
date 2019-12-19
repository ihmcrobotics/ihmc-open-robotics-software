package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

import java.util.ArrayList;
import java.util.List;

public class LQRMomentumControllerTest
{
   private static final double epsilon = 1e-12;

   @Test
   public void test()
   {
      LQRMomentumController controller = new LQRMomentumController();

      Point3D vrpStart = new Point3D(0.0, 0.0, 1.0);
      Point3D vrpEnd = new Point3D(1.0, 0.5, 1.0);
      Trajectory3D vrpTrajectory = new Trajectory3D(4);
      vrpTrajectory.setLinear(0.0, 1.0, vrpStart, vrpEnd);
      List<Trajectory3D> trajectories = new ArrayList<>();
      trajectories.add(vrpTrajectory);

      controller.setVrpTrajectory(trajectories);

      DenseMatrix64F AExpected = new DenseMatrix64F(6, 6);
      AExpected.set(0, 3, 1.0);
      AExpected.set(1, 4, 1.0);
      AExpected.set(2, 5, 1.0);

      DenseMatrix64F BExpected = new DenseMatrix64F(6, 3);
      BExpected.set(3, 0, 1.0);
      BExpected.set(4, 1, 1.0);
      BExpected.set(5, 2, 1.0);

      EjmlUnitTests.assertEquals(AExpected, controller.A, epsilon);
      EjmlUnitTests.assertEquals(BExpected, controller.B, epsilon);

//      controller.computeControlInput(new DenseMatrix64F(6, 1), 0.5);

   }
}
