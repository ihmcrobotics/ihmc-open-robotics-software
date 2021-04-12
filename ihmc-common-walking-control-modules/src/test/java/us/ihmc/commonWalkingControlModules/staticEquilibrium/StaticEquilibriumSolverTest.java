package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;

public class StaticEquilibriumSolverTest
{
   private static final boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize", "false"));

   @Test
   public void testTriangleFlat()
   {
      StaticEquilibriumSolverInput input = new StaticEquilibriumSolverInput();
      double angle = Math.toRadians(10.0);
      double[] angles = new double[] {angle, angle, angle};

      double distance = 1.0;
      for (int i = 0; i < 3; i++)
      {
         double theta = i * 2.0 * Math.PI / 3.0;
         FramePoint3D contactPoint = new FramePoint3D(ReferenceFrame.getWorldFrame(), distance * Math.cos(theta), distance * Math.sin(theta), 0.0);

         Vector3D axis = new Vector3D(contactPoint);
         axis.normalize();
         axis.set(-axis.getY(), axis.getX(), 0.0);

         AxisAngle axisAngle = new AxisAngle(axis, angles[i]);
         FrameVector3D normal = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 1.0);
         axisAngle.transform(normal);

         input.addContactPoint(contactPoint, normal);
      }

      input.setRobotMass(1.0);

      if (VISUALIZE)
      {
         new StaticEquilibriumSolverVisualizer(input);
      }
   }
}
