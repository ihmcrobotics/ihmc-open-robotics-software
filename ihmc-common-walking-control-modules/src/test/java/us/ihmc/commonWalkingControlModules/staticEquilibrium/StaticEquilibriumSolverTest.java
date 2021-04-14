package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class StaticEquilibriumSolverTest
{
   private static final boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize", "true"));

   @Test
   public void testTriangleFlat()
   {
      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleFlatGround();
      if (VISUALIZE)
      {
         new StaticEquilibriumSolverVisualizer(input);
      }
   }

   @Test
   public void testTriangleLowAngle()
   {
      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly();
      if (VISUALIZE)
      {
         new StaticEquilibriumSolverVisualizer(input);
      }
   }

   @Test
   public void testTriangleHighAngle()
   {
      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutALot();
      if (VISUALIZE)
      {
         new StaticEquilibriumSolverVisualizer(input);
      }
   }

   @Test
   public void testBipedFeet()
   {
      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeet();
      if (VISUALIZE)
      {
         new StaticEquilibriumSolverVisualizer(input);
      }
   }

   @Test
   public void testBipedFeetWithHandhold()
   {
      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createBipedFeetWithHandhold();
      if (VISUALIZE)
      {
         new StaticEquilibriumSolverVisualizer(input);
      }
   }
}
