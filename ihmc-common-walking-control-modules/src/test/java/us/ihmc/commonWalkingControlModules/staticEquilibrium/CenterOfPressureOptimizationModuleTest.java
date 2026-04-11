package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Test class for {@link CenterOfPressureStabilityMarginOptimizationModule}
 */
public class CenterOfPressureOptimizationModuleTest
{
   private static final boolean DEBUG = false;

   /**
    * One of the simplest possible tests of CenterOfPressureStabilityMarginOptimizationModule -- four coplanar contact points at z=0 and the surface normal
    * points vertically. The CoM is at (0, 0, 1) and robot has mass 10kg.
    */
   @Test
   public void runSimpleToyTest()
   {
      PoseReferenceFrame comFrame = new PoseReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame());
      PoseReferenceFrame feetFrame = new PoseReferenceFrame("feetFrame", ReferenceFrame.getWorldFrame());

      Point3D com = new Point3D(0.0, 0.0, 1.0);
      Point3D footFrame = new Point3D(0.0, 0.0, 0.0);

      comFrame.setPoseAndUpdate(new FramePose3D(ReferenceFrame.getWorldFrame(), com, new Quaternion()));
      feetFrame.setPoseAndUpdate(new FramePose3D(ReferenceFrame.getWorldFrame(), footFrame, new Quaternion()));

      double robotMass = 10.0;
      CenterOfPressureStabilityMarginOptimizationModule optimizationModule = new CenterOfPressureStabilityMarginOptimizationModule("",
                                                                                                                                   robotMass,
                                                                                                                                   comFrame,
                                                                                                                                   () -> feetFrame,
                                                                                                                                   new YoRegistry(getClass().getSimpleName()),
                                                                                                                                   new YoGraphicsListRegistry());

      MutableWholeBodyContactState mutableWholeBodyContactState = new MutableWholeBodyContactState();

      Point3D contactPoint0 = new Point3D(0.5, 0.5, 0.0);
      Point3D contactPoint1 = new Point3D(0.5, -0.5, 0.0);
      Point3D contactPoint2 = new Point3D(-0.5, 0.5, 0.0);
      Point3D contactPoint3 = new Point3D(-0.5, -0.5, 0.0);

      mutableWholeBodyContactState.addContactPoint(contactPoint0, Axis3D.Z, 1.5);
      mutableWholeBodyContactState.addContactPoint(contactPoint1, Axis3D.Z, 1.5);
      mutableWholeBodyContactState.addContactPoint(contactPoint2, Axis3D.Z, 1.5);
      mutableWholeBodyContactState.addContactPoint(contactPoint3, Axis3D.Z, 1.5);

      optimizationModule.updateContactState(mutableWholeBodyContactState);

      // Query along (1, 1). We expect all the force to be on the contact point at (-0.5, -0.5) and the reaction force in xy to be along the vector (1, 1)
      boolean solve = optimizationModule.solve(1.0, 1.0);
      Assertions.assertTrue(solve);

      Vector3D force0 = new Vector3D();
      Vector3D force1 = new Vector3D();
      Vector3D force2 = new Vector3D();
      Vector3D force3 = new Vector3D();

      optimizationModule.getResolvedForce(0, force0);
      optimizationModule.getResolvedForce(1, force1);
      optimizationModule.getResolvedForce(2, force2);
      optimizationModule.getResolvedForce(3, force3);

      if (DEBUG)
      {
         System.out.println(force0);
         System.out.println(force1);
         System.out.println(force2);
         System.out.println(force3);
      }

      double epsilon = 1.0e-12;
      Assertions.assertTrue(force0.epsilonEquals(new Vector3D(), epsilon));
      Assertions.assertTrue(force1.epsilonEquals(new Vector3D(), epsilon));
      Assertions.assertTrue(force2.epsilonEquals(new Vector3D(), epsilon));

      double fx = force3.getX();
      double fy = force3.getY();
      double fz = force3.getZ();

      // Check fz equals weight of the robot
      double expectedForceZ = robotMass * StabilityMarginOptimizationModule.GRAVITY;
      Assertions.assertTrue(MathTools.epsilonEquals(expectedForceZ, fz, epsilon));

      // Simple check is the direction and sign of f_xy, cannot check more without knowing the basis vector
      Assertions.assertTrue(MathTools.epsilonEquals(fx, fy, epsilon));
      Assertions.assertTrue(fx > 0.0);

      // Check net torque is zero, i.e. f || (c - p)
      Vector3D normalizedContactPointToCom = new Vector3D();
      Vector3D normalizedReactionForce = new Vector3D(force3);

      normalizedContactPointToCom.sub(com, contactPoint3);
      normalizedContactPointToCom.normalize();
      normalizedReactionForce.normalize();
      Assertions.assertTrue(normalizedContactPointToCom.epsilonEquals(normalizedReactionForce, epsilon));
   }
}
