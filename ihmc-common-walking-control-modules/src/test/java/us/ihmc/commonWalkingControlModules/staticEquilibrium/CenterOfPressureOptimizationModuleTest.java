package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CenterOfPressureOptimizationModuleTest
{
   private static void runSimpleToyTest()
   {
      PoseReferenceFrame comFrame = new PoseReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame());
      comFrame.setPoseAndUpdate(new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(0.0, 0.0, 1.0), new Quaternion()));
      PoseReferenceFrame feetFrame = new PoseReferenceFrame("feetFrame", ReferenceFrame.getWorldFrame());
      feetFrame.setPoseAndUpdate(new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(0.0, 0.0, 0.0), new Quaternion()));

      CenterOfPressureStabilityMarginOptimizationModule optimizationModule = new CenterOfPressureStabilityMarginOptimizationModule("", 10.0, comFrame, feetFrame, new YoRegistry("a"), new YoGraphicsListRegistry());

      MutableWholeBodyContactState mutableWholeBodyContactState = new MutableWholeBodyContactState();
      mutableWholeBodyContactState.addContactPoint(new Point3D(0.5, 0.5, 0.0), Axis3D.Z, 1.5);
      mutableWholeBodyContactState.addContactPoint(new Point3D(0.5, -0.5, 0.0), Axis3D.Z, 1.5);
      mutableWholeBodyContactState.addContactPoint(new Point3D(-0.5, 0.5, 0.0), Axis3D.Z, 1.5);
      mutableWholeBodyContactState.addContactPoint(new Point3D(-0.5, -0.5, 0.0), Axis3D.Z, 1.5);

      optimizationModule.updateContactState(mutableWholeBodyContactState);
      boolean solve = optimizationModule.solve(1.0, 0.0);
      System.out.println(solve);

      Vector3D force0 = new Vector3D();
      Vector3D force1 = new Vector3D();
      Vector3D force2 = new Vector3D();
      Vector3D force3 = new Vector3D();

      optimizationModule.getResolvedForce(0, force0);
      optimizationModule.getResolvedForce(1, force1);
      optimizationModule.getResolvedForce(2, force2);
      optimizationModule.getResolvedForce(3, force3);

      System.out.println(force0);
      System.out.println(force1);
      System.out.println(force2);
      System.out.println(force3);
   }

   public static void main(String[] args)
   {
      runSimpleToyTest();
   }
}
