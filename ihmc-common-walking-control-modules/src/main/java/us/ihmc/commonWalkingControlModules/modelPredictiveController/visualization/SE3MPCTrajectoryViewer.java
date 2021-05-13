package us.ihmc.commonWalkingControlModules.modelPredictiveController.visualization;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.EuclideanModelPredictiveController;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.SE3ModelPredictiveController;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SE3MPCTrajectoryViewer extends LinearMPCTrajectoryViewer
{
   private static final int numberOfVectors = 100;
   private static final double dt = 0.1;

   private static final double tallAxis = 0.1;
   private static final double wideAxis = 0.075;
   private static final double forwardAxis = 0.05;
   private static final Vector3D radii = new Vector3D(forwardAxis, wideAxis, tallAxis);

   private final BagOfEllipses orientationTrajectoryVectors;

   private final FramePoint3D comPositionToThrowAway = new FramePoint3D();
   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FramePoint3D dcmPositionToThrowAway = new FramePoint3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D vrpPositionToThrowAway = new FramePoint3D();
   private final FrameVector3D vrpVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D ecmpPositionToThrowAway = new FramePoint3D();

   public SE3MPCTrajectoryViewer(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(registry, yoGraphicsListRegistry);

      AppearanceDefinition orientationAppearance = YoAppearance.Green();
      orientationAppearance.setTransparency(0.8);

      orientationTrajectoryVectors = new BagOfEllipses(numberOfVectors, "Orientation", orientationAppearance, registry, yoGraphicsListRegistry);
   }

   private final FrameQuaternion orientation = new FrameQuaternion();

   public void compute(SE3ModelPredictiveController mpc, double currentTimeInState)
   {
      super.compute((EuclideanModelPredictiveController) mpc, currentTimeInState);

      orientationTrajectoryVectors.reset();

      int max = Math.min(numberOfVectors, (int) (0.75 / dt));
      for (int i = 0; i < max; i++)
      {
         double time = dt * i + currentTimeInState;

         mpc.compute(time,
                     comPositionToThrowAway,
                     comVelocityToThrowAway,
                     comAccelerationToThrowAway,
                     dcmPositionToThrowAway,
                     dcmVelocityToThrowAway,
                     vrpPositionToThrowAway,
                     vrpVelocityToThrowAway,
                     ecmpPositionToThrowAway);

         orientation.set(mpc.getDesiredBodyOrientationSolution());

         orientationTrajectoryVectors.setEllipseLoop(comPositionToThrowAway, orientation, radii);
      }
   }
}
