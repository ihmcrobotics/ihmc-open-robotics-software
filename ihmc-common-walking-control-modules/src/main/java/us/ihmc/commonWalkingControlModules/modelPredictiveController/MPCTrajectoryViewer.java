package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MPCTrajectoryViewer
{
   private static final int numberOfVectors = 500;
   private static final double dt = 0.025;
   private static final double ballSize = 0.005;
   private static final double ellipseScale = 0.25;

   private final BagOfVectors comTrajectoryVectors;
   private final BagOfVectors dcmTrajectoryVectors;
   private final BagOfVectors vrpTrajectoryVectors;
   private final BagOfVectors angularVelocityTrajectoryVectors;
   private final BagOfEllipses orientationTrajectoryEllipses;

   private final FramePoint3D comPositionToThrowAway = new FramePoint3D();
   private final FrameVector3D comVelocityToThrowAway = new FrameVector3D();
   private final FrameVector3D comAccelerationToThrowAway = new FrameVector3D();
   private final FramePoint3D dcmPositionToThrowAway = new FramePoint3D();
   private final FrameVector3D dcmVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D vrpPositionToThrowAway = new FramePoint3D();
   private final FrameVector3D vrpVelocityToThrowAway = new FrameVector3D();
   private final FramePoint3D ecmpPositionToThrowAway = new FramePoint3D();
   private final FrameQuaternion bodyOrientationToThrowAway = new FrameQuaternion();
   private final FrameVector3D bodyAngularVelocityToThrowAway = new FrameVector3D();

   private final SpatialInertia inertia;
   private final Vector3D inertiaAxes = new Vector3D();

   public MPCTrajectoryViewer(SpatialInertia inertia, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.inertia = inertia;

      comTrajectoryVectors = new BagOfVectors(numberOfVectors, dt, ballSize, "CoM", YoAppearance.Black(), registry, yoGraphicsListRegistry);
      dcmTrajectoryVectors = new BagOfVectors(numberOfVectors, dt, ballSize, "DCM", YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      vrpTrajectoryVectors = new BagOfVectors(numberOfVectors, dt, ballSize, "VRP", YoAppearance.Green(), registry, yoGraphicsListRegistry);
      angularVelocityTrajectoryVectors = new BagOfVectors(numberOfVectors, dt, 0.0, "AngularVelocity", YoAppearance.Blue(), registry, yoGraphicsListRegistry);

      AppearanceDefinition orientationAppearance = YoAppearance.Green();
      orientationAppearance.setTransparency(0.8);

      orientationTrajectoryEllipses = new BagOfEllipses(numberOfVectors, "Orientation", orientationAppearance, registry, yoGraphicsListRegistry);
   }

   public void compute(CoMTrajectoryModelPredictiveController mpc, double currentTimeInState)
   {
      comTrajectoryVectors.reset();
      dcmTrajectoryVectors.reset();
      vrpTrajectoryVectors.reset();

      getInertiaEllipsoidRadii(inertia.getMomentOfInertia().getM00(),
                               inertia.getMomentOfInertia().getM11(),
                               inertia.getMomentOfInertia().getM22(),
                               inertia.getMass(),
                               inertiaAxes);

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
                     ecmpPositionToThrowAway,
                     bodyOrientationToThrowAway,
                     bodyAngularVelocityToThrowAway);

         dcmTrajectoryVectors.setVector(dcmPositionToThrowAway, dcmVelocityToThrowAway);
         vrpTrajectoryVectors.setVector(vrpPositionToThrowAway, vrpVelocityToThrowAway);
         comTrajectoryVectors.setVector(comPositionToThrowAway, comVelocityToThrowAway);
         angularVelocityTrajectoryVectors.setVector(comPositionToThrowAway, bodyAngularVelocityToThrowAway);
         orientationTrajectoryEllipses.setEllipse(comPositionToThrowAway, bodyOrientationToThrowAway, inertiaAxes);
      }
   }

   public static void getInertiaEllipsoidRadii(double Ixx, double Iyy, double Izz, double mass, Vector3DBasics inertiaAxesToPack)
   {
      //    http://en.wikipedia.org/wiki/Ellipsoid#Mass_properties
      inertiaAxesToPack.setX(Math.sqrt(5.0 / 2.0 * (Iyy + Izz - Ixx) / mass));
      inertiaAxesToPack.setY(Math.sqrt(5.0 / 2.0 * (Izz + Ixx - Iyy) / mass));
      inertiaAxesToPack.setZ(Math.sqrt(5.0 / 2.0 * (Ixx + Iyy - Izz) / mass));
      inertiaAxesToPack.scale(ellipseScale);
   }
}
