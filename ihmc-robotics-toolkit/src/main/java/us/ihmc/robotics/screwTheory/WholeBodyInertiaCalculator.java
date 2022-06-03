package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicEllipsoid;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;

import java.util.stream.Stream;

public class WholeBodyInertiaCalculator
{
   private final RigidBodyBasics[] rigidBodiesInOrders;
   private final SpatialInertia tempInertia = new SpatialInertia();
   private final ReferenceFrame centerOfMassFrame;

   private final SpatialInertia wholeBodyInertia = new SpatialInertia();
   private final Matrix3DEigenValueCalculator eigenValueCalculator = new Matrix3DEigenValueCalculator();

   private final YoFramePoseUsingYawPitchRoll comPose;
   private final YoGraphicEllipsoid inertiaEllipsoid;

   public WholeBodyInertiaCalculator(ReferenceFrame centerOfMassFrame, YoGraphicsListRegistry graphicsListRegistry, RigidBodyBasics... rigidBodies )
   {
      this.centerOfMassFrame = centerOfMassFrame;
      rigidBodiesInOrders = Stream.of(rigidBodies).filter(body -> body.getInertia() != null).toArray(RigidBodyBasics[]::new);

      AppearanceDefinition appearance = YoAppearance.Green();
      appearance.setTransparency(0.5);
      comPose = new YoFramePoseUsingYawPitchRoll("comPose", ReferenceFrame.getWorldFrame(), null);

      if (graphicsListRegistry != null)
      {
         inertiaEllipsoid = new YoGraphicEllipsoid("inertiaEllipse", comPose, appearance, radii);
         graphicsListRegistry.registerYoGraphic("WholeBodyInertia", inertiaEllipsoid);
      }
      else
         inertiaEllipsoid = null;
   }

   public WholeBodyInertiaCalculator(ReferenceFrame centerOfMassFrame, YoGraphicsListRegistry graphicsListRegistry, RigidBodyBasics rootBody)
   {
      this(centerOfMassFrame, graphicsListRegistry, rootBody.subtreeArray());
   }


   public void computeAndPack(SpatialInertiaBasics wholeBodyInertia)
   {
      compute();
      wholeBodyInertia.setIncludingFrame(this.wholeBodyInertia);
   }

   public void compute()
   {
      wholeBodyInertia.setToZero(centerOfMassFrame, centerOfMassFrame);

      for (RigidBodyBasics rigidBody : rigidBodiesInOrders)
      {
         tempInertia.setIncludingFrame(rigidBody.getInertia());
         tempInertia.changeFrame(centerOfMassFrame);
         wholeBodyInertia.add(tempInertia);
      }

      if (inertiaEllipsoid != null)
         updateVisualizer();
   }

   public SpatialInertiaReadOnly getWholeBodyInertia()
   {
      return wholeBodyInertia;
   }

   private final  Vector3D radii = new Vector3D();

   private void updateVisualizer()
   {
      comPose.setFromReferenceFrame(centerOfMassFrame);
      eigenValueCalculator.compute(wholeBodyInertia.getMomentOfInertia());

      double meanDensity = 1000.0;

      double numerator = Math.pow(eigenValueCalculator.getFirstEigenValue() * eigenValueCalculator.getSecondEigenValue() * eigenValueCalculator.getThirdEigenValue(), 0.4);
      double constDenominator = Math.pow(8.0 * Math.PI * meanDensity / 15.0, 0.2);

      double axis1 = numerator / (eigenValueCalculator.getFirstEigenValue() * constDenominator);
      double axis2 = numerator / (eigenValueCalculator.getSecondEigenValue() * constDenominator);
      double axis3 = numerator / (eigenValueCalculator.getThirdEigenValue() * constDenominator);

      radii.set(axis1, axis2, axis3);
      inertiaEllipsoid.setRadii(radii);
   }
}
