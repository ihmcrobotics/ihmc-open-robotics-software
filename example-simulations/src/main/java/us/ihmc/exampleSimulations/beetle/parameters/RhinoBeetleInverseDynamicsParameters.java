package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.wholeBodyControlCore.pidGains.GainCoupling;
import us.ihmc.wholeBodyControlCore.pidGains.PIDSE3GainsBasics;
import us.ihmc.wholeBodyControlCore.pidGains.implementations.YoPID3DGains;
import us.ihmc.wholeBodyControlCore.pidGains.implementations.YoPIDSE3Gains;
import us.ihmc.wholeBodyControlCore.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RhinoBeetleInverseDynamicsParameters implements HexapodControllerParameters
{

   private final String name = "idParams_";
   private final YoRegistry registry = new YoRegistry(name);

   private final PIDSE3GainsBasics footGains;

   //body spatial feeback controller params
   private final Vector3D linearWeight = new Vector3D(1.0, 1.0, 2.0);
   private final Vector3D angularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final SymmetricYoPIDSE3Gains bodySpatialGains;
   private final double bodyProportionalGains = 500.0;
   private final double bodyDampingRatio = 1.1;
   private final YoFrameVector3D bodySpatialLinearQPWeight;
   private final YoFrameVector3D bodySpatialAngularQPWeight;
   private final SelectionMatrix6D bodySpatialSelectionMatrix = new SelectionMatrix6D();

   public RhinoBeetleInverseDynamicsParameters(YoRegistry parentRegistry)
   {
      bodySpatialGains = new SymmetricYoPIDSE3Gains("bodySpatialGains", registry);
      bodySpatialGains.setProportionalGains(bodyProportionalGains);
      bodySpatialGains.setDampingRatios(bodyDampingRatio);

      bodySpatialLinearQPWeight = new YoFrameVector3D("bodySpatial_linear_QPWeight", ReferenceFrame.getWorldFrame(), registry);
      bodySpatialAngularQPWeight = new YoFrameVector3D("bodySpatial_angular_QPWeight", ReferenceFrame.getWorldFrame(), registry);
      bodySpatialAngularQPWeight.set(angularWeight);
      bodySpatialLinearQPWeight.set(linearWeight);

      YoPID3DGains positionGains = new YoPID3DGains(name + "FootPosition", GainCoupling.XY, false, registry);
      positionGains.setProportionalGains(getSwingXYProportionalGain(), getSwingXYProportionalGain(), getSwingZProportionalGain());
      positionGains.setDampingRatios(0.9);
      YoPID3DGains orientationGains = new YoPID3DGains(name + "FootOrientation", GainCoupling.XY, false, registry);
      orientationGains.setProportionalGains(0.0, 0.0, 0.0);
      footGains = new YoPIDSE3Gains(positionGains, orientationGains);

      parentRegistry.addChild(registry);
   }

   @Override
   public double getSwingTime()
   {
      return 0.6;
   }

   @Override
   public double getTransferTime()
   {
      return 0.4;
   }

   @Override
   public double getSwingXYProportionalGain()
   {
      return 2000.0;
   }

   @Override
   public double getSwingZProportionalGain()
   {
      return 2000.0;
   }

   @Override
   public PIDSE3GainsBasics getBodySpatialGains()
   {
      return bodySpatialGains;
   }

   @Override
   public void getBodySpatialLinearQPWeight(Vector3D linearWeight)
   {
      linearWeight.set(bodySpatialLinearQPWeight);
   }

   @Override
   public void getBodySpatialAngularQPWeight(Vector3D angularWeight)
   {
      angularWeight.set(bodySpatialAngularQPWeight);
   }

   @Override
   public SelectionMatrix6D getBodySpatialSelectionMatrix()
   {
      return bodySpatialSelectionMatrix;
   }

   @Override
   public PIDSE3GainsBasics getFootGains()
   {
      return footGains;
   }

   @Override
   public WholeBodyControllerCoreMode getControlMode()
   {
      return WholeBodyControllerCoreMode.INVERSE_DYNAMICS;
   }
}
