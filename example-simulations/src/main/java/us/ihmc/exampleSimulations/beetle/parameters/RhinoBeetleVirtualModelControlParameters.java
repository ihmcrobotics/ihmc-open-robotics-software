package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class RhinoBeetleVirtualModelControlParameters implements HexapodControllerParameters
{
   private final String name = "vmcParams_";
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoPIDSE3Gains footGains;

   //body spatial feeback controller params
   private final Vector3D linearWeight = new Vector3D(1.0, 1.0, 10.0);
   private final Vector3D angularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final SymmetricYoPIDSE3Gains bodySpatialGains;
   private final double bodyProportionalGains = 8000.0;
   private final double bodyDampingRatio = 3.0;
   private final YoFrameVector3D bodySpatialLinearQPWeight;
   private final YoFrameVector3D bodySpatialAngularQPWeight;
   private final SelectionMatrix6D bodySpatialSelectionMatrix = new SelectionMatrix6D();

   public RhinoBeetleVirtualModelControlParameters(YoVariableRegistry parentRegistry)
   {
      bodySpatialGains = new SymmetricYoPIDSE3Gains(name + "bodySpatialGains", registry);
      bodySpatialGains.setProportionalGains(bodyProportionalGains);
      bodySpatialGains.setDampingRatios(bodyDampingRatio);

      bodySpatialLinearQPWeight = new YoFrameVector3D(name + "bodySpatial_linear_QPWeight", ReferenceFrame.getWorldFrame(), registry);
      bodySpatialAngularQPWeight = new YoFrameVector3D(name + "bodySpatial_angular_QPWeight", ReferenceFrame.getWorldFrame(), registry);
      bodySpatialAngularQPWeight.set(angularWeight);
      bodySpatialLinearQPWeight.set(linearWeight);

      DefaultYoPID3DGains positionGains = new DefaultYoPID3DGains(name + "FootPosition", GainCoupling.XY, false, registry);
      positionGains.setProportionalGains(getSwingXYProportionalGain(), getSwingXYProportionalGain(), getSwingZProportionalGain());
      positionGains.setDampingRatios(0.9);
      DefaultYoPID3DGains orientationGains = new DefaultYoPID3DGains(name + "FootOrientation", GainCoupling.XY, false, registry);
      orientationGains.setProportionalGains(0.0, 0.0, 0.0);
      footGains = new DefaultYoPIDSE3Gains(positionGains, orientationGains);

      parentRegistry.addChild(registry);
   }

   @Override
   public double getSwingTime()
   {
      return 0.5;
   }

   @Override
   public double getSwingXYProportionalGain()
   {
      return 8000.0;
   }

   @Override
   public double getSwingZProportionalGain()
   {
      return 6000.0;
   }

   @Override
   public PIDSE3Gains getBodySpatialGains()
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
   public YoPIDSE3Gains getFootGains()
   {
      return footGains;
   }

   @Override
   public double getTransferTime()
   {
      return 0;
   }
}
