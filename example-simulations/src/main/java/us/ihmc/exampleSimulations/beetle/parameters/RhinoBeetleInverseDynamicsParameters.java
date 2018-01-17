package us.ihmc.exampleSimulations.beetle.parameters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RhinoBeetleInverseDynamicsParameters implements HexapodControllerParameters
{

   private final String name = "idParams_";
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final YoPIDSE3Gains footGains;

   //body spatial feeback controller params
   private final Vector3D linearWeight = new Vector3D(1.0, 1.0, 2.0);
   private final Vector3D angularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final SymmetricYoPIDSE3Gains bodySpatialGains;
   private final double bodyProportionalGains = 500.0;
   private final double bodyDampingRatio = 1.1;
   private final YoFrameVector bodySpatialLinearQPWeight;
   private final YoFrameVector bodySpatialAngularQPWeight;
   private final SelectionMatrix6D bodySpatialSelectionMatrix = new SelectionMatrix6D();

   public RhinoBeetleInverseDynamicsParameters(YoVariableRegistry parentRegistry)
   {
      bodySpatialGains = new SymmetricYoPIDSE3Gains("bodySpatialGains", registry);
      bodySpatialGains.setProportionalGains(bodyProportionalGains);
      bodySpatialGains.setDampingRatios(bodyDampingRatio);

      bodySpatialLinearQPWeight = new YoFrameVector("bodySpatial_linear_QPWeight", ReferenceFrame.getWorldFrame(), registry);
      bodySpatialAngularQPWeight = new YoFrameVector("bodySpatial_angular_QPWeight", ReferenceFrame.getWorldFrame(), registry);
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
}
