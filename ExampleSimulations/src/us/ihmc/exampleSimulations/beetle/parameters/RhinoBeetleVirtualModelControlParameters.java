package us.ihmc.exampleSimulations.beetle.parameters;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootOrientationGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootPositionGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class RhinoBeetleVirtualModelControlParameters implements HexapodControllerParameters
{
   private final String name = "vmcParams_";
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   private final YoSE3PIDGainsInterface footGains;
   
   //body spatial feeback controller params
   private final Vector3D linearWeight = new Vector3D(1.0, 1.0, 10.0);
   private final Vector3D angularWeight = new Vector3D(1.0, 1.0, 1.0);
   private final YoSymmetricSE3PIDGains bodySpatialGains;
   private final double bodyProportionalGains = 8000.0;
   private final double bodyDampingRatio = 3.0;
   private final YoFrameVector bodySpatialLinearQPWeight;
   private final YoFrameVector bodySpatialAngularQPWeight;
   private final DenseMatrix64F bodySpatialSelectionMatrix = CommonOps.identity(SpatialAccelerationVector.SIZE);
   
   public RhinoBeetleVirtualModelControlParameters(YoVariableRegistry parentRegistry)
   {
      bodySpatialGains = new YoSymmetricSE3PIDGains(name + "bodySpatialGains", registry);
      bodySpatialGains.setProportionalGain(bodyProportionalGains);
      bodySpatialGains.setDampingRatio(bodyDampingRatio);
      bodySpatialGains.createDerivativeGainUpdater(true);
      
      bodySpatialLinearQPWeight = new YoFrameVector(name + "bodySpatial_linear_QPWeight", ReferenceFrame.getWorldFrame(), registry);
      bodySpatialAngularQPWeight = new YoFrameVector(name + "bodySpatial_angular_QPWeight", ReferenceFrame.getWorldFrame(), registry);
      bodySpatialAngularQPWeight.setVector(angularWeight);
      bodySpatialLinearQPWeight.setVector(linearWeight);
      
      
      footGains = new YoFootSE3Gains(name + "FootGains", registry);
      YoFootPositionGains positionGains = new YoFootPositionGains(name + "footPositionGains", registry);
      positionGains.setProportionalGains(getSwingXYProportionalGain(), getSwingZProportionalGain());
      positionGains.setDampingRatio(0.9);
      positionGains.createDerivativeGainUpdater(true);
      footGains.set(positionGains);
      YoFootOrientationGains orientationGains = new YoFootOrientationGains(name + "footOrientationGains", registry);
      orientationGains.setProportionalGains(0.0, 0.0, 0.0);
      footGains.set(orientationGains);
      
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
   public SE3PIDGainsInterface getBodySpatialGains()
   {
      return bodySpatialGains;
   }

   @Override
   public void getBodySpatialLinearQPWeight(Vector3D linearWeight)
   {
      bodySpatialLinearQPWeight.get(linearWeight);
   }

   @Override
   public void getBodySpatialAngularQPWeight(Vector3D angularWeight)
   {
      bodySpatialAngularQPWeight.get(angularWeight);
   }

   @Override
   public DenseMatrix64F getBodySpatialSelectionMatrix()
   {
      return bodySpatialSelectionMatrix;
   }

   @Override
   public YoSE3PIDGainsInterface getFootGains()
   {
      return footGains;
   }

   @Override
   public double getTransferTime()
   {
      // TODO Auto-generated method stub
      return 0;
   }
}
