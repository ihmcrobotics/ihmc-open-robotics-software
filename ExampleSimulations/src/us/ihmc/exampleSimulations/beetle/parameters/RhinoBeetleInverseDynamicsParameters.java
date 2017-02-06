package us.ihmc.exampleSimulations.beetle.parameters;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootOrientationGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootPositionGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;

public class RhinoBeetleInverseDynamicsParameters implements HexapodControllerParameters
{

   private final String name = "idParams_";
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   
   private final YoSE3PIDGainsInterface footGains;
   
   DoubleParameter bodySpatialLinearQPWeightX = parameterFactory.createDouble("bodySpatialLinearQPWeightX", 1.0);
   DoubleParameter bodySpatialLinearQPWeightY = parameterFactory.createDouble("bodySpatialLinearQPWeightY", 1.0);
   DoubleParameter bodySpatialLinearQPWeightZ = parameterFactory.createDouble("bodySpatialLinearQPWeightZ", 2.0);
   DoubleParameter bodySpatialAngularQPWeightX = parameterFactory.createDouble("bodySpatialAngularQPWeightX", 1.0);
   DoubleParameter bodySpatialAngularQPWeightY = parameterFactory.createDouble("bodySpatialAngularQPWeightY", 1.0);
   DoubleParameter bodySpatialAngularQPWeightZ = parameterFactory.createDouble("bodySpatialAngularQPWeightZ", 2.0);
   
   DoubleParameter swingTime = parameterFactory.createDouble("swingTime", 0.6);
   DoubleParameter transferTime = parameterFactory.createDouble("transferTime", 0.6);
   
   //body spatial feeback controller params
   private final YoSymmetricSE3PIDGains bodySpatialGains;
   private final double bodyProportionalGains = 500.0;
   private final double bodyDampingRatio = 1.1;
   private final DenseMatrix64F bodySpatialSelectionMatrix = CommonOps.identity(SpatialAccelerationVector.SIZE);
   
   public RhinoBeetleInverseDynamicsParameters(YoVariableRegistry parentRegistry)
   {
      bodySpatialGains = new YoSymmetricSE3PIDGains("bodySpatialGains", registry);
      bodySpatialGains.setProportionalGain(bodyProportionalGains);
      bodySpatialGains.setDampingRatio(bodyDampingRatio);
      bodySpatialGains.createDerivativeGainUpdater(true);
      
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
      return swingTime.get();
   }
   
   @Override
   public double getTransferTime()
   {
      return swingTime.get();
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
   public SE3PIDGainsInterface getBodySpatialGains()
   {
      return bodySpatialGains;
   }

   @Override
   public void getBodySpatialLinearQPWeight(Vector3d linearWeight)
   {
      linearWeight.setX(bodySpatialLinearQPWeightX.get());
      linearWeight.setY(bodySpatialLinearQPWeightY.get());
      linearWeight.setZ(bodySpatialLinearQPWeightZ.get());
   }

   @Override
   public void getBodySpatialAngularQPWeight(Vector3d angularWeight)
   {
      angularWeight.setX(bodySpatialAngularQPWeightX.get());
      angularWeight.setY(bodySpatialAngularQPWeightY.get());
      angularWeight.setZ(bodySpatialAngularQPWeightZ.get());
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
}
