package us.ihmc.darpaRoboticsChallenge.stateEstimation.kinematicsBasedStateEstimator;

import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.AngularVelocitySensorConfiguration;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.LinearAccelerationSensorConfiguration;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class PelvisIMUBasedLinearStateCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final BooleanYoVariable enableEstimationOfGravity = new BooleanYoVariable("enableEstimationOfGravity", registry);
   private final DoubleYoVariable alphaGravityEstimation = new DoubleYoVariable("alphaGravityEstimation", registry);
   private final AlphaFilteredYoVariable gravityEstimation = new AlphaFilteredYoVariable("gravityEstimation", registry, alphaGravityEstimation);
   private final FrameVector gravity = new FrameVector(worldFrame);

   private final YoFrameVector rootJointLinearVelocity = new YoFrameVector("imuRootJointLinearVelocity", worldFrame, registry);
   private final YoFrameVector rootJointPosition = new YoFrameVector("imuRootJointPosition", worldFrame, registry);

   private final YoFrameVector yoMeasurementFrameLinearVelocity;
   private final YoFrameVector yoMeasurementFrameLinearVelocityInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurementInWorld;
   private final YoFrameVector yoLinearAccelerationMeasurement;
   
   private final BooleanYoVariable imuBasedStateEstimationEnabled = new BooleanYoVariable("imuBasedStateEstimationEnabled", registry);

   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;
   
   private final SixDoFJoint rootJoint;
   private final ReferenceFrame rootJointFrame;
   
   private final double estimatorDT;
   
   private final TwistCalculator twistCalculator;

   // Temporary variables
   private final FrameVector linearAccelerationMeasurement = new FrameVector();
   private final FrameVector angularVelocityMeasurement = new FrameVector();
   private final FrameVector measurementFrameLinearVelocityPrevValue = new FrameVector();
   private final FrameVector tempRootJointVelocityIntegrated = new FrameVector();

   private final LinearAccelerationSensorConfiguration linearAccelerationSensorConfiguration;
   private final AngularVelocitySensorConfiguration angularVelocitySensorConfiguration;

   @Deprecated
   private final BooleanYoVariable useOldHackishAccelIntegrationWorkingForAtlas = new BooleanYoVariable("useOldHackishAccelIntegrationWorkingForAtlas", registry);

   public PelvisIMUBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure,
         List<AngularVelocitySensorConfiguration> angularVelocitySensorConfigurations,
         List<LinearAccelerationSensorConfiguration> linearAccelerationSensorConfigurations, double estimatorDT, double gravitationalAcceleration,
         YoVariableRegistry parentRegistry)
   {
      this.estimatorDT = estimatorDT;
      this.twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
      this.rootJointFrame = rootJoint.getFrameAfterJoint();

      enableEstimationOfGravity.set(false);
      gravityEstimation.reset();
      gravityEstimation.update(Math.abs(gravitationalAcceleration));
      gravity.setIncludingFrame(worldFrame, 0.0, 0.0, gravityEstimation.getDoubleValue());
      
      if (linearAccelerationSensorConfigurations.size() == 0)
      {
         linearAccelerationSensorConfiguration = null;
         angularVelocitySensorConfiguration = null;
         imuBasedStateEstimationEnabled.set(false);
      }
      else if (linearAccelerationSensorConfigurations.size() > 1)
      {
         throw new RuntimeException("Cannot handle more than one IMU yet");
      }
      else
      {
         linearAccelerationSensorConfiguration = linearAccelerationSensorConfigurations.get(0);
         angularVelocitySensorConfiguration = angularVelocitySensorConfigurations.get(0);
         imuBasedStateEstimationEnabled.set(true);
      }
      
      if (imuBasedStateEstimationEnabled.getBooleanValue())
      {
         measurementLink = linearAccelerationSensorConfiguration.getLinearAccelerationMeasurementLink();
         measurementFrame = linearAccelerationSensorConfiguration.getMeasurementFrame();
      }
      else
      {
         measurementLink = null;
         measurementFrame = null;
      }
      
      yoMeasurementFrameLinearVelocity = new YoFrameVector("imuLinearVelocity", measurementFrame, registry);
      yoMeasurementFrameLinearVelocityInWorld = new YoFrameVector("imuLinearVelocityInWorld", worldFrame, registry);
      
      yoLinearAccelerationMeasurement = new YoFrameVector("imuLinearAcceleration", measurementFrame, registry);
      yoLinearAccelerationMeasurementInWorld = new YoFrameVector("imuLinearAccelerationInWorld", worldFrame, registry);
      
      parentRegistry.addChild(registry);
   }

   @Deprecated
   public void useHackishAccelerationIntegration(boolean val)
   {
      useOldHackishAccelIntegrationWorkingForAtlas.set(val);
   }
   
   public void enableGravityEstimation(boolean enable)
   {
      enableEstimationOfGravity.set(enable);
   }

   public void enableEsimationModule(boolean enable)
   {
      imuBasedStateEstimationEnabled.set(enable);
   }

   public boolean isEstimationEnabled()
   {
      return imuBasedStateEstimationEnabled.getBooleanValue();
   }
   
   public void setAlphaGravityEstimation(double alphaFilter)
   {
      alphaGravityEstimation.set(alphaFilter);
   }

   private final DenseMatrix64F A = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F x = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F b = new DenseMatrix64F(3, 1);
   
   private final Vector3d linearVelocity = new Vector3d();
   
   private void updateLinearAccelerationMeasurement(FrameVector measurementFrameLinearVelocityPrevValue)
   {
      if (!isEstimationEnabled())
         return;

      linearAccelerationMeasurement.setIncludingFrame(measurementFrame, linearAccelerationSensorConfiguration.getOutputPort().getData());
      if (enableEstimationOfGravity.getBooleanValue())
         gravityEstimation.update(linearAccelerationMeasurement.length());
      gravity.setIncludingFrame(worldFrame, 0.0, 0.0, gravityEstimation.getDoubleValue());

      // Update acceleration in world (minus gravity)
      linearAccelerationMeasurement.changeFrame(worldFrame);
      linearAccelerationMeasurement.sub(gravity);
      yoLinearAccelerationMeasurementInWorld.set(linearAccelerationMeasurement);

      // Update acceleration in local frame (minus gravity)
      gravity.changeFrame(measurementFrame);
      linearAccelerationMeasurement.setIncludingFrame(measurementFrame, linearAccelerationSensorConfiguration.getOutputPort().getData());
      linearAccelerationMeasurement.sub(gravity);
      yoLinearAccelerationMeasurement.set(linearAccelerationMeasurement);
      
      // Estimate IMU linear velocity in world
      yoLinearAccelerationMeasurementInWorld.getFrameTupleIncludingFrame(linearAccelerationMeasurement);
      linearAccelerationMeasurement.scale(estimatorDT);
      yoMeasurementFrameLinearVelocityInWorld.add(linearAccelerationMeasurement);
      
      // Do fancy stuff to estimate the IMU linear velocity in the measurement frame
      // omega_{measurementFrame}^{measurementFrame, world}
      angularVelocityMeasurement.setIncludingFrame(measurementFrame, angularVelocitySensorConfiguration.getOutputPort().getData());

      // vdot_{measurementFrame}^{measurementFrame, world}
      yoLinearAccelerationMeasurement.getFrameTupleIncludingFrame(linearAccelerationMeasurement);
      // vdot_{measurementFrame}^{measurementFrame, world} * estimatorDT
      linearAccelerationMeasurement.scale(estimatorDT);
      
      // Write the problem A*x = b, with:
      // A = Identity + skewOmega_{measurementFrame}^{measurementFrame, world}*estimatorDT
      // x = {t}^v_{measurementFrame}^{measurementFrame, world}
      // b = vdot_{measurementFrame}^{measurementFrame, world} * estimatorDT
      // This solution seems to remove the unwanted Coriolis terms when integrating the linear acceleration in the measurement frame
      
      MatrixTools.insertTuple3dIntoEJMLVector(linearAccelerationMeasurement.getVector(), b, 0);
      MatrixTools.vectorToSkewSymmetricMatrix(A, angularVelocityMeasurement.getVector());
      CommonOps.scale(estimatorDT, A);
      for (int i = 0; i < 3; i++)
         A.set(i, i, 1.0);
      
      CommonOps.solve(A, b, x);
      MatrixTools.extractTuple3dFromEJMLVector(linearVelocity, x, 0);
      
      yoMeasurementFrameLinearVelocity.set(measurementFrameLinearVelocityPrevValue);
      yoMeasurementFrameLinearVelocity.add(linearVelocity);
   }

   private final FrameVector linearVelocityMeasurementFrameRelativeToWorld = new FrameVector();

   private final Twist twistMeasurementLinkRelativeToWorld = new Twist();
   private final Twist twistRootJointFrameRelativeToMeasurementLink = new Twist();
   private final Twist twistRootJointFrameRelativeToWorld = new Twist();
   
   public void updatePelvisLinearVelocity(FrameVector rootJointLinearVelocityPrevValue, FrameVector rootJointLinearVelocityToPack)
   {
      if (!isEstimationEnabled())
         throw new RuntimeException("IMU estimation module for pelvis linear velocity is disabled.");
      
      if (useOldHackishAccelIntegrationWorkingForAtlas.getBooleanValue())
      {
         measurementFrameLinearVelocityPrevValue.setToZero(measurementFrame);
         updateLinearAccelerationMeasurement(measurementFrameLinearVelocityPrevValue);
         
         yoLinearAccelerationMeasurementInWorld.getFrameTupleIncludingFrame(linearAccelerationMeasurement);
         linearAccelerationMeasurement.scale(estimatorDT);
         rootJointLinearVelocity.set(rootJointLinearVelocityPrevValue);
         rootJointLinearVelocity.add(linearAccelerationMeasurement);
         rootJointLinearVelocity.getFrameTupleIncludingFrame(rootJointLinearVelocityToPack);
         
         return;
      }

      // T_{rootBody}^{rootBody, measurementLink}
      twistCalculator.packRelativeTwist(twistRootJointFrameRelativeToMeasurementLink, measurementLink, rootJoint.getSuccessor());
      // T_{rootBody}^{measurementFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeFrame(measurementFrame);
      // T_{rootBody}^{measurementFrame, measurementFrame}
      twistRootJointFrameRelativeToMeasurementLink.changeBaseFrameNoRelativeTwist(measurementFrame);
      // T_{rootJointFrame}^{rootJointFrame, measurementFrame}
      twistRootJointFrameRelativeToMeasurementLink.changeFrame(rootJointFrame);
      // T_{rootJointFrame}^{rootBody, measurementFrame}
      twistRootJointFrameRelativeToMeasurementLink.changeBodyFrameNoRelativeTwist(rootJointFrame);
      
      // T_{measurementLink}^{measurementLink, world}
      twistCalculator.packTwistOfBody(twistMeasurementLinkRelativeToWorld, measurementLink);

      // T_{measurementFrame}^{measurementLink, world}
      twistMeasurementLinkRelativeToWorld.changeBodyFrameNoRelativeTwist(measurementFrame);
      // T_{measurementFrame}^{measurementFrame, world}
      twistMeasurementLinkRelativeToWorld.changeFrame(measurementFrame);
      
      twistMeasurementLinkRelativeToWorld.packLinearPart(measurementFrameLinearVelocityPrevValue);
      updateLinearAccelerationMeasurement(measurementFrameLinearVelocityPrevValue);
      
      yoMeasurementFrameLinearVelocity.getFrameTupleIncludingFrame(linearVelocityMeasurementFrameRelativeToWorld);
      angularVelocityMeasurement.setIncludingFrame(measurementFrame, angularVelocitySensorConfiguration.getOutputPort().getData());
      twistMeasurementLinkRelativeToWorld.setLinearPart(linearVelocityMeasurementFrameRelativeToWorld);
      twistMeasurementLinkRelativeToWorld.setAngularPart(angularVelocityMeasurement);

      // T_{measurementFrame}^{rootJointFrame, world}
      twistMeasurementLinkRelativeToWorld.changeFrame(rootJointFrame);
      
      // T_{rootJointFrame}^{rootJointFrame, world} = T_{measurementFrame}^{rootJointFrame, world} + T_{rootJointFrame}^{rootJointFrame, measurementFrame}
      twistRootJointFrameRelativeToWorld.set(twistMeasurementLinkRelativeToWorld);
      twistRootJointFrameRelativeToWorld.add(twistRootJointFrameRelativeToMeasurementLink);
      
      ReferenceFrame originalReferenceFrame = rootJointLinearVelocityToPack.getReferenceFrame();
      twistRootJointFrameRelativeToWorld.packLinearPart(rootJointLinearVelocityToPack);
      rootJointLinearVelocityToPack.changeFrame(worldFrame);
      rootJointLinearVelocity.set(rootJointLinearVelocityToPack);
      rootJointLinearVelocityToPack.changeFrame(originalReferenceFrame);
   }

   public void updatePelvisPosition(FramePoint rootJointPositionPrevValue, FramePoint rootJointPositionToPack)
   {
      if (!isEstimationEnabled())
         throw new RuntimeException("IMU estimation module for pelvis linear velocity is disabled.");

      rootJointLinearVelocity.getFrameTupleIncludingFrame(tempRootJointVelocityIntegrated);
      tempRootJointVelocityIntegrated.scale(estimatorDT);

      rootJointPosition.set(rootJointPositionPrevValue);
      rootJointPosition.add(tempRootJointVelocityIntegrated);
      rootJointPosition.getFrameTupleIncludingFrame(rootJointPositionToPack);
   }
}
