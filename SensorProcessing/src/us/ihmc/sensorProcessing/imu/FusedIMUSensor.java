package us.ihmc.sensorProcessing.imu;

import javax.vecmath.*;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;


public class FusedIMUSensor implements IMUSensorReadOnly
{
   private final String sensorName;

   private final ReferenceFrame fusedMeasurementFrame;
   private final RigidBody measurementLink;

   private final IMUSensorReadOnly firstIMU;
   private final IMUSensorReadOnly secondIMU;

   private final YoVariableRegistry registry;
   private final YoFrameQuaternion quaternion;
   private final YoFrameOrientation orientation;
   private final YoFrameVector angularVelocity;
   private final YoFrameVector linearAcceleration;
   
   // Variables use estimating the IMUs drift around z
   private final DoubleYoVariable firstIMUYaw;
   private final DoubleYoVariable secondIMUYaw;
   private final DoubleYoVariable firstIMUYawPrevValue;
   private final DoubleYoVariable secondIMUYawPrevValue;
   private final DoubleYoVariable firstDriftYawRate;
   private final DoubleYoVariable secondDriftYawRate;
   private final DoubleYoVariable alphaIMUDriftFilter;
   private final AlphaFilteredYoVariable firstDriftYawRateFiltered;
   private final AlphaFilteredYoVariable secondDriftYawRateFiltered;
   private final DoubleYoVariable firstDriftYaw;
   private final DoubleYoVariable secondDriftYaw;

   private final double updateDT;
   
   // Temporary variables
   private final Matrix3d rotationFromIMUToWorld = new Matrix3d();
   private final Matrix3d rotationFromFusedIMUToWorld = new Matrix3d();

   private final RigidBodyTransform transformFromIMUToWorld = new RigidBodyTransform();
   private final RigidBodyTransform transformFromFusedIMUToIMU = new RigidBodyTransform();
   private final RigidBodyTransform transformFromFusedIMUToWorld = new RigidBodyTransform();

   private final FrameOrientation fusedFrameOrientation = new FrameOrientation();
   private final FrameOrientation firstFrameOrientation = new FrameOrientation();
   private final FrameOrientation secondFrameOrientation = new FrameOrientation();

   private final Vector3d firstVector = new Vector3d();
   private final Vector3d secondVector = new Vector3d();

   private final FrameVector firstFrameVector = new FrameVector();
   private final FrameVector secondFrameVector = new FrameVector();
   
   private final double[] tempYawPitchRoll = new double[3];

   public FusedIMUSensor(IMUSensorReadOnly firstIMU, IMUSensorReadOnly secondIMU, double updateDT, double imuDriftFilterFreqInHertz, YoVariableRegistry parentRegistry)
   {
      this.firstIMU = firstIMU;
      this.secondIMU = secondIMU;
      
      this.updateDT = updateDT;

      sensorName = createSensorName();

      if (firstIMU.getMeasurementLink().equals(secondIMU.getMeasurementLink()))
         this.measurementLink = firstIMU.getMeasurementLink();
      else
         throw new RuntimeException("Both IMUs have to be attached to the same RigidBody.");

      fusedMeasurementFrame = createFusedMeasurementFrame();

      registry = new YoVariableRegistry(sensorName);
      quaternion = new YoFrameQuaternion(sensorName, fusedMeasurementFrame, registry);
      orientation = new YoFrameOrientation(sensorName, fusedMeasurementFrame, registry);
      angularVelocity = new YoFrameVector("qd_w", sensorName, fusedMeasurementFrame, registry);
      linearAcceleration = new YoFrameVector("qdd_", sensorName, fusedMeasurementFrame, registry);

      firstIMUYaw = new DoubleYoVariable(firstIMU.getSensorName() + "Yaw", registry);
      secondIMUYaw = new DoubleYoVariable(secondIMU.getSensorName() + "Yaw", registry);
      firstIMUYawPrevValue = new DoubleYoVariable(firstIMU.getSensorName() + "YawPrevValue", registry);
      secondIMUYawPrevValue = new DoubleYoVariable(secondIMU.getSensorName() + "YawPrevValue", registry);
      firstDriftYawRate = new DoubleYoVariable(firstIMU.getSensorName() + "EstimatedDriftYawRate", registry);
      secondDriftYawRate = new DoubleYoVariable(secondIMU.getSensorName() + "EstimatedDriftYawRate", registry);
      alphaIMUDriftFilter = new DoubleYoVariable("alphaIMUDriftFilter", registry);
      alphaIMUDriftFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(5.0, updateDT));
      firstDriftYawRateFiltered = new AlphaFilteredYoVariable(firstIMU.getSensorName() + "EstimatedDriftYawRateFiltered", registry, alphaIMUDriftFilter, firstDriftYawRate);
      secondDriftYawRateFiltered = new AlphaFilteredYoVariable(secondIMU.getSensorName() + "EstimatedDriftYawRateFiltered", registry, alphaIMUDriftFilter, secondDriftYawRate);
      firstDriftYaw = new DoubleYoVariable(firstIMU.getSensorName() + "EstimatedDriftYaw", registry);
      secondDriftYaw = new DoubleYoVariable(secondIMU.getSensorName() + "EstimatedDriftYaw", registry);

      fusedFrameOrientation.setToZero(fusedMeasurementFrame);

      parentRegistry.addChild(registry);
   }

   private String createSensorName()
   {
      String firstName = new String(firstIMU.getSensorName());
      String secondName = new String(secondIMU.getSensorName());
      String fusedName = new String(firstName);

      firstName = removeSidePrefixFromName(firstName);
      secondName = removeSidePrefixFromName(secondName);

      while (fusedName.length() > 0)
      {
         if (firstName.contains(fusedName) && secondName.contains(fusedName))
            break;
         else
            fusedName = fusedName.substring(1);
      }

      if (fusedName.length() == 0)
         fusedName = "Fused" + firstIMU.getMeasurementLink().getName();
      else
         fusedName = "Fused" + fusedName;

      return fusedName;
   }

   private String removeSidePrefixFromName(String name)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         name = name.replace(robotSide.getCamelCaseNameForMiddleOfExpression(), "");
         name = name.replace(robotSide.getCamelCaseNameForStartOfExpression(), "");
      }

      return name;
   }

   private ReferenceFrame createFusedMeasurementFrame()
   {
      ReferenceFrame firstMeasurementFrame = firstIMU.getMeasurementFrame();
      ReferenceFrame secondMeasurementFrame = secondIMU.getMeasurementFrame();

      firstMeasurementFrame.getParent().checkReferenceFrameMatch(secondMeasurementFrame.getParent());

      RigidBodyTransform firstTransform = firstMeasurementFrame.getTransformToParent();
      RigidBodyTransform secondTransform = secondMeasurementFrame.getTransformToParent();

      double[] firstYawPitchRoll = new double[3];
      RotationFunctions.getYawPitchRoll(firstYawPitchRoll, firstTransform);
      double[] secondYawPitchRoll = new double[3];
      RotationFunctions.getYawPitchRoll(secondYawPitchRoll, secondTransform);

      Vector3d firstOffset = new Vector3d();
      firstTransform.get(firstOffset);
      Vector3d secondOffset = new Vector3d();
      secondTransform.get(secondOffset);

      Vector3d fusedOffset = new Vector3d();
      fusedOffset.add(firstOffset, secondOffset);
      fusedOffset.scale(0.5);

      double[] fusedYawPitchRoll = new double[3];
      for (int i = 0; i < 3; i++)
      {
         fusedYawPitchRoll[i] = firstYawPitchRoll[i] + secondYawPitchRoll[i];
         fusedYawPitchRoll[i] *= 0.5;
      }

      Quat4d fusedQuaternion = new Quat4d();
      RotationFunctions.setQuaternionBasedOnYawPitchRoll(fusedQuaternion, fusedYawPitchRoll);

      RigidBodyTransform fusedTransform = new RigidBodyTransform(fusedQuaternion, fusedOffset);
      ReferenceFrame fusedMeasurementFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(sensorName + "Frame",
            firstMeasurementFrame.getParent(), fusedTransform);

      return fusedMeasurementFrame;
   }

   public void update()
   {
      updateOrientation();
      updateAngularVelocity();
      updateLinearAcceleration();
   }

   private void updateOrientation()
   {
      measureOrientationInFusedFrame(firstFrameOrientation, firstIMU);
      measureOrientationInFusedFrame(secondFrameOrientation, secondIMU);

//      estimateYawDriftAndCorrectOrientation(firstFrameOrientation, secondFrameOrientation, fusedFrameOrientation);

      fusedFrameOrientation.interpolate(firstFrameOrientation, secondFrameOrientation, 0.0);

      orientation.set(fusedFrameOrientation);
      quaternion.set(fusedFrameOrientation);
   }

   private void estimateYawDriftAndCorrectOrientation(FrameOrientation firstFrameOrientation, FrameOrientation secondFrameOrientation, FrameOrientation fusedFrameOrientation)
   {
      firstFrameOrientation.checkReferenceFrameMatch(fusedMeasurementFrame);
      secondFrameOrientation.checkReferenceFrameMatch(fusedMeasurementFrame);
      fusedFrameOrientation.checkReferenceFrameMatch(fusedMeasurementFrame);
      
      double fusedIMUYaw = fusedFrameOrientation.getYaw();
      
      firstIMUYaw.set(firstFrameOrientation.getYaw() - fusedIMUYaw);
      secondIMUYaw.set(secondFrameOrientation.getYaw() - fusedIMUYaw);
      
      firstDriftYawRate.set(AngleTools.computeAngleDifferenceMinusPiToPi(firstIMUYaw.getDoubleValue(), firstIMUYawPrevValue.getDoubleValue()) / updateDT);
      secondDriftYawRate.set(AngleTools.computeAngleDifferenceMinusPiToPi(secondIMUYaw.getDoubleValue(), secondIMUYawPrevValue.getDoubleValue()) / updateDT);
      firstDriftYawRateFiltered.update();
      secondDriftYawRateFiltered.update();
      
      firstDriftYaw.add(firstDriftYawRateFiltered.getDoubleValue() * updateDT);
      secondDriftYaw.add(secondDriftYawRateFiltered.getDoubleValue() * updateDT);
      
      firstIMUYawPrevValue.set(firstIMUYaw.getDoubleValue());
      secondIMUYawPrevValue.set(secondIMUYaw.getDoubleValue());
      
      firstFrameOrientation.getYawPitchRoll(tempYawPitchRoll);
      tempYawPitchRoll[0] -= firstDriftYaw.getDoubleValue();
      tempYawPitchRoll[0] = AngleTools.trimAngleMinusPiToPi(tempYawPitchRoll[0]);
      firstFrameOrientation.setYawPitchRoll(tempYawPitchRoll);
      
      secondFrameOrientation.getYawPitchRoll(tempYawPitchRoll);
      tempYawPitchRoll[0] -= secondDriftYaw.getDoubleValue();
      tempYawPitchRoll[0] = AngleTools.trimAngleMinusPiToPi(tempYawPitchRoll[0]);
      secondFrameOrientation.setYawPitchRoll(tempYawPitchRoll);
   }

   private void measureOrientationInFusedFrame(FrameOrientation orientationToPack, IMUSensorReadOnly imu)
   {
      // R_{IMU}^{world}
      imu.getOrientationMeasurement(rotationFromIMUToWorld);
      transformFromIMUToWorld.setRotationAndZeroTranslation(rotationFromIMUToWorld);

      // R_{Fused IMU}^{IMU}
      fusedMeasurementFrame.getTransformToDesiredFrame(transformFromFusedIMUToIMU, imu.getMeasurementFrame());

      // R_{Fused IMU}^{world} = R_{IMU}^{world} * R_{Fused IMU}^{IMU}
      transformFromFusedIMUToWorld.multiply(transformFromIMUToWorld, transformFromFusedIMUToIMU);
      transformFromFusedIMUToWorld.get(rotationFromFusedIMUToWorld);

      orientationToPack.setIncludingFrame(fusedMeasurementFrame, rotationFromFusedIMUToWorld);
   }

   private void updateAngularVelocity()
   {
      firstIMU.getAngularVelocityMeasurement(firstVector);
      secondIMU.getAngularVelocityMeasurement(secondVector);

      firstFrameVector.setIncludingFrame(firstIMU.getMeasurementFrame(), firstVector);
      secondFrameVector.setIncludingFrame(secondIMU.getMeasurementFrame(), secondVector);

      firstFrameVector.changeFrame(fusedMeasurementFrame);
      secondFrameVector.changeFrame(fusedMeasurementFrame);

      angularVelocity.add(firstFrameVector, secondFrameVector);
      angularVelocity.scale(0.5);
   }

   private void updateLinearAcceleration()
   {
      firstIMU.getLinearAccelerationMeasurement(firstVector);
      secondIMU.getLinearAccelerationMeasurement(secondVector);

      firstFrameVector.setIncludingFrame(firstIMU.getMeasurementFrame(), firstVector);
      secondFrameVector.setIncludingFrame(secondIMU.getMeasurementFrame(), secondVector);

      firstFrameVector.changeFrame(fusedMeasurementFrame);
      secondFrameVector.changeFrame(fusedMeasurementFrame);

      linearAcceleration.add(firstFrameVector, secondFrameVector);
      linearAcceleration.scale(0.5);
   }

   public String getSensorName()
   {
      return sensorName;
   }

   public ReferenceFrame getMeasurementFrame()
   {
      return fusedMeasurementFrame;
   }

   public RigidBody getMeasurementLink()
   {
      return measurementLink;
   }

   public void getOrientationMeasurement(Matrix3d orientationToPack)
   {
      quaternion.get(orientationToPack);
   }

   public void getAngularVelocityMeasurement(Vector3d angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   public void getLinearAccelerationMeasurement(Vector3d linearAccelerationToPack)
   {
      linearAcceleration.get(linearAccelerationToPack);
   }

   @Override public void getOrientationMeasurement(Matrix3f orientationToPack)
   {
      quaternion.get(orientationToPack);
   }

   @Override public void getAngularVelocityMeasurement(Vector3f angularVelocityToPack)
   {
      angularVelocity.get(angularVelocityToPack);
   }

   @Override public void getLinearAccelerationMeasurement(Vector3f linearAccelerationToPack)
   {
      linearAcceleration.get(linearAccelerationToPack);
   }

   public void getOrientationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getOrientationNoiseCovariance(noiseCovarianceToPack);
   }

   public void getAngularVelocityNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getAngularVelocityNoiseCovariance(noiseCovarianceToPack);
   }

   public void getAngularVelocityBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getAngularVelocityBiasProcessNoiseCovariance(biasProcessNoiseCovarianceToPack);
   }

   public void getLinearAccelerationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getLinearAccelerationNoiseCovariance(noiseCovarianceToPack);
   }

   public void getLinearAccelerationBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getLinearAccelerationBiasProcessNoiseCovariance(biasProcessNoiseCovarianceToPack);
   }
}
