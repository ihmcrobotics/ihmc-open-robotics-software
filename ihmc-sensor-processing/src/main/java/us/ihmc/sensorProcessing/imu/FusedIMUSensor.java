package us.ihmc.sensorProcessing.imu;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FusedIMUSensor implements IMUSensorReadOnly
{
   private final String sensorName;

   private final ReferenceFrame fusedMeasurementFrame;
   private final RigidBodyBasics measurementLink;

   private final IMUSensorReadOnly firstIMU;
   private final IMUSensorReadOnly secondIMU;

   private final YoRegistry registry;
   private final YoFrameQuaternion quaternion;
   private final YoFrameYawPitchRoll orientation;
   private final YoFrameVector3D angularVelocity;
   private final YoFrameVector3D linearAcceleration;

   // Variables use estimating the IMUs drift around z
   private final YoDouble firstIMUYaw;
   private final YoDouble secondIMUYaw;
   private final YoDouble firstIMUYawPrevValue;
   private final YoDouble secondIMUYawPrevValue;
   private final YoDouble firstDriftYawRate;
   private final YoDouble secondDriftYawRate;
   private final YoDouble alphaIMUDriftFilter;
   private final AlphaFilteredYoVariable firstDriftYawRateFiltered;
   private final AlphaFilteredYoVariable secondDriftYawRateFiltered;
   private final YoDouble firstDriftYaw;
   private final YoDouble secondDriftYaw;

   private final double updateDT;

   // Temporary variables
   private final RotationMatrix rotationFromIMUToWorld = new RotationMatrix();
   private final RotationMatrix rotationFromFusedIMUToWorld = new RotationMatrix();

   private final RigidBodyTransform transformFromIMUToWorld = new RigidBodyTransform();
   private final RigidBodyTransform transformFromFusedIMUToIMU = new RigidBodyTransform();
   private final RigidBodyTransform transformFromFusedIMUToWorld = new RigidBodyTransform();

   private final FrameQuaternion fusedFrameOrientation = new FrameQuaternion();
   private final FrameQuaternion firstFrameOrientation = new FrameQuaternion();
   private final FrameQuaternion secondFrameOrientation = new FrameQuaternion();

   private final Vector3D firstVector = new Vector3D();
   private final Vector3D secondVector = new Vector3D();

   private final FrameVector3D firstFrameVector = new FrameVector3D();
   private final FrameVector3D secondFrameVector = new FrameVector3D();

   private final YawPitchRoll tempYawPitchRoll = new YawPitchRoll();

   public FusedIMUSensor(IMUSensorReadOnly firstIMU, IMUSensorReadOnly secondIMU, double updateDT,
                         YoRegistry parentRegistry)
   {
      this.firstIMU = firstIMU;
      this.secondIMU = secondIMU;

      this.updateDT = updateDT;

      sensorName = createSensorName();

      if (firstIMU.getMeasurementLink().equals(secondIMU.getMeasurementLink()))
         measurementLink = firstIMU.getMeasurementLink();
      else
         throw new RuntimeException("Both IMUs have to be attached to the same RigidBody.");

      fusedMeasurementFrame = createFusedMeasurementFrame();

      registry = new YoRegistry(sensorName);
      quaternion = new YoFrameQuaternion(sensorName, fusedMeasurementFrame, registry);
      orientation = new YoFrameYawPitchRoll(sensorName, fusedMeasurementFrame, registry);
      angularVelocity = new YoFrameVector3D("qd_w", sensorName, fusedMeasurementFrame, registry);
      linearAcceleration = new YoFrameVector3D("qdd_", sensorName, fusedMeasurementFrame, registry);

      firstIMUYaw = new YoDouble(firstIMU.getSensorName() + "Yaw", registry);
      secondIMUYaw = new YoDouble(secondIMU.getSensorName() + "Yaw", registry);
      firstIMUYawPrevValue = new YoDouble(firstIMU.getSensorName() + "YawPrevValue", registry);
      secondIMUYawPrevValue = new YoDouble(secondIMU.getSensorName() + "YawPrevValue", registry);
      firstDriftYawRate = new YoDouble(firstIMU.getSensorName() + "EstimatedDriftYawRate", registry);
      secondDriftYawRate = new YoDouble(secondIMU.getSensorName() + "EstimatedDriftYawRate", registry);
      alphaIMUDriftFilter = new YoDouble("alphaIMUDriftFilter", registry);
      alphaIMUDriftFilter.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(5.0, updateDT));
      firstDriftYawRateFiltered = new AlphaFilteredYoVariable(firstIMU.getSensorName() + "EstimatedDriftYawRateFiltered", registry, alphaIMUDriftFilter,
                                                              firstDriftYawRate);
      secondDriftYawRateFiltered = new AlphaFilteredYoVariable(secondIMU.getSensorName() + "EstimatedDriftYawRateFiltered", registry, alphaIMUDriftFilter,
                                                               secondDriftYawRate);
      firstDriftYaw = new YoDouble(firstIMU.getSensorName() + "EstimatedDriftYaw", registry);
      secondDriftYaw = new YoDouble(secondIMU.getSensorName() + "EstimatedDriftYaw", registry);

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

      YawPitchRoll firstYawPitchRoll = new YawPitchRoll();
      firstYawPitchRoll.set(firstTransform.getRotation());
      YawPitchRoll secondYawPitchRoll = new YawPitchRoll();
      secondYawPitchRoll.set(secondTransform.getRotation());

      Vector3D firstOffset = new Vector3D();
      firstOffset.set(firstTransform.getTranslation());
      Vector3D secondOffset = new Vector3D();
      secondOffset.set(secondTransform.getTranslation());

      Vector3D fusedOffset = new Vector3D();
      fusedOffset.add(firstOffset, secondOffset);
      fusedOffset.scale(0.5);

      YawPitchRoll fusedYawPitchRoll = new YawPitchRoll();
      for (int i = 0; i < 3; i++)
      {
         fusedYawPitchRoll.setElement(i, 0.5 * (firstYawPitchRoll.getElement(i) + secondYawPitchRoll.getElement(i)));
      }

      Quaternion fusedQuaternion = new Quaternion();
      fusedQuaternion.set(fusedYawPitchRoll);

      RigidBodyTransform fusedTransform = new RigidBodyTransform(fusedQuaternion, fusedOffset);
      ReferenceFrame fusedMeasurementFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(sensorName + "Frame", firstMeasurementFrame.getParent(), fusedTransform);

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

   private void estimateYawDriftAndCorrectOrientation(FrameQuaternion firstFrameOrientation, FrameQuaternion secondFrameOrientation,
                                                      FrameQuaternion fusedFrameOrientation)
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

      tempYawPitchRoll.set(firstFrameOrientation);
      tempYawPitchRoll.setYaw(tempYawPitchRoll.getYaw() - firstDriftYaw.getDoubleValue());
      tempYawPitchRoll.setYaw(AngleTools.trimAngleMinusPiToPi(tempYawPitchRoll.getYaw()));
      firstFrameOrientation.set(tempYawPitchRoll);

      tempYawPitchRoll.set(secondFrameOrientation);
      tempYawPitchRoll.setYaw(tempYawPitchRoll.getYaw() - secondDriftYaw.getDoubleValue());
      tempYawPitchRoll.setYaw(AngleTools.trimAngleMinusPiToPi(tempYawPitchRoll.getYaw()));
      secondFrameOrientation.set(tempYawPitchRoll);
   }

   private void measureOrientationInFusedFrame(FrameQuaternion orientationToPack, IMUSensorReadOnly imu)
   {
      // R_{IMU}^{world}
      rotationFromIMUToWorld.set(imu.getOrientationMeasurement());
      transformFromIMUToWorld.setRotationAndZeroTranslation(rotationFromIMUToWorld);

      // R_{Fused IMU}^{IMU}
      fusedMeasurementFrame.getTransformToDesiredFrame(transformFromFusedIMUToIMU, imu.getMeasurementFrame());

      // R_{Fused IMU}^{world} = R_{IMU}^{world} * R_{Fused IMU}^{IMU}
      transformFromFusedIMUToWorld.set(transformFromIMUToWorld);
      transformFromFusedIMUToWorld.multiply(transformFromFusedIMUToIMU);
      rotationFromFusedIMUToWorld.set(transformFromFusedIMUToWorld.getRotation());

      orientationToPack.setIncludingFrame(fusedMeasurementFrame, rotationFromFusedIMUToWorld);
   }

   private void updateAngularVelocity()
   {
      firstVector.set(firstIMU.getAngularVelocityMeasurement());
      secondVector.set(secondIMU.getAngularVelocityMeasurement());

      firstFrameVector.setIncludingFrame(firstIMU.getMeasurementFrame(), firstVector);
      secondFrameVector.setIncludingFrame(secondIMU.getMeasurementFrame(), secondVector);

      firstFrameVector.changeFrame(fusedMeasurementFrame);
      secondFrameVector.changeFrame(fusedMeasurementFrame);

      angularVelocity.add(firstFrameVector, secondFrameVector);
      angularVelocity.scale(0.5);
   }

   private void updateLinearAcceleration()
   {
      firstVector.set(firstIMU.getLinearAccelerationMeasurement());
      secondVector.set(secondIMU.getLinearAccelerationMeasurement());

      firstFrameVector.setIncludingFrame(firstIMU.getMeasurementFrame(), firstVector);
      secondFrameVector.setIncludingFrame(secondIMU.getMeasurementFrame(), secondVector);

      firstFrameVector.changeFrame(fusedMeasurementFrame);
      secondFrameVector.changeFrame(fusedMeasurementFrame);

      linearAcceleration.add(firstFrameVector, secondFrameVector);
      linearAcceleration.scale(0.5);
   }

   @Override
   public String getSensorName()
   {
      return sensorName;
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return fusedMeasurementFrame;
   }

   @Override
   public RigidBodyBasics getMeasurementLink()
   {
      return measurementLink;
   }

   @Override
   public QuaternionReadOnly getOrientationMeasurement()
   {
      return quaternion;
   }

   @Override
   public Vector3DReadOnly getAngularVelocityMeasurement()
   {
      return angularVelocity;
   }

   @Override
   public Vector3DReadOnly getLinearAccelerationMeasurement()
   {
      return linearAcceleration;
   }

   @Override
   public void getOrientationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getOrientationNoiseCovariance(noiseCovarianceToPack);
   }

   @Override
   public void getAngularVelocityNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getAngularVelocityNoiseCovariance(noiseCovarianceToPack);
   }

   @Override
   public void getAngularVelocityBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getAngularVelocityBiasProcessNoiseCovariance(biasProcessNoiseCovarianceToPack);
   }

   @Override
   public void getLinearAccelerationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getLinearAccelerationNoiseCovariance(noiseCovarianceToPack);
   }

   @Override
   public void getLinearAccelerationBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
   {
      // TODO Maybe do something smarter for that
      firstIMU.getLinearAccelerationBiasProcessNoiseCovariance(biasProcessNoiseCovarianceToPack);
   }
}
