package us.ihmc.sensorProcessing.imu;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class FusedIMUSensor implements IMUSensorReadOnly
{
   private final String sensorName;

   private final ReferenceFrame measurementFrame;
   private final RigidBody measurementLink;

   private final IMUSensorReadOnly firstIMU;
   private final IMUSensorReadOnly secondIMU;

   private final YoVariableRegistry registry;
   private final YoFrameQuaternion quaternion;
   private final YoFrameOrientation orientation;
   private final YoFrameVector angularVelocity;
   private final YoFrameVector linearAcceleration;

   // Temporary variables
   private final Matrix3d rotationFromIMUToWorld = new Matrix3d();
   private final Matrix3d rotationFromFusedIMUToWorld = new Matrix3d();

   private final Transform3D transformFromIMUToWorld = new Transform3D();
   private final Transform3D transformFromFusedIMUToIMU = new Transform3D();
   private final Transform3D transformFromFusedIMUToWorld = new Transform3D();

   private final FrameOrientation fusedFrameOrientation = new FrameOrientation();
   private final FrameOrientation firstFrameOrientation = new FrameOrientation();
   private final FrameOrientation secondFrameOrientation = new FrameOrientation();

   private final Vector3d firstVector = new Vector3d();
   private final Vector3d secondVector = new Vector3d();

   private final FrameVector firstFrameVector = new FrameVector();
   private final FrameVector secondFrameVector = new FrameVector();

   public FusedIMUSensor(IMUSensorReadOnly firstIMU, IMUSensorReadOnly secondIMU, YoVariableRegistry parentRegistry)
   {
      this.firstIMU = firstIMU;
      this.secondIMU = secondIMU;

      sensorName = createSensorName();

      if (firstIMU.getMeasurementLink().equals(secondIMU.getMeasurementLink()))
         this.measurementLink = firstIMU.getMeasurementLink();
      else
         throw new RuntimeException("Both IMUs have to be attached to the same RigidBody.");

      measurementFrame = createFusedMeasurementFrame();

      registry = new YoVariableRegistry(sensorName);
      quaternion = new YoFrameQuaternion(sensorName, measurementFrame, registry);
      orientation = new YoFrameOrientation(sensorName, measurementFrame, registry);
      angularVelocity = new YoFrameVector("qd_w", sensorName, measurementFrame, registry);
      linearAcceleration = new YoFrameVector("qdd_", sensorName, measurementFrame, registry);

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

      Transform3D firstTransform = firstMeasurementFrame.getTransformToParent();
      Transform3D secondTransform = secondMeasurementFrame.getTransformToParent();

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

      Transform3D fusedTransform = new Transform3D(fusedQuaternion, fusedOffset, 1.0);
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

      fusedFrameOrientation.setToZero(measurementFrame);
      fusedFrameOrientation.interpolate(firstFrameOrientation, secondFrameOrientation, 0.5);

      orientation.set(fusedFrameOrientation);
      quaternion.set(fusedFrameOrientation);
   }

   private void measureOrientationInFusedFrame(FrameOrientation orientationToPack, IMUSensorReadOnly imu)
   {
      // R_{IMU}^{world}
      imu.getOrientationMeasurement(rotationFromIMUToWorld);
      transformFromIMUToWorld.set(rotationFromIMUToWorld);

      // R_{Fused IMU}^{IMU}
      measurementFrame.getTransformToDesiredFrame(transformFromFusedIMUToIMU, imu.getMeasurementFrame());

      // R_{Fused IMU}^{world} = R_{IMU}^{world} * R_{Fused IMU}^{IMU}
      transformFromFusedIMUToWorld.mul(transformFromIMUToWorld, transformFromFusedIMUToIMU);
      transformFromFusedIMUToWorld.get(rotationFromFusedIMUToWorld);

      orientationToPack.setIncludingFrame(measurementFrame, rotationFromFusedIMUToWorld);
   }

   private void updateAngularVelocity()
   {
      firstIMU.getAngularVelocityMeasurement(firstVector);
      secondIMU.getAngularVelocityMeasurement(secondVector);

      firstFrameVector.setIncludingFrame(firstIMU.getMeasurementFrame(), firstVector);
      secondFrameVector.setIncludingFrame(secondIMU.getMeasurementFrame(), secondVector);

      firstFrameVector.changeFrame(measurementFrame);
      secondFrameVector.changeFrame(measurementFrame);

      angularVelocity.add(firstFrameVector, secondFrameVector);
      angularVelocity.scale(0.5);
   }

   private void updateLinearAcceleration()
   {
      firstIMU.getLinearAccelerationMeasurement(firstVector);
      secondIMU.getLinearAccelerationMeasurement(secondVector);

      firstFrameVector.setIncludingFrame(firstIMU.getMeasurementFrame(), firstVector);
      secondFrameVector.setIncludingFrame(secondIMU.getMeasurementFrame(), secondVector);

      firstFrameVector.changeFrame(measurementFrame);
      secondFrameVector.changeFrame(measurementFrame);

      linearAcceleration.add(firstFrameVector, secondFrameVector);
      linearAcceleration.scale(0.5);
   }

   public String getSensorName()
   {
      return sensorName;
   }

   public ReferenceFrame getMeasurementFrame()
   {
      return measurementFrame;
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
