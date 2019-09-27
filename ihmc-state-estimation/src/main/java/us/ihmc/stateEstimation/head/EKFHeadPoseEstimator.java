package us.ihmc.stateEstimation.head;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.ekf.filter.sensor.implementations.MagneticFieldSensor;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * An estimator for the head pose based on the EKF package. The inputs to the estimation algorithm are:
 *
 * <li>The head position as estimated by some other estimation algorithm.</li>
 * <li>The (possibly biased) angular velocity measurement of an IMU attached to the head.</li>
 * <li>The linear acceleration measurement of an IMU attached to the head.</li>
 * <li>The magnetic field vector measurement of an IMU attached to the head.</li>
 *
 * <p>
 * To use this class you must set all measurements each tick and then call {@link #compute()} to perform a single
 * prediction and correction step using the most recently set measurements. To initialize (optional) the estimation call
 * {@link #initialize(RigidBodyTransform, FrameVector3D)}. This method can also be called to reset the estimator after
 * use.
 *
 * @author Georg Wiedebach
 *
 */
public class EKFHeadPoseEstimator implements AvatarHeadPoseEstimatorInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble confirmedDt = new YoDouble("confirmedDt", registry);
   private final YoLong previousCallNanos = new YoLong("previousCallNanos", registry);

   private final PoseState poseState;
   private final LinearAccelerationSensor linearAccelerationSensor;
   private final MagneticFieldSensor magneticFieldSensor;
   private final AngularVelocitySensor angularVelocitySensor;
   private final PositionSensor positionSensor;
   private final StateEstimator stateEstimator;

   private final SixDoFJoint headJoint;

   private final RigidBodyTransform headTransform = new RigidBodyTransform();
   private final Twist headTwist = new Twist();

   private final YoFramePoseUsingYawPitchRoll headPose;

   private FullRobotModel fullRobotModel;
   private YoGraphicsListRegistry yoGraphicListRegistry;

   /**
    * Creates a new pose estimator.
    *
    * @param dt the time interval at which compute is called and the measurements are sampled.
    * @param imuToHead the transform that describes the location of the IMU on the head.
    * @param estimateAngularVelocityBias whether a bias should be estimated for the angular velocity measurements.
    */
   public EKFHeadPoseEstimator(double dt, RigidBodyTransform imuToHead, boolean estimateAngularVelocityBias)
   {
      LogTools.info("EKFHeadPoseEstimator dt = " + dt);
      previousCallNanos.set(System.nanoTime());

      // Creates a simple joint structure representing the head attached to world with a floating joint:
      RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      headJoint = new SixDoFJoint("head_joint", elevator);
      RigidBodyBasics headBody = new RigidBody("head", headJoint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      MovingReferenceFrame headFrame = headJoint.getFrameAfterJoint();
      MovingReferenceFrame imuFrame = MovingReferenceFrame.constructFrameFixedInParent("imu", headFrame, imuToHead);

      // Create all the sensors:
      angularVelocitySensor = new AngularVelocitySensor("AngularVelocity", dt, headBody, imuFrame, estimateAngularVelocityBias, registry);
      linearAccelerationSensor = new LinearAccelerationSensor("LinearAcceleration", dt, headBody, imuFrame, false, registry);
      magneticFieldSensor = new MagneticFieldSensor("MagneticField", dt, headBody, imuFrame, registry);
      positionSensor = new PositionSensor("Position", dt, registry);
//      List<Sensor> sensors = Arrays.asList(new Sensor[] {angularVelocitySensor, positionSensor, linearAccelerationSensor, magneticFieldSensor});
      List<Sensor> sensors = Arrays.asList(new Sensor[] {angularVelocitySensor, positionSensor, linearAccelerationSensor});

      // Create the state and the estimator:
      poseState = new PoseState("Head", dt, headFrame, registry);
      RobotState robotState = new RobotState(poseState, Collections.emptyList());
      stateEstimator = new StateEstimator(sensors, robotState, registry);

      headPose = new YoFramePoseUsingYawPitchRoll("EstimatedHeadPose", ReferenceFrame.getWorldFrame(), registry);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   /**
    * Packs the most recent estimate of the head pose.
    *
    * @param headTransform to be packed.
    */
   @Override
   public void getHeadTransform(RigidBodyTransform headTransform)
   {
      poseState.getTransform(headTransform);
   }

   /**
    * Initializes and resets the estimation. This method allows to calibrate the magnetic field direction in world and
    * to initialize the estimated pose with an initial guess. The call to this method is optional: by default the head
    * pose will start as identity and the magnetic field direction (north) will start as the positive x-axis in world.
    *
    * @param initialHeadTransform the initial guess for the head pose.
    * @param magneticFieldDirection the "north" direction in world frame.
    */
   @Override
   public void initialize(RigidBodyTransform initialHeadTransform, FrameVector3D magneticFieldDirection)
   {
      headTwist.setToZero(headJoint.getFrameAfterJoint(), headJoint.getFrameBeforeJoint(), headJoint.getFrameAfterJoint());
      poseState.initialize(initialHeadTransform, headTwist);
      magneticFieldSensor.setNorth(magneticFieldDirection);
      linearAccelerationSensor.resetBias();
      angularVelocitySensor.resetBias();
      stateEstimator.reset();
      updateRobot();
   }

   /**
    * Should be called each tick before {@link #compute()} to set the sensor measurement.
    *
    * @param angularVelocity measurement in IMU frame.
    */
   public void setImuAngularVelocity(Vector3DReadOnly angularVelocity)
   {
      angularVelocitySensor.setMeasurement(angularVelocity);
   }

   /**
    * Should be called each tick before {@link #compute()} to set the sensor measurement.
    *
    * @param linearAcceleration measurement in IMU frame.
    */
   public void setImuLinearAcceleration(Vector3DReadOnly linearAcceleration)
   {
      linearAccelerationSensor.setMeasurement(linearAcceleration);
   }

   /**
    * Should be called each tick before {@link #compute()} to set the sensor measurement.
    *
    * @param magneticFieldVector measurement in IMU frame.
    */
   public void setImuMagneticFieldVector(Vector3DReadOnly magneticFieldVector)
   {
      magneticFieldSensor.setMeasurement(magneticFieldVector);
   }

   /**
    * Should be called each tick before {@link #compute()} to set the sensor measurement.
    *
    * @param headPosition estimate in world frame.
    */
   public void setEstimatedHeadPosition(Point3DReadOnly headPosition)
   {
      positionSensor.setMeasurement(headPosition);
   }

   /**
    * Call every tick to perform one estimation update (prediction and correction). Call
    * <li>{@link #setImuAngularVelocity(Vector3DReadOnly)}</li>
    * <li>{@link #setImuLinearAcceleration(Vector3DReadOnly)}</li>
    * <li>{@link #setImuMagneticFieldVector(Vector3DReadOnly)}</li>
    * <li>{@link #setEstimatedHeadPosition(Point3DReadOnly)}</li>
    * <p>
    * each tick with the newest measurements before calling this method. After the call to this method the newest head
    * pose estimate can be obtained via {@link #getHeadTransform(RigidBodyTransform)}.
    */
   @Override
   public void compute()
   {
      long nanoTime = System.nanoTime();

      confirmedDt.set((nanoTime - previousCallNanos.getLongValue()) * 1.0e-9);
      previousCallNanos.set(nanoTime);

      stateEstimator.predict();
      updateRobot();
      stateEstimator.correct();
      updateRobot();

      // Update Yo-Pose with the last estimate.
      if (headPose != null)
      {
         poseState.getTransform(headTransform);
         headPose.set(headTransform);
      }
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
   }

   protected FullRobotModel getFullRobotModel()
   {
      return this.fullRobotModel;
   }

   private void updateRobot()
   {
      poseState.getTransform(headTransform);
      headJoint.setJointConfiguration(headTransform);
      poseState.getTwist(headTwist);
      headJoint.setJointTwist(headTwist);
      headJoint.updateFramesRecursively();
   }
}
