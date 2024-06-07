package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

/**
 * PelvisRotationalStateUpdater reads and transforms the orientation and angular velocity obtained
 * from the IMU to update the pelvis orientation and angular velocity in world.
 *
 * @author Sylvain
 */
public class IMUBasedPelvisRotationalStateUpdater implements PelvisRotationalStateUpdaterInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoFrameYawPitchRoll yoRootJointFrameOrientation;
   private final YoFrameQuaternion yoRootJointFrameQuaternion;
   private final YoDouble rootJointUnclampedYaw = new YoDouble("rootJointEstimatedUnclampedYaw", registry);

   private final YoFrameVector3D yoRootJointAngularVelocityMeasFrame;
   private final YoFrameVector3D yoRootJointAngularVelocity;
   private final YoFrameVector3D yoRootJointAngularVelocityInWorld;

   private final BooleanParameter zeroYawAtInitialization = new BooleanParameter("zeroEstimatedRootYawAtInitialization", registry, true);
   private final YoDouble initialYaw = new YoDouble("initialEstimatedRootYaw", registry);

   private final FiniteDifferenceAngularVelocityYoFrameVector yoRootJointAngularVelocityFromFD;

   private final FloatingJointBasics rootJoint;
   private final ReferenceFrame rootJointFrame;

   private final IMUSensorReadOnly imuProcessedOutput;
   private final IMUBiasProvider imuBiasProvider;
   private final YawDriftProvider imuYawDriftEstimator;

   private final ReferenceFrame measurementFrame;
   private final RigidBodyBasics measurementLink;

   public IMUBasedPelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure,
                                               List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                               double dt,
                                               YoRegistry parentRegistry)
   {
      this(inverseDynamicsStructure, imuProcessedOutputs, null, null, dt, parentRegistry);
   }

   public IMUBasedPelvisRotationalStateUpdater(FullInverseDynamicsStructure inverseDynamicsStructure,
                                               List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                               IMUBiasProvider imuBiasProvider,
                                               YawDriftProvider imuYawDriftEstimator,
                                               double dt,
                                               YoRegistry parentRegistry)
   {
      this.imuBiasProvider = imuBiasProvider;
      this.imuYawDriftEstimator = imuYawDriftEstimator;
      checkNumberOfSensors(imuProcessedOutputs);

      imuProcessedOutput = imuProcessedOutputs.get(0);

      rootJoint = inverseDynamicsStructure.getRootJoint();
      rootJointFrame = rootJoint.getFrameAfterJoint();

      measurementFrame = imuProcessedOutput.getMeasurementFrame();
      measurementLink = imuProcessedOutput.getMeasurementLink();

      yoRootJointFrameOrientation = new YoFrameYawPitchRoll("estimatedRootJoint", worldFrame, registry);
      yoRootJointFrameQuaternion = new YoFrameQuaternion("estimatedRootJoint", worldFrame, registry);

      yoRootJointAngularVelocity = new YoFrameVector3D("estimatedRootJointAngularVelocity", rootJointFrame, registry);
      yoRootJointAngularVelocityInWorld = new YoFrameVector3D("estimatedRootJointAngularVelocityWorld", worldFrame, registry);
      yoRootJointAngularVelocityMeasFrame = new YoFrameVector3D("estimatedRootJointAngularVelocityMeasFrame", measurementFrame, registry);

      yoRootJointAngularVelocityFromFD = new FiniteDifferenceAngularVelocityYoFrameVector("estimatedRootJointAngularVelocityFromFD",
                                                                                          yoRootJointFrameQuaternion,
                                                                                          dt,
                                                                                          registry);

      parentRegistry.addChild(registry);
   }

   public IMUSensorReadOnly getIMUUsedForEstimation()
   {
      return imuProcessedOutput;
   }

   private void checkNumberOfSensors(List<? extends IMUSensorReadOnly> imuProcessedOutputs)
   {
      if (imuProcessedOutputs.size() > 1)
         System.out.println(getClass().getSimpleName() + ": More than 1 IMU sensor, using only the first one: " + imuProcessedOutputs.get(0).getSensorName());

      if (imuProcessedOutputs.size() == 0)
         throw new RuntimeException("No sensor set up for the IMU.");
   }

   @Override
   public void initialize()
   {
      if (zeroYawAtInitialization.getValue())
      {
         computeOrientationAtEstimateFrame(measurementFrame, imuProcessedOutput.getOrientationMeasurement(), rootJointFrame, rotationFromRootJointFrameToWorld);
         initialYaw.set(rotationFromRootJointFrameToWorld.getYaw());
      }
      else
      {
         initialYaw.set(0.0);
      }

      updateRootJointOrientationAndAngularVelocity();
   }

   @Override
   public void updateRootJointOrientationAndAngularVelocity()
   {
      updateRootJointRotation();
      updateRootJointTwistAngularPart();
      updateViz();
   }

   private final RotationMatrix rotationFromRootJointFrameToWorld = new RotationMatrix();

   private void updateRootJointRotation()
   {
      computeOrientationAtEstimateFrame(measurementFrame, imuProcessedOutput.getOrientationMeasurement(), rootJointFrame, rotationFromRootJointFrameToWorld);

      rootJoint.setJointOrientation(rotationFromRootJointFrameToWorld);
      rootJointFrame.update();

      if (zeroYawAtInitialization.getValue())
      {
         rotationFromRootJointFrameToWorld.prependYawRotation(-initialYaw.getValue());
      }

      if (imuYawDriftEstimator != null)
      {
         imuYawDriftEstimator.update();
         rotationFromRootJointFrameToWorld.prependYawRotation(-imuYawDriftEstimator.getYawBiasInWorldFrame());
      }

      rootJoint.setJointOrientation(rotationFromRootJointFrameToWorld);
      rootJointFrame.update();
   }

   /**
    * Computes the orientation of {@code estimateFrame} given the orientation at
    * {@code measurementFrame}.
    * <p>
    * This method assumes that {@code estimateFrame} and {@code measurementFrame} are connected and
    * that the relative transform between the two frames is known.
    * </p>
    *
    * @param measurementFrame          reference frame in which the measurement was taken.
    * @param orientationMeasurement    the measurement of the {@code measurementFrame} orientation. Not
    *                                  modified.
    * @param estimateFrame             the reference frame for which the orientation is to be computed.
    * @param orientationEstimateToPack result: the orientation of {@code estimateFrame}.
    */
   public static void computeOrientationAtEstimateFrame(ReferenceFrame measurementFrame,
                                                        Orientation3DReadOnly orientationMeasurement,
                                                        ReferenceFrame estimateFrame,
                                                        Orientation3DBasics orientationEstimateToPack)
   {
      orientationEstimateToPack.setToZero();
      // R_{estimateFrame}^{measurementFrame}
      estimateFrame.transformFromThisToDesiredFrame(measurementFrame, orientationEstimateToPack);

      // R_{estimateFrame}^{world} = R_{measurementFrame}^{world} * R_{estimateFrame}^{measurementFrame}
      orientationEstimateToPack.prepend(orientationMeasurement);
   }

   private final Vector3D angularVelocityMeasurement = new Vector3D();

   /** Angular velocity of the measurement link, with respect to world. */
   private final FrameVector3D angularVelocityMeasurementLinkRelativeToWorld = new FrameVector3D();

   /** Angular velocity of the estimation link, with respect to the measurement link. */
   private final FrameVector3D angularVelocityRootJointFrameRelativeToMeasurementLink = new FrameVector3D();

   /** Twist of the estimation link, with respect to the measurement link. */
   private final Twist twistRootJointFrameRelativeToMeasurementLink = new Twist();

   private void updateRootJointTwistAngularPart()
   {
      // T_{rootBody}^{rootBody, measurementLink}
      rootJoint.getSuccessor().getBodyFixedFrame().getTwistRelativeToOther(measurementLink.getBodyFixedFrame(), twistRootJointFrameRelativeToMeasurementLink);
      // T_{rootBody}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.changeFrame(rootJointFrame);
      // T_{rootJointFrame}^{rootJointFrame, measurementLink}
      twistRootJointFrameRelativeToMeasurementLink.setBodyFrame(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, measurementLink}
      angularVelocityRootJointFrameRelativeToMeasurementLink.setIncludingFrame(twistRootJointFrameRelativeToMeasurementLink.getAngularPart());

      // omega_{measurementLink}^{measurementFrame, world}
      angularVelocityMeasurement.set(imuProcessedOutput.getAngularVelocityMeasurement());
      if (imuBiasProvider != null)
      {
         FrameVector3DReadOnly angularVelocityBiasInIMUFrame = imuBiasProvider.getAngularVelocityBiasInIMUFrame(imuProcessedOutput);
         if (angularVelocityBiasInIMUFrame != null)
            angularVelocityMeasurement.sub(angularVelocityBiasInIMUFrame);
      }
      angularVelocityMeasurementLinkRelativeToWorld.setIncludingFrame(measurementFrame, angularVelocityMeasurement);

      // omega_{measurementLink}^{rootJointFrame, world}
      angularVelocityMeasurementLinkRelativeToWorld.changeFrame(rootJointFrame);

      // omega_{rootJointFrame}^{rootJointFrame, world} = omega_{rootJointFrame}^{rootJointFrame, measurementLink} + omega_{measurementLink}^{rootJointFrame, world}
      rootJoint.getJointTwist().getAngularPart().add(angularVelocityRootJointFrameRelativeToMeasurementLink, angularVelocityMeasurementLinkRelativeToWorld);
      rootJoint.updateFrame();

      yoRootJointAngularVelocity.setMatchingFrame(rootJoint.getJointTwist().getAngularPart());
      yoRootJointAngularVelocityMeasFrame.setMatchingFrame(angularVelocityMeasurementLinkRelativeToWorld);
      yoRootJointAngularVelocityInWorld.setMatchingFrame(rootJoint.getJointTwist().getAngularPart());
   }

   private void updateViz()
   {
      yoRootJointFrameQuaternion.checkReferenceFrameMatch(worldFrame);
      yoRootJointFrameQuaternion.set(rotationFromRootJointFrameToWorld);
      yoRootJointAngularVelocityFromFD.update();

      yoRootJointFrameOrientation.checkReferenceFrameMatch(worldFrame);
      double yawPrevious = yoRootJointFrameOrientation.getYaw();
      yoRootJointFrameOrientation.set(rotationFromRootJointFrameToWorld);
      double yawCurrent = yoRootJointFrameOrientation.getYaw();
      rootJointUnclampedYaw.add(EuclidCoreTools.angleDifferenceMinusPiToPi(yawCurrent, yawPrevious));
   }

   @Override
   public FrameOrientation3DReadOnly getEstimatedOrientation()
   {
      return yoRootJointFrameQuaternion;
   }

   @Override
   public FrameVector3DReadOnly getEstimatedAngularVelocity()
   {
      return yoRootJointAngularVelocityInWorld;
   }
}
