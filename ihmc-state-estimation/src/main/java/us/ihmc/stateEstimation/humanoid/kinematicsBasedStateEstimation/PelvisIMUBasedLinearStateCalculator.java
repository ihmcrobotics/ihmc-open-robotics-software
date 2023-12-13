package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisIMUBasedLinearStateCalculator implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double estimatorDT;
   private final ReferenceFrame rootJointFrame;
   private final ReferenceFrame rootJointBaseFrame;
   private final ReferenceFrame measurementFrame;
   private final IMUSensorReadOnly imuProcessedOutput;
   private final IMUBiasProvider imuBiasProvider;

   private final BooleanProvider useAccelerometerForEstimation;
   private final BooleanProvider cancelGravityFromAccelerationMeasurement;

   private final FrameVector3DReadOnly gravityVector;

   private final YoFrameVector3D yoLinearAccelerationMeasurementInWorld;
   private final YoFrameVector3D yoLinearAccelerationMeasurement;

   // Debug variables: pure integration of IMU to velocity and position
   private final YoBoolean resetRootJointPositionIMUOnly = new YoBoolean("resetRootJointPositionIMUOnly", registry);
   private final YoDouble alphaLeakIMUOnly = new YoDouble("imuOnlyAlphaLeak", registry);

   private final YoFrameVector3D previousMeasurementFrameLinearVelocity = new YoFrameVector3D("previousEstimatedMeasurementFrameLinearVelocity",
                                                                                              worldFrame,
                                                                                              registry);
   private final YoFrameVector3D rootJointPositionIMUOnly = new YoFrameVector3D("rootJointPositionIMUOnly", worldFrame, registry);
   private final YoFrameVector3D imuLinearVelocityIMUOnly = new YoFrameVector3D("imuLinearVelocityIMUOnly", worldFrame, registry);
   private final YoFrameVector3D rootJointLinearVelocityIMUOnly = new YoFrameVector3D("rootJointLinearVelocityIMUOnly", worldFrame, registry);

   // Temporary variables
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final Twist twist = new Twist();

   public PelvisIMUBasedLinearStateCalculator(FloatingJointBasics rootJoint,
                                              List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                              IMUBiasProvider imuBiasProvider,
                                              BooleanProvider cancelGravityFromAccelerationMeasurement,
                                              double estimatorDT,
                                              double gravitationalAcceleration,
                                              StateEstimatorParameters stateEstimatorParameters,
                                              YoGraphicsListRegistry yoGraphicsListRegistry,
                                              YoRegistry parentRegistry)
   {
      this.imuBiasProvider = imuBiasProvider;
      this.estimatorDT = estimatorDT;
      this.rootJointFrame = rootJoint.getFrameAfterJoint();
      this.rootJointBaseFrame = rootJoint.getFrameBeforeJoint();
      this.cancelGravityFromAccelerationMeasurement = cancelGravityFromAccelerationMeasurement;

      if (stateEstimatorParameters == null)
      {
         useAccelerometerForEstimation = new BooleanParameter("useAccelerometerForEstimation", registry);
      }
      else
      {
         boolean initialValue = stateEstimatorParameters.useAccelerometerForEstimation();
         useAccelerometerForEstimation = new BooleanParameter("useAccelerometerForEstimation", registry, initialValue);
      }

      gravityVector = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(() -> worldFrame, new Vector3D(0, 0, -Math.abs(gravitationalAcceleration)));

      if (imuProcessedOutputs.size() == 0)
      {
         imuProcessedOutput = null;
         measurementFrame = null;
      }
      else
      {
         if (imuProcessedOutputs.size() > 1)
            System.out.println(getClass().getSimpleName() + ": More than 1 IMU sensor, using only the first one: "
                  + imuProcessedOutputs.get(0).getSensorName());
         imuProcessedOutput = imuProcessedOutputs.get(0);
         measurementFrame = imuProcessedOutput.getMeasurementFrame();
      }

      yoLinearAccelerationMeasurement = new YoFrameVector3D("imuLinearAcceleration", measurementFrame, registry);
      yoLinearAccelerationMeasurementInWorld = new YoFrameVector3D("imuLinearAccelerationInWorld", worldFrame, registry);

      resetRootJointPositionIMUOnly.set(true);
      alphaLeakIMUOnly.set(0.999);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      imuLinearVelocityIMUOnly.setToZero();
      resetRootJointPositionIMUOnly.set(true);
      previousMeasurementFrameLinearVelocity.setToZero();
   }

   public boolean isEstimationEnabled()
   {
      return useAccelerometerForEstimation.getValue() && imuProcessedOutput != null;
   }

   public void updateLinearAccelerationMeasurement()
   {
      if (!isEstimationEnabled())
         return;

      FrameVector3DReadOnly biasInput = imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imuProcessedOutput);
      Vector3DReadOnly rawInput = imuProcessedOutput.getLinearAccelerationMeasurement();

      linearAcceleration.setReferenceFrame(measurementFrame);
      linearAcceleration.sub(rawInput, biasInput);

      // Update acceleration in world (minus gravity)
      if (cancelGravityFromAccelerationMeasurement.getValue())
      {
         linearAcceleration.changeFrame(worldFrame);
         linearAcceleration.add(gravityVector);
      }

      yoLinearAccelerationMeasurementInWorld.setMatchingFrame(linearAcceleration);
      yoLinearAccelerationMeasurement.setMatchingFrame(linearAcceleration);
   }

   public void estimateRootJointLinearVelocity(FrameVector3DReadOnly estimatedRootJointAngularVelocity, FixedFrameVector3DBasics rootJointVelocityToUpdate)
   {
      twist.setToZero(rootJointFrame, rootJointBaseFrame, measurementFrame);
      twist.getLinearPart().setMatchingFrame(previousMeasurementFrameLinearVelocity);
      twist.getLinearPart().scaleAdd(estimatorDT, yoLinearAccelerationMeasurement, twist.getLinearPart());
      twist.getAngularPart().setMatchingFrame(estimatedRootJointAngularVelocity);
      twist.changeFrame(rootJointFrame);
      rootJointVelocityToUpdate.setMatchingFrame(twist.getLinearPart());

      // Update debug variables
      linearVelocity.setIncludingFrame(yoLinearAccelerationMeasurementInWorld);
      linearVelocity.scale(estimatorDT);
      imuLinearVelocityIMUOnly.scaleAdd(alphaLeakIMUOnly.getDoubleValue(), linearVelocity);
      twist.setToZero(rootJointFrame, rootJointBaseFrame, measurementFrame);
      twist.getLinearPart().setMatchingFrame(imuLinearVelocityIMUOnly);
      twist.getAngularPart().setMatchingFrame(estimatedRootJointAngularVelocity);
      twist.changeFrame(rootJointFrame);
      rootJointLinearVelocityIMUOnly.setMatchingFrame(twist.getLinearPart());
   }

   public void estimateRootJointPosition(FramePoint3DReadOnly previousRootJointPosition,
                                         FrameVector3DReadOnly estimatedRootJointAngularVelocity,
                                         FixedFramePoint3DBasics rootJointPositionToUpdate)
   {
      twist.setToZero(rootJointFrame, rootJointBaseFrame, measurementFrame);
      twist.getLinearPart().setMatchingFrame(previousMeasurementFrameLinearVelocity);
      twist.getLinearPart().scaleAdd(0.5 * estimatorDT, yoLinearAccelerationMeasurement, twist.getLinearPart());
      twist.getAngularPart().setMatchingFrame(estimatedRootJointAngularVelocity);
      twist.changeFrame(rootJointFrame);
      linearVelocity.setIncludingFrame(twist.getLinearPart());
      linearVelocity.changeFrame(worldFrame);

      // Update debug variables
      if (resetRootJointPositionIMUOnly.getValue())
      {
         resetRootJointPositionIMUOnly.set(false);
         rootJointPositionIMUOnly.set(previousRootJointPosition);
      }
      rootJointPositionIMUOnly.scaleAdd(estimatorDT, rootJointLinearVelocityIMUOnly, rootJointPositionIMUOnly);

      // Updating last in case rootJointPositionToUpdate == previousRootJointPosition
      rootJointPositionToUpdate.scaleAdd(estimatorDT, linearVelocity, previousRootJointPosition);
   }

   /**
    * The estimated linear velocity at the measurement frame has to be saved in world for next
    * estimation tick.
    * 
    * @param estimatedRootJointTwist the final value of the root joint estimated twist for this
    *                                estimation tick.
    */
   public void saveMeasurementFrameTwist(TwistReadOnly estimatedRootJointTwist)
   {
      twist.setIncludingFrame(estimatedRootJointTwist);
      twist.changeFrame(measurementFrame);
      previousMeasurementFrameLinearVelocity.setMatchingFrame(twist.getLinearPart());
   }

   public ReferenceFrame getIMUMeasurementFrame()
   {
      return measurementFrame;
   }

   public FrameVector3DReadOnly getLinearAccelerationMeasurement()
   {
      return yoLinearAccelerationMeasurement;
   }

   public FrameVector3DReadOnly getLinearAccelerationMeasurementInWorld()
   {
      return yoLinearAccelerationMeasurementInWorld;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
