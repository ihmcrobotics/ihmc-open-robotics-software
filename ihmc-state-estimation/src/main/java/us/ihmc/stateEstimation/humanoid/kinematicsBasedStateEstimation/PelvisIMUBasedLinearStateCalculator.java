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
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class PelvisIMUBasedLinearStateCalculator implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final BooleanProvider cancelGravityFromAccelerationMeasurement;

   private final FrameVector3DReadOnly gravityVector;

   private final YoFrameVector3D yoLinearAccelerationMeasurementInWorld;
   private final YoFrameVector3D yoLinearAccelerationMeasurement;

   private final BooleanProvider useAccelerometerForEstimation;

   private final ReferenceFrame measurementFrame;

   private final FloatingJointBasics rootJoint;

   private final double estimatorDT;

   // Temporary variables
   private final FrameVector3D linearAccelerationMeasurement = new FrameVector3D();

   private final IMUSensorReadOnly imuProcessedOutput;
   private final IMUBiasProvider imuBiasProvider;

   public PelvisIMUBasedLinearStateCalculator(FullInverseDynamicsStructure inverseDynamicsStructure,
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
      this.rootJoint = inverseDynamicsStructure.getRootJoint();
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

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
   }

   public boolean isEstimationEnabled()
   {
      return useAccelerometerForEstimation.getValue() && imuProcessedOutput != null;
   }

   private void updateLinearAccelerationMeasurement()
   {
      if (!isEstimationEnabled())
         return;

      FrameVector3DReadOnly biasInput = imuBiasProvider.getLinearAccelerationBiasInIMUFrame(imuProcessedOutput);
      Vector3DReadOnly rawInput = imuProcessedOutput.getLinearAccelerationMeasurement();

      linearAccelerationMeasurement.setReferenceFrame(measurementFrame);
      linearAccelerationMeasurement.sub(rawInput, biasInput);

      // Update acceleration in world (minus gravity)
      if (cancelGravityFromAccelerationMeasurement.getValue())
      {
         linearAccelerationMeasurement.changeFrame(worldFrame);
         linearAccelerationMeasurement.add(gravityVector);
      }

      yoLinearAccelerationMeasurementInWorld.setMatchingFrame(linearAccelerationMeasurement);
      yoLinearAccelerationMeasurement.setMatchingFrame(linearAccelerationMeasurement);
   }

   public ReferenceFrame getIMUMeasurementFrame()
   {
      return measurementFrame;
   }

   private final Twist measurementFrameTwist = new Twist();

   public void estimateRootJointLinearVelocity(TwistReadOnly previousRootJointTwistEstimate, FixedFrameVector3DBasics rootJointVelocityUpdate)
   {
      updateLinearAccelerationMeasurement();
      measurementFrameTwist.setIncludingFrame(previousRootJointTwistEstimate);
      measurementFrameTwist.changeFrame(measurementFrame);
      measurementFrameTwist.getLinearPart().scaleAdd(estimatorDT, yoLinearAccelerationMeasurement, measurementFrameTwist.getLinearPart());
      measurementFrameTwist.changeFrame(rootJoint.getFrameAfterJoint());
      rootJointVelocityUpdate.setMatchingFrame(measurementFrameTwist.getLinearPart());
   }

   private final FrameVector3D tempVector = new FrameVector3D();

   public void estimateRootJointPosition(FramePoint3DReadOnly previousRootJointPosition,
                                         TwistReadOnly previousRootJointTwistEstimate,
                                         FixedFramePoint3DBasics rootJointPositionUpdate)
   {
      measurementFrameTwist.setIncludingFrame(previousRootJointTwistEstimate);
      measurementFrameTwist.changeFrame(measurementFrame);
      measurementFrameTwist.getLinearPart().scaleAdd(0.5 * estimatorDT, yoLinearAccelerationMeasurement, measurementFrameTwist.getLinearPart());
      measurementFrameTwist.changeFrame(rootJoint.getFrameAfterJoint());
      tempVector.setIncludingFrame(measurementFrameTwist.getLinearPart());
      tempVector.changeFrame(worldFrame);
      rootJointPositionUpdate.scaleAdd(estimatorDT, tempVector, previousRootJointPosition);
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
