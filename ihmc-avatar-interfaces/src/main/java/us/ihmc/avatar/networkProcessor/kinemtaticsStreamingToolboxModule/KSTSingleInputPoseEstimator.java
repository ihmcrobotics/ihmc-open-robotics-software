package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameSpatialVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTSingleInputPoseEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFramePoint3D rawInputPosition;
   private final YoFrameQuaternion rawInputOrientation;
   private final YoFramePoint3D rawExtrapolatedInputPosition;
   private final YoFrameQuaternion rawExtrapolatedInputOrientation;
   private final AlphaFilteredYoFramePoint filteredExtrapolatedInputPosition;
   private final AlphaFilteredYoFrameQuaternion filteredExtrapolatedInputOrientation;

   private final YoFixedFrameSpatialVector rawInputSpatialVelocity;
   private final YoFixedFrameSpatialVector decayingInputSpatialVelocity;
   private final YoDouble inputVelocityDecayFactor;

   public KSTSingleInputPoseEstimator(RigidBodyReadOnly endEffector, DoubleProvider inputsAlphaProvider, YoRegistry registry)
   {
      String namePrefix = endEffector.getName() + "Input";

      rawInputPosition = new YoFramePoint3D(namePrefix + "RawPosition", worldFrame, registry);
      rawInputOrientation = new YoFrameQuaternion(namePrefix + "RawOrientation", worldFrame, registry);
      rawExtrapolatedInputPosition = new YoFramePoint3D(namePrefix + "RawExtrapolatedPosition", worldFrame, registry);
      rawExtrapolatedInputOrientation = new YoFrameQuaternion(namePrefix + "RawExtrapolatedOrientation", worldFrame, registry);
      filteredExtrapolatedInputPosition = new AlphaFilteredYoFramePoint(namePrefix + "FilteredExtrapolatedPosition",
                                                                        "",
                                                                        registry,
                                                                        inputsAlphaProvider,
                                                                        rawExtrapolatedInputPosition);
      filteredExtrapolatedInputOrientation = new AlphaFilteredYoFrameQuaternion(namePrefix + "FilteredExtrapolatedOrientation",
                                                                                "",
                                                                                rawExtrapolatedInputOrientation,
                                                                                inputsAlphaProvider,
                                                                                registry);

      rawInputSpatialVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "RawAngularVelocity", worldFrame, registry),
                                                              new YoFrameVector3D(namePrefix + "RawLinearVelocity", worldFrame, registry));
      decayingInputSpatialVelocity = new YoFixedFrameSpatialVector(new YoFrameVector3D(namePrefix + "DecayingAngularVelocity", worldFrame, registry),
                                                                   new YoFrameVector3D(namePrefix + "DecayingLinearVelocity", worldFrame, registry));
      inputVelocityDecayFactor = new YoDouble(namePrefix + "InputVelocityDecayFactor", registry);
   }

   public void reset()
   {
      rawInputPosition.setToNaN();
      rawInputOrientation.setToNaN();
      rawExtrapolatedInputPosition.setToNaN();
      rawExtrapolatedInputOrientation.setToNaN();
      filteredExtrapolatedInputPosition.setToNaN();
      filteredExtrapolatedInputPosition.reset();
      filteredExtrapolatedInputOrientation.setToNaN();
      filteredExtrapolatedInputOrientation.reset();
      resetVelocity();
   }

   public void resetVelocity()
   {
      rawInputSpatialVelocity.setToZero();
      decayingInputSpatialVelocity.setToZero();
      inputVelocityDecayFactor.set(0.0);
   }

   public void updateInput(FramePose3DReadOnly pose)
   {
      updateInput(pose.getPosition(), pose.getOrientation());
   }

   public void updateInput(FramePoint3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      rawInputPosition.set(position);
      rawInputOrientation.set(orientation);
      rawExtrapolatedInputPosition.set(position);
      rawExtrapolatedInputOrientation.set(orientation);
      filteredExtrapolatedInputPosition.update();
      filteredExtrapolatedInputOrientation.update();
   }

   public void extrapolateInput(double integrationDT)
   {
      KSTTools.integrateLinearVelocity(integrationDT, rawInputPosition, decayingInputSpatialVelocity.getLinearPart(), rawExtrapolatedInputPosition);
      KSTTools.integrateAngularVelocity(integrationDT, rawInputOrientation, decayingInputSpatialVelocity.getAngularPart(), rawExtrapolatedInputOrientation);
      filteredExtrapolatedInputPosition.update();
      filteredExtrapolatedInputOrientation.update();
   }

   public void decayInputVelocity(double decayIncrement)
   {
      double alpha = Math.min(1.0, inputVelocityDecayFactor.getDoubleValue() + decayIncrement);
      inputVelocityDecayFactor.set(alpha);
      decayingInputSpatialVelocity.getLinearPart().interpolate(rawInputSpatialVelocity.getLinearPart(), EuclidCoreTools.zeroVector3D, alpha);
      decayingInputSpatialVelocity.getAngularPart().interpolate(rawInputSpatialVelocity.getAngularPart(), EuclidCoreTools.zeroVector3D, alpha);
   }

   public void estimateVelocity(double dt, FramePose3DReadOnly previousPose, FramePose3DReadOnly currentPose)
   {
      /*
       * We compute the velocity of the inputs and then do a 1st-order extrapolation in the future and
       * update the raw input. This way, if for the next control tick we didn't get any new inputs, the IK
       * keep moving which in result should improve continuity of any motion.
       */
      KSTTools.computeLinearVelocity(dt, previousPose.getPosition(), currentPose.getPosition(), rawInputSpatialVelocity.getLinearPart());
      KSTTools.computeAngularVelocity(dt, previousPose.getOrientation(), currentPose.getOrientation(), rawInputSpatialVelocity.getAngularPart());
      decayingInputSpatialVelocity.set(rawInputSpatialVelocity);
      inputVelocityDecayFactor.set(0.0);
   }

   public FramePoint3DReadOnly getFilteredExtrapolatedInputPosition()
   {
      return filteredExtrapolatedInputPosition;
   }

   public FrameQuaternionReadOnly getFilteredExtrapolatedInputOrientation()
   {
      return filteredExtrapolatedInputOrientation;
   }
}
