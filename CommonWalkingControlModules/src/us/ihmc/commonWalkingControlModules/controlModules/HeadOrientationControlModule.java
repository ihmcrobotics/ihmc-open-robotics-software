package us.ihmc.commonWalkingControlModules.controlModules;


import org.apache.commons.lang.ArrayUtils;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.OriginAndPointFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientationInMultipleFrames;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePointInMultipleFrames;

public class HeadOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameOrientationInMultipleFrames orientationToTrack;
   private final YoFramePointInMultipleFrames pointToTrack;
   private final ReferenceFrame[] framesToTrackIn;
   private final OriginAndPointFrame pointTrackingFrame;

   private final IntegerYoVariable trackingFrameIndex = new IntegerYoVariable("trackingFrameIndex", registry);
   private final EnumYoVariable<HeadTrackingMode> headTrackingMode = EnumYoVariable.create("headTrackingMode", HeadTrackingMode.class, registry);

   private final DoubleYoVariable yawLimit = new DoubleYoVariable("yawLimit", registry);
   private final DoubleYoVariable pitchLowerLimit = new DoubleYoVariable("pitchLowerLimit", registry);
   private final DoubleYoVariable pitchUpperLimit = new DoubleYoVariable("pitchUpperLimit", registry);
   private final DoubleYoVariable rollLimit = new DoubleYoVariable("rollLimit", registry);

   public HeadOrientationControlModule(GeometricJacobian neckJacobian, TwistCalculator twistCalculator, RigidBody chest, YoVariableRegistry parentRegistry)
   {
      super("head", twistCalculator.getRootBody(), neckJacobian.getEndEffector(), neckJacobian, twistCalculator);
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      pointTrackingFrame = new OriginAndPointFrame("headTrackingFrame", neckJacobian.getEndEffectorFrame());

      framesToTrackIn = new ReferenceFrame[] {chestFrame, twistCalculator.getRootBody().getBodyFixedFrame()};

      orientationToTrack = new YoFrameOrientationInMultipleFrames("headOrientationToTrack", framesToTrackIn, registry);
      pointToTrack = new YoFramePointInMultipleFrames("headPointToTrack", framesToTrackIn, registry);

      headTrackingMode.set(HeadTrackingMode.ORIENTATION);
      trackingFrameIndex.set(getTrackingFrameIndex(chestFrame));

      setHeadOrientationLimits();

      parentRegistry.addChild(registry);
   }

   @Override
   protected FrameOrientation getDesiredFrameOrientation()
   {
      ReferenceFrame referenceFrame = framesToTrackIn[trackingFrameIndex.getIntegerValue()];
      switch (headTrackingMode.getEnumValue())
      {
         case ORIENTATION :
         {
            return orientationToTrack.getOrientationInFrame(referenceFrame).getFrameOrientationCopy();
         }

         case POINT :
         {
            FramePoint positionToPointAt = pointToTrack.getPointInFrame(referenceFrame).getFramePointCopy();
            pointTrackingFrame.setPositionToPointAt(positionToPointAt);
            pointTrackingFrame.update();

            FrameOrientation frameOrientation = new FrameOrientation(pointTrackingFrame);
            enforceLimits(frameOrientation);

            return frameOrientation;
         }

         default :
            throw new RuntimeException("Case " + headTrackingMode.getEnumValue() + " not handled.");
      }
   }

   private void enforceLimits(FrameOrientation orientation)
   {
      ReferenceFrame initialReferenceFrame = orientation.getReferenceFrame();
      orientation.changeFrame(getJacobian().getBaseFrame());

      double[] yawPitchRoll = orientation.getYawPitchRoll();

      yawPitchRoll[0] = MathTools.clipToMinMax(yawPitchRoll[0], -yawLimit.getDoubleValue(), yawLimit.getDoubleValue());
      yawPitchRoll[1] = MathTools.clipToMinMax(yawPitchRoll[1], pitchLowerLimit.getDoubleValue(), pitchUpperLimit.getDoubleValue());
      yawPitchRoll[2] = MathTools.clipToMinMax(yawPitchRoll[2], -rollLimit.getDoubleValue(), rollLimit.getDoubleValue());

      orientation.setYawPitchRoll(yawPitchRoll);
      orientation.changeFrame(initialReferenceFrame);
   }

   private void setHeadOrientationLimits()
   {
      yawLimit.set(Math.PI / 2);
      pitchLowerLimit.set(-Math.PI / 3);
      pitchUpperLimit.set(Math.PI / 4);
      rollLimit.set(Math.PI / 4);

//    yawLimit.set(0.0);
//    pitchLowerLimit.set(0.0);
//    pitchUpperLimit.set(0.0);
//    rollLimit.set(0.0);
   }

   @Override
   protected FrameVector getDesiredAngularVelocity()
   {
      ReferenceFrame frameToTrackIn = framesToTrackIn[trackingFrameIndex.getIntegerValue()];

      return new FrameVector(frameToTrackIn);
   }

   @Override
   protected FrameVector getDesiredAngularAccelerationFeedForward()
   {
      ReferenceFrame frameToTrackIn = framesToTrackIn[trackingFrameIndex.getIntegerValue()];

      return new FrameVector(frameToTrackIn);
   }

   public void setOrientationToTrack(FrameOrientation orientation, ReferenceFrame frameToTrackIn)
   {
      this.trackingFrameIndex.set(getTrackingFrameIndex(frameToTrackIn));
      this.headTrackingMode.set(HeadTrackingMode.ORIENTATION);
      this.orientationToTrack.setFrameOrientation(orientation);
   }

   public void setPointToTrack(FramePoint point, ReferenceFrame frameToTrackIn)
   {
      this.trackingFrameIndex.set(getTrackingFrameIndex(frameToTrackIn));
      this.headTrackingMode.set(HeadTrackingMode.POINT);
      this.pointToTrack.setFramePoint(point);
   }

   private int getTrackingFrameIndex(ReferenceFrame frameToTrackIn)
   {
      int trackingFrameIndex = ArrayUtils.indexOf(framesToTrackIn, frameToTrackIn);
      if (trackingFrameIndex == -1)
         throw new RuntimeException("Frame to track in not found");

      return trackingFrameIndex;
   }

   private enum HeadTrackingMode {ORIENTATION, POINT;}
}
