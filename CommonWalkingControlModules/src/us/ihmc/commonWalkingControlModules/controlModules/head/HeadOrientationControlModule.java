package us.ihmc.commonWalkingControlModules.controlModules.head;


import org.apache.commons.lang.ArrayUtils;

import us.ihmc.commonWalkingControlModules.controlModules.DegenerateOrientationControlModule;
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
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientationInMultipleFrames;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePointInMultipleFrames;

public class HeadOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameOrientationInMultipleFrames orientationToTrack;
   private final YoFramePointInMultipleFrames pointToTrack;
   private final ReferenceFrame[] framesToTrackIn;
   private final OriginAndPointFrame pointTrackingFrame;
   private final DynamicGraphicReferenceFrame pointTrackingFrameFiz;

   private final IntegerYoVariable trackingFrameIndex = new IntegerYoVariable("headTrackingFrameIndex", registry);
   private final EnumYoVariable<HeadTrackingMode> headTrackingMode = EnumYoVariable.create("headTrackingMode", HeadTrackingMode.class, registry);

   private final DoubleYoVariable yawLimit = new DoubleYoVariable("yawLimit", registry);
   private final DoubleYoVariable pitchLowerLimit = new DoubleYoVariable("pitchLowerLimit", registry);
   private final DoubleYoVariable pitchUpperLimit = new DoubleYoVariable("pitchUpperLimit", registry);
   private final DoubleYoVariable rollLimit = new DoubleYoVariable("rollLimit", registry);
   
   public HeadOrientationControlModule(GeometricJacobian neckJacobian, RigidBody pelvis, RigidBody elevator, TwistCalculator twistCalculator,
           ReferenceFrame[] availableHeadOrientationControlFrames, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super("head", new RigidBody[] {elevator, pelvis}, neckJacobian.getEndEffector(), neckJacobian, twistCalculator, parentRegistry);

      pointTrackingFrame = new OriginAndPointFrame("headPointTrackingFrame", ReferenceFrame.getWorldFrame());

      this.framesToTrackIn = availableHeadOrientationControlFrames;

      orientationToTrack = new YoFrameOrientationInMultipleFrames("headOrientationToTrack", framesToTrackIn, registry);
      pointToTrack = new YoFramePointInMultipleFrames("headPointToTrack", framesToTrackIn, registry);
      
      if (dynamicGraphicObjectsListRegistry != null)
      {
         pointTrackingFrameFiz = new DynamicGraphicReferenceFrame(pointTrackingFrame, registry, 0.3);
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList(getClass().getSimpleName());
         dynamicGraphicObjectsList.add(pointTrackingFrameFiz);
         dynamicGraphicObjectsList.hideDynamicGraphicObjects();
      }
      else
      {
         pointTrackingFrameFiz = null;
      }

      setHeadOrientationLimits();
   }

   @Override
   protected FrameOrientation getDesiredFrameOrientation()
   {
      ReferenceFrame referenceFrame = framesToTrackIn[trackingFrameIndex.getIntegerValue()];

      FramePoint positionToPointAt = pointToTrack.getPointInFrame(referenceFrame).getFramePointCopy();
      ReferenceFrame headFrame = jacobian.getEndEffectorFrame(); // TODO: change to midEyeFrame?
      pointTrackingFrame.setOriginAndPositionToPointAt(new FramePoint(headFrame), positionToPointAt);

      FrameOrientation frameOrientation;
      switch (headTrackingMode.getEnumValue())
      {
         case ORIENTATION :
         {
            frameOrientation = orientationToTrack.getOrientationInFrame(referenceFrame).getFrameOrientationCopy();

            break;
         }

         case POINT :
         {
            pointTrackingFrame.update();
            pointTrackingFrameFiz.update();

            frameOrientation = new FrameOrientation(pointTrackingFrame);

            break;
         }

         default :
            throw new RuntimeException("Case " + headTrackingMode.getEnumValue() + " not handled.");
      }

      enforceLimits(frameOrientation);

      return frameOrientation;
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

   public void setOrientationToTrack(FrameOrientation orientation, RigidBody base)
   {
      this.trackingFrameIndex.set(getTrackingFrameIndex(orientation.getReferenceFrame()));
      this.headTrackingMode.set(HeadTrackingMode.ORIENTATION);
      this.orientationToTrack.setFrameOrientation(orientation);
   }

   public void setPointToTrack(FramePoint point, RigidBody base)
   {
      this.trackingFrameIndex.set(getTrackingFrameIndex(point.getReferenceFrame()));
      this.headTrackingMode.set(HeadTrackingMode.POINT);
      this.pointToTrack.setFramePoint(point);
   }

   private int getTrackingFrameIndex(ReferenceFrame frameToTrackIn)
   {
      int trackingFrameIndex = ArrayUtils.indexOf(framesToTrackIn, frameToTrackIn);
      if (trackingFrameIndex == -1)
         throw new RuntimeException("Frame to track in not found: " + frameToTrackIn.getName());

      return trackingFrameIndex;
   }

   private enum HeadTrackingMode {ORIENTATION, POINT;}

   public ReferenceFrame[] getAvailableHeadControlFrames()
   {
      return framesToTrackIn;
   }
}
