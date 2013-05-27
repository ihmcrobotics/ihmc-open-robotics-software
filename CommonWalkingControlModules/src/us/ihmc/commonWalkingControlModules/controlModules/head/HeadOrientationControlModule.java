package us.ihmc.commonWalkingControlModules.controlModules.head;


import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
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
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class HeadOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameOrientation orientationToTrack;
   private final YoFramePoint pointToTrack;
   private final ReferenceFrame chestFrame;
   private final ReferenceFrame headOrientationFrame;
   private final OriginAndPointFrame pointTrackingFrame;
   private final DynamicGraphicReferenceFrame pointTrackingFrameFiz;

   private final EnumYoVariable<HeadTrackingMode> headTrackingMode = EnumYoVariable.create("headTrackingMode", HeadTrackingMode.class, registry);

   private final DoubleYoVariable yawLimit = new DoubleYoVariable("yawLimit", registry);
   private final DoubleYoVariable pitchLowerLimit = new DoubleYoVariable("pitchLowerLimit", registry);
   private final DoubleYoVariable pitchUpperLimit = new DoubleYoVariable("pitchUpperLimit", registry);
   private final DoubleYoVariable rollLimit = new DoubleYoVariable("rollLimit", registry);
   
   public HeadOrientationControlModule(GeometricJacobian neckJacobian, RigidBody pelvis, RigidBody elevator, TwistCalculator twistCalculator,
         ReferenceFrame headOrientationFrame, ReferenceFrame chestFrame, HeadOrientationControllerParameters headOrientationControllerParameters, YoVariableRegistry parentRegistry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super("head", new RigidBody[] {elevator, pelvis}, neckJacobian.getEndEffector(), new GeometricJacobian[]{neckJacobian}, twistCalculator, parentRegistry);

      pointTrackingFrame = new OriginAndPointFrame("headPointTrackingFrame", ReferenceFrame.getWorldFrame());

      this.chestFrame = chestFrame;
      this.headOrientationFrame = headOrientationFrame;
      orientationToTrack = new YoFrameOrientation("headOrientationToTrack", headOrientationFrame, registry);
      pointToTrack = new YoFramePoint("headPointToTrack", ReferenceFrame.getWorldFrame(), registry);
      
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

      setHeadOrientationLimits(headOrientationControllerParameters);
   }

   @Override
   protected FrameOrientation getDesiredFrameOrientation()
   {
      FramePoint positionToPointAt = pointToTrack.getFramePointCopy();
      GeometricJacobian jacobian = super.getJacobian();
      ReferenceFrame headFrame = jacobian .getEndEffectorFrame(); // TODO: change to midEyeFrame?
      pointTrackingFrame.setOriginAndPositionToPointAt(new FramePoint(headFrame), positionToPointAt);

      FrameOrientation frameOrientation;
      switch (headTrackingMode.getEnumValue())
      {
         case ORIENTATION :
         {
            frameOrientation = orientationToTrack.getFrameOrientationCopy();

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
      frameOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      enforceLimits(frameOrientation);

      return frameOrientation;
   }

   private void enforceLimits(FrameOrientation orientation)
   {
      ReferenceFrame initialReferenceFrame = orientation.getReferenceFrame();

      //Limit pitch with respect to the chest frame
      {
         orientation.changeFrame(chestFrame);
         double[] yawPitchRoll = orientation.getYawPitchRoll();
         yawPitchRoll[1] = MathTools.clipToMinMax(yawPitchRoll[1], pitchLowerLimit.getDoubleValue(), pitchUpperLimit.getDoubleValue());

         orientation.setYawPitchRoll(yawPitchRoll);
      }

      //Limit roll and yaw
      {
         orientation.changeFrame(getJacobian().getBaseFrame());

         double[] yawPitchRoll = orientation.getYawPitchRoll();
         yawPitchRoll[0] = MathTools.clipToMinMax(yawPitchRoll[0], -yawLimit.getDoubleValue(), yawLimit.getDoubleValue());
         yawPitchRoll[2] = MathTools.clipToMinMax(yawPitchRoll[2], -rollLimit.getDoubleValue(), rollLimit.getDoubleValue());

         orientation.setYawPitchRoll(yawPitchRoll);
         orientation.changeFrame(initialReferenceFrame);
      }
   }

   private void setHeadOrientationLimits(HeadOrientationControllerParameters headOrientationControllerParameters)
   {
      yawLimit.set(headOrientationControllerParameters.getHeadYawLimit());
      pitchUpperLimit.set(headOrientationControllerParameters.getUpperNeckPitchLimit());
      pitchLowerLimit.set(headOrientationControllerParameters.getLowerNeckPitchLimit());
      rollLimit.set(headOrientationControllerParameters.getHeadRollLimit());

//    yawLimit.set(0.0);
//    pitchLowerLimit.set(0.0);
//    pitchUpperLimit.set(0.0);
//    rollLimit.set(0.0);
   }

   @Override
   protected FrameVector getDesiredAngularVelocity()
   {
      return new FrameVector(ReferenceFrame.getWorldFrame());
   }

   @Override
   protected FrameVector getDesiredAngularAccelerationFeedForward()
   {
      return new FrameVector(ReferenceFrame.getWorldFrame());
   }

   public void setOrientationToTrack(FrameOrientation orientation)
   {
      this.headTrackingMode.set(HeadTrackingMode.ORIENTATION);
      this.orientationToTrack.set(orientation);
   }

   public void setPointToTrack(FramePoint point)
   {
      this.headTrackingMode.set(HeadTrackingMode.POINT);
      this.pointToTrack.set(point);
   }

   private enum HeadTrackingMode {ORIENTATION, POINT;}

   public ReferenceFrame getHeadOrientationFrame()
   {
      return headOrientationFrame;
   }
}
