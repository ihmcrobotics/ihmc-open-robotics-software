package us.ihmc.commonWalkingControlModules.controlModules.head;


import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameQuaternion;
import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.DegenerateOrientationControlModule;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class HeadOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameQuaternion orientationToTrack;
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
   
   private final RigidBody head; 
   
   public HeadOrientationControlModule(double controlDT, RigidBody pelvis, RigidBody elevator, RigidBody head, TwistCalculator twistCalculator,
         ReferenceFrame headOrientationFrame, ReferenceFrame chestFrame, HeadOrientationControllerParameters headOrientationControllerParameters, YoVariableRegistry parentRegistry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super("head", new RigidBody[] {}, head, new GeometricJacobian[]{}, twistCalculator, controlDT, parentRegistry);

      this.head = head;
      
      pointTrackingFrame = new OriginAndPointFrame("headPointTrackingFrame", ReferenceFrame.getWorldFrame());

      this.chestFrame = chestFrame;
      this.headOrientationFrame = headOrientationFrame;
      orientationToTrack = new YoFrameQuaternion("headOrientationToTrack", headOrientationFrame, registry);
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

   public RigidBody getHead()
   {
      return head;
   }
   
   @Override
   protected void packDesiredFrameOrientation(FrameOrientation orientationToPack)
   {
      FramePoint positionToPointAt = pointToTrack.getFramePointCopy();
      GeometricJacobian jacobian = super.getJacobian();
      ReferenceFrame headFrame = jacobian.getEndEffectorFrame(); // TODO: change to midEyeFrame?
      pointTrackingFrame.setOriginAndPositionToPointAt(new FramePoint(headFrame), positionToPointAt);

      switch (headTrackingMode.getEnumValue())
      {
         case ORIENTATION :
         {
            orientationToPack.setToZero(orientationToTrack.getReferenceFrame());
            orientationToTrack.get(orientationToPack);

            break;
         }

         case POINT :
         {
            pointTrackingFrame.update();
            pointTrackingFrameFiz.update();

            orientationToPack.setToZero(pointTrackingFrame);
            break;
         }

         default :
            throw new RuntimeException("Case " + headTrackingMode.getEnumValue() + " not handled.");
      }
      orientationToPack.changeFrame(ReferenceFrame.getWorldFrame());

      enforceLimits(orientationToPack);
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
   protected void packDesiredAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(ReferenceFrame.getWorldFrame());
   }

   @Override
   protected void packDesiredAngularAccelerationFeedForward(FrameVector angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(ReferenceFrame.getWorldFrame());
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
