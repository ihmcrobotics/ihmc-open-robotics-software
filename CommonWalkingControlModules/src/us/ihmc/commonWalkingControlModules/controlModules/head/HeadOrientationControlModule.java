package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.configurations.HeadOrientationControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.DegenerateOrientationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.OriginAndPointFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;


public class HeadOrientationControlModule extends DegenerateOrientationControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFrameQuaternion orientationToTrack;
   private final YoFramePoint pointToTrack;
   private final ReferenceFrame chestFrame;
   private final ReferenceFrame headFrame;
   private final OriginAndPointFrame pointTrackingFrame;
   private final YoGraphicReferenceFrame pointTrackingFrameFiz;

   private enum HeadTrackingMode {ORIENTATION, POINT}

   private final EnumYoVariable<HeadTrackingMode> headTrackingMode = EnumYoVariable.create("headTrackingMode", HeadTrackingMode.class, registry);

   private final DoubleYoVariable yawLimit = new DoubleYoVariable("yawLimit", registry);
   private final DoubleYoVariable pitchLowerLimit = new DoubleYoVariable("pitchLowerLimit", registry);
   private final DoubleYoVariable pitchUpperLimit = new DoubleYoVariable("pitchUpperLimit", registry);
   private final DoubleYoVariable rollLimit = new DoubleYoVariable("rollLimit", registry);

   private final RigidBody head;

   /*
    * TODO Sylvain. In walking, the head in controlled with respect to the
    * elevator (see WalkingHighLevelHumanoidController.setupManagers()). This
    * causes the head to have a little lag when rotating the pelvis (around z
    * only). The best option would be to control the head with respect to two
    * bases: for the pitch and roll with respect to the elevator, and for the
    * yaw with respect to the pelvis.
    */

   public HeadOrientationControlModule(MomentumBasedController momentumBasedController, ReferenceFrame headOrientationExpressedInFrame,
           HeadOrientationControllerParameters headOrientationControllerParameters, YoVariableRegistry parentRegistry,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(momentumBasedController, headOrientationExpressedInFrame, headOrientationControllerParameters, null, parentRegistry, yoGraphicsListRegistry);
   }

   public HeadOrientationControlModule(MomentumBasedController momentumBasedController, ReferenceFrame headOrientationExpressedInFrame,
           HeadOrientationControllerParameters headOrientationControllerParameters, YoOrientationPIDGains gains, YoVariableRegistry parentRegistry,
           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super("head", new RigidBody[]
      {
      }, momentumBasedController.getFullRobotModel().getHead(), new GeometricJacobian[]
      {
      }, momentumBasedController.getTwistCalculator(), momentumBasedController.getControlDT(), gains, parentRegistry);

      FullRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      this.head = fullRobotModel.getHead();

      pointTrackingFrame = new OriginAndPointFrame("headPointTrackingFrame", worldFrame);

      this.chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      this.headFrame = head.getBodyFixedFrame();
      orientationToTrack = new YoFrameQuaternion("headOrientationToTrack", headOrientationExpressedInFrame, registry);
      pointToTrack = new YoFramePoint("headPointToTrack", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         pointTrackingFrameFiz = new YoGraphicReferenceFrame(pointTrackingFrame, registry, 0.3);
         YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
         yoGraphicsList.add(pointTrackingFrameFiz);
         yoGraphicsList.hideYoGraphics();
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

   private final FramePoint headPosition = new FramePoint();

   @Override
   protected void packDesiredFrameOrientation(FrameOrientation orientationToPack)
   {
      FramePoint positionToPointAt = pointToTrack.getFramePointCopy();
      headPosition.setToZero(headFrame);
      pointTrackingFrame.setOriginAndPositionToPointAt(headPosition, positionToPointAt);

      switch (headTrackingMode.getEnumValue())
      {
         case ORIENTATION :
         {
            orientationToPack.setToZero(orientationToTrack.getReferenceFrame());
            orientationToTrack.getFrameOrientationIncludingFrame(orientationToPack);

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

      orientationToPack.changeFrame(worldFrame);

      enforceLimits(orientationToPack);
   }

   private void enforceLimits(FrameOrientation orientation)
   {
      ReferenceFrame initialReferenceFrame = orientation.getReferenceFrame();

      // Limit pitch with respect to the chest frame
      {
         orientation.changeFrame(chestFrame);
         double[] yawPitchRoll = orientation.getYawPitchRoll();
         yawPitchRoll[1] = MathTools.clipToMinMax(yawPitchRoll[1], pitchLowerLimit.getDoubleValue(), pitchUpperLimit.getDoubleValue());

         orientation.setYawPitchRoll(yawPitchRoll);
      }

      // Limit roll and yaw
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
      pitchUpperLimit.set(headOrientationControllerParameters.getNeckPitchUpperLimit());
      pitchLowerLimit.set(headOrientationControllerParameters.getNeckPitchLowerLimit());
      rollLimit.set(headOrientationControllerParameters.getHeadRollLimit());
   }

   @Override
   protected void packDesiredAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(worldFrame);
   }

   @Override
   protected void packDesiredAngularAccelerationFeedForward(FrameVector angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(worldFrame);
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
}
