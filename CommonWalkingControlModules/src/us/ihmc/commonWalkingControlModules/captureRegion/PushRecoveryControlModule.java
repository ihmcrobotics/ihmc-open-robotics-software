package us.ihmc.commonWalkingControlModules.captureRegion;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ICPAndMomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public class PushRecoveryControlModule
{
   private static final double DOUBLESUPPORT_SUPPORT_POLYGON_SCALE = 0.95;
   private static final double FAST_SWING_TIME = 0.4;

   private final BooleanYoVariable enablePushRecoveryFromDoubleSupport;
   private final ICPAndMomentumBasedController icpAndMomentumBasedController;
   private final MomentumBasedController momentumBasedController;
   private final OrientationStateVisualizer orientationStateVisualizer;
   private final FootstepAdjustor footstepAdjustor;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final BooleanYoVariable footstepWasProjectedInCaptureRegion;
   private final BooleanYoVariable enablePushRecovery;

   private boolean recoveringFromDoubleSupportFall;
   private Footstep recoverFromDoubleSupportFallFootStep;
   private BooleanYoVariable readyToGrabNextFootstep;
   private Footstep oldFootStep;

   public PushRecoveryControlModule(DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry registry,
         MomentumBasedController momentumBasedController, WalkingControllerParameters walkingControllerParameters, BooleanYoVariable readyToGrabNextFootstep,
         Footstep initialFootstep, ICPAndMomentumBasedController icpAndMomentumBasedController, BooleanYoVariable enablePushRecovery)
   {
      this.momentumBasedController = momentumBasedController;
      this.readyToGrabNextFootstep = readyToGrabNextFootstep;
      this.oldFootStep = initialFootstep;
      this.icpAndMomentumBasedController = icpAndMomentumBasedController;
      this.enablePushRecovery = enablePushRecovery;
      
      this.enablePushRecoveryFromDoubleSupport = new BooleanYoVariable("enablePushRecoveryFromDoubleSupport", registry);
      this.enablePushRecoveryFromDoubleSupport.set(false);

      if (enablePushRecoveryFromDoubleSupport.getBooleanValue())
      {
         // TODO find a way to set the swing time in the controller to the FAST_SWING_TIME or even less if necessary
         //      but maybe do not put in the constructor 
      }

      captureRegionCalculator = new OneStepCaptureRegionCalculator(momentumBasedController.getReferenceFrames(), walkingControllerParameters, registry,
            dynamicGraphicObjectsListRegistry);
      footstepAdjustor = new FootstepAdjustor(registry, dynamicGraphicObjectsListRegistry);
      orientationStateVisualizer = new OrientationStateVisualizer(dynamicGraphicObjectsListRegistry, registry);

      footstepWasProjectedInCaptureRegion = new BooleanYoVariable("footstepWasProjectedInCaptureRegion", registry);

      initialize();
   }

   public void initialize()
   {
      recoveringFromDoubleSupportFall = false;
      footstepWasProjectedInCaptureRegion.set(false);
      recoverFromDoubleSupportFallFootStep = null;
      captureRegionCalculator.hideCaptureRegion();
   }

   public class IsFallingFromDoubleSupport implements StateTransitionCondition
   {
      private final FramePoint2d capturePoint2d = new FramePoint2d();
      private RobotSide swingSide = null;
      private RobotSide transferToSide = null;

      private Transform3D fromWorldToPelvis = new Transform3D();
      private Transform3D scaleTransformation = new Transform3D();
      private FrameConvexPolygon2d reducedSupportPolygon;
      private ReferenceFrame midFeetZUp;
      private double capturePointYAxis;
      private FrameVector projectedCapturePoint;

      public IsFallingFromDoubleSupport(RobotSide robotSide)
      {
         this.transferToSide = robotSide;
         this.scaleTransformation.setScale(DOUBLESUPPORT_SUPPORT_POLYGON_SCALE);
         this.reducedSupportPolygon = new FrameConvexPolygon2d(icpAndMomentumBasedController.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp());
         this.projectedCapturePoint = new FrameVector(worldFrame, 0, 0, 0);
      }

      @Override
      public boolean checkCondition()
      {
         if (enablePushRecovery.getBooleanValue())
         {
            if (getDoubleSupportEnableState())
            {
               midFeetZUp = icpAndMomentumBasedController.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp().getReferenceFrame();

               // get current robot status
               reducedSupportPolygon.changeFrame(midFeetZUp);
               reducedSupportPolygon.setAndUpdate(icpAndMomentumBasedController.getBipedSupportPolygons().getSupportPolygonInMidFeetZUp());
               icpAndMomentumBasedController.getCapturePoint().getFrameTuple2dIncludingFrame(capturePoint2d);

               capturePoint2d.changeFrame(midFeetZUp);
               reducedSupportPolygon.applyTransform(scaleTransformation);

               // update the visualization
               fromWorldToPelvis = momentumBasedController.getFullRobotModel().getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(worldFrame);
               orientationStateVisualizer.updatePelvisReferenceFrame(fromWorldToPelvis);
               orientationStateVisualizer.updateReducedSupportPolygon(reducedSupportPolygon);

               if (!reducedSupportPolygon.isPointInside(capturePoint2d) && recoverFromDoubleSupportFallFootStep == null)
               {
                  System.out.println("Robot is falling from double support");
                  projectedCapturePoint.changeFrame(capturePoint2d.getReferenceFrame());
                  projectedCapturePoint.set(capturePoint2d.getX(), capturePoint2d.getY(), 0);
                  projectedCapturePoint.changeFrame(momentumBasedController.getFullRobotModel().getPelvis().getBodyFixedFrame());
                  capturePointYAxis = projectedCapturePoint.getY();
                  if (capturePointYAxis >= 0)
                  {
                     swingSide = RobotSide.LEFT;
                  }
                  else
                  {
                     swingSide = RobotSide.RIGHT;
                  }

                  if (transferToSide == swingSide)
                  {
                     return false;
                  }

                  captureRegionCalculator.calculateCaptureRegion(
                        swingSide,
                        FAST_SWING_TIME,
                        capturePoint2d,
                        icpAndMomentumBasedController.getOmega0(),
                        computeFootPolygon(swingSide.getOppositeSide(),
                              momentumBasedController.getReferenceFrames().getAnkleZUpFrame(swingSide.getOppositeSide())));

                  footstepAdjustor.adjustFootstep(oldFootStep, captureRegionCalculator.getCaptureRegion());
                  readyToGrabNextFootstep.set(false);
                  momentumBasedController.getUpcomingSupportLeg().set(transferToSide.getOppositeSide());
                  recoverFromDoubleSupportFallFootStep = oldFootStep;
                  recoveringFromDoubleSupportFall = true;

                  return true;
               }

               // we need this to reset the reference frame 
               reducedSupportPolygon.changeFrame(worldFrame);
               return false;
            }
         }

         return false;
      }
   }

   public boolean checkAndUpdateFootstep(RobotSide swingSide, double swingTimeRemaining, FramePoint2d capturePoint2d, Footstep nextFootstep, double omega0,
         FrameConvexPolygon2d footPolygon)
   {
      if (enablePushRecovery.getBooleanValue())
      {
         if (footstepWasProjectedInCaptureRegion.getBooleanValue())
         {
            // can not re-plan again
            return false;
         }

         captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, capturePoint2d, omega0, footPolygon);
         footstepWasProjectedInCaptureRegion.set(footstepAdjustor.adjustFootstep(nextFootstep, captureRegionCalculator.getCaptureRegion()));

         return footstepWasProjectedInCaptureRegion.getBooleanValue();
      }
      return false;
   }

   private FrameConvexPolygon2d computeFootPolygon(RobotSide robotSide, ReferenceFrame referenceFrame)
   {
      final List<FramePoint> tempContactPoints = new ArrayList<FramePoint>();
      final FrameConvexPolygon2d tempFootPolygon = new FrameConvexPolygon2d(worldFrame);

      momentumBasedController.getContactPoints(momentumBasedController.getContactablePlaneFeet().get(robotSide), tempContactPoints);
      tempFootPolygon.setIncludingFrameByProjectionOntoXYPlaneAndUpdate(referenceFrame, tempContactPoints);

      return tempFootPolygon;
   }

   public boolean getDoubleSupportEnableState()
   {
      return enablePushRecoveryFromDoubleSupport.getBooleanValue();
   }

   public boolean getIsRecoveringFromDoubleSupportFall()
   {
      return recoveringFromDoubleSupportFall;
   }

   public Footstep getRecoverFromDoubleSupportFootStep()
   {
      return recoverFromDoubleSupportFallFootStep;
   }

   public void setRecoveringFromDoubleSupportState(boolean value)
   {
      recoveringFromDoubleSupportFall = value;
   }

   public void setRecoverFromDoubleSupportFootStep(Footstep recoverFootStep)
   {
      recoverFromDoubleSupportFallFootStep = recoverFootStep;
   }

}
