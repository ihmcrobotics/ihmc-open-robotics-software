package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * This control module can activate the use of upper body momentum during walking to allow the robot to recover from
 * disturbances or balance on small footholds.
 *
 * There are two kinds of action this module can take:
 *  - increase the momentum weight: this allows the QP to favor balancing over other objectives
 *  - allow the CMP to go outside the support polygon: this will cause lunging with the upper body
 *
 * @author Georg
 *
 */

public class MomentumRecoveryControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable usingUpperBodyMomentum = new BooleanYoVariable("usingUpperBodyMomentum", registry);
   private final BooleanYoVariable usingHighMomentumWeight = new BooleanYoVariable("usingHighMomentumWeight", registry);

   private final BooleanYoVariable allowUpperBodyMomentumInSingleSupport = new BooleanYoVariable("allowUpperBodyMomentumInSingleSupport", registry);
   private final BooleanYoVariable allowUpperBodyMomentumInDoubleSupport = new BooleanYoVariable("allowUpperBodyMomentumInDoubleSupport", registry);
   private final BooleanYoVariable allowUsingHighMomentumWeight = new BooleanYoVariable("allowUsingHighMomentumWeight", registry);

   private final HighLevelHumanoidControllerToolbox momentumBasedController;

   private boolean icpErrorUpToDate = false;
   private boolean robotSideUpToDate = false;
   private boolean nextFootstepUpToDate = false;

   private final FrameVector2d icpError = new FrameVector2d();
   private RobotSide supportSide;
   private Footstep nextFootstep;
   private final DoubleYoVariable maxIcpError = new DoubleYoVariable("maxIcpError", registry);

   public MomentumRecoveryControlModule(HighLevelHumanoidControllerToolbox momentumBasedController, YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;
      maxIcpError.set(0.015);

      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(false);

      parentRegistry.addChild(registry);
   }

   public void compute()
   {
      checkIfUpToDate();

      boolean inSingleSupport = supportSide != null;
      boolean icpErrorSmall = icpError.lengthSquared() < maxIcpError.getDoubleValue() * maxIcpError.getDoubleValue();

      if (inSingleSupport)
      {
         if (allowUpperBodyMomentumInSingleSupport.getBooleanValue())
         {
            checkIfUseUpperBodyMomentumSingleSupport();
         }
         else
         {
            usingUpperBodyMomentum.set(false);
         }
      }
      else
      {
         if (allowUpperBodyMomentumInDoubleSupport.getBooleanValue())
         {
            checkIfUseUpperBodyMomentumDoubleSupport();
         }
         else
         {
            usingUpperBodyMomentum.set(false);
         }
      }

      if (!allowUsingHighMomentumWeight.getBooleanValue())
      {
         usingHighMomentumWeight.set(false);
      }
      else if (usingUpperBodyMomentum.getBooleanValue())
      {
         usingHighMomentumWeight.set(true);
      }
      else
      {
         usingHighMomentumWeight.set(!icpErrorSmall);
      }

   }

   private final FramePoint2d tmpCapturePoint = new FramePoint2d();
   private final ConvexPolygonShrinker polygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempPolygon1 = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempPolygon2 = new FrameConvexPolygon2d();
   private final FramePoint2d capturePoint2d = new FramePoint2d();

   private void checkIfUseUpperBodyMomentumSingleSupport()
   {
      FrameConvexPolygon2d support = momentumBasedController.getBipedSupportPolygons().getFootPolygonInSoleFrame(supportSide);

      // icp based fall detection:
      // compute the safe area for the capture point as the support polygon after the step completed
      safeArea.setIncludingFrame(support);
      momentumBasedController.getDefaultFootPolygon(nextFootstep.getRobotSide(), tempPolygon1);
      tempPolygon2.setIncludingFrameAndUpdate(nextFootstep.getSoleReferenceFrame(), tempPolygon1.getConvexPolygon2d());
      tempPolygon2.changeFrameAndProjectToXYPlane(safeArea.getReferenceFrame());
      polygonShrinker.shrinkConstantDistanceInto(tempPolygon2, -0.05, tempPolygon1);
      safeArea.addVertices(tempPolygon1);
      safeArea.update();

      // hysteresis:
      // shrink the safe area if we are already using upper body momentum
      if (usingUpperBodyMomentum.getBooleanValue())
      {
         polygonShrinker.shrinkConstantDistanceInto(safeArea, 0.05, tempPolygon1);
         safeArea.setIncludingFrameAndUpdate(tempPolygon1);
      }
      else
      {
         polygonShrinker.shrinkConstantDistanceInto(safeArea, 0.02, tempPolygon1);
         safeArea.setIncludingFrameAndUpdate(tempPolygon1);
      }

      safeArea.changeFrameAndProjectToXYPlane(worldFrame);

      // check if the icp is in the safe area
      momentumBasedController.getCapturePoint(capturePoint2d);
      tmpCapturePoint.setIncludingFrame(capturePoint2d);
      tmpCapturePoint.changeFrameAndProjectToXYPlane(safeArea.getReferenceFrame());
      boolean icpInSafeArea = safeArea.isPointInside(tmpCapturePoint);

      if (!icpInSafeArea)
      {
         usingUpperBodyMomentum.set(true);
      }
      else
      {
         usingUpperBodyMomentum.set(false);
      }
   }

   private void checkIfUseUpperBodyMomentumDoubleSupport()
   {
      usingUpperBodyMomentum.set(false);
      // TODO Auto-generated method stub
   }

   public void setICPError(FrameVector2d icpError)
   {
      this.icpError.setIncludingFrame(icpError);
      icpErrorUpToDate = true;
   }

   public void setSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
      robotSideUpToDate = true;
   }

   public void setNextFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
      nextFootstepUpToDate = true;
   }

   private void checkIfUpToDate()
   {
      if (!icpErrorUpToDate)
         throw new RuntimeException("ICP error not up to date.");
      icpErrorUpToDate = false;

      if (!robotSideUpToDate)
         throw new RuntimeException("Support side not up to date.");
      robotSideUpToDate = false;

      if (supportSide != null)
      {
         if (!nextFootstepUpToDate)
            throw new RuntimeException("Next footstep not up to date.");
      }
      else
         nextFootstepUpToDate = false;

   }

   public void updateIcpControlModule(ICPBasedLinearMomentumRateOfChangeControlModule icpBasedLinearMomentumRateOfChangeControlModule)
   {
      if (usingHighMomentumWeight.getBooleanValue())
      {
         icpBasedLinearMomentumRateOfChangeControlModule.setHighMomentumWeight();
      }
      else
      {
         icpBasedLinearMomentumRateOfChangeControlModule.setDefaultMomentumWeight();
      }

      if (usingUpperBodyMomentum.getBooleanValue())
      {
         icpBasedLinearMomentumRateOfChangeControlModule.keepCMPInsideSupportPolygon(false);
      }
      else
      {
         icpBasedLinearMomentumRateOfChangeControlModule.keepCMPInsideSupportPolygon(true);
      }
   }

}
