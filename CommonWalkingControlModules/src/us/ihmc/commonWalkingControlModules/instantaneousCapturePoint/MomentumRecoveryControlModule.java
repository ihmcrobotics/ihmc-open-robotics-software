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
   private static final double defaultDistanceToExtendUpcomingFoothold = 0.05;
   private static final double defaultDistanceToShrinkSafeAreaSS = 0.02;
   private static final double defaultDistanceToShrinkSafeAreaIfRecoveringSS = 0.05;
   private static final double defaultDistanceToShrinkSafeAreaDS = 0.01;
   private static final double defaultDistanceToShrinkSafeAreaIfRecoveringDS = 0.05;
   private static final double defaultMaxIcpError = 0.015;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable distanceToExtendUpcomingFoothold = new DoubleYoVariable("DistanceToExtendUpcomingFoothold", registry);
   private final DoubleYoVariable distanceToShrinkSafeAreaSS = new DoubleYoVariable("DistanceToShrinkSafeAreaSS", registry);
   private final DoubleYoVariable distanceToShrinkSafeAreaIfRecoveringSS = new DoubleYoVariable("DistanceToShrinkSafeAreaIfRecoveringSS", registry);
   private final DoubleYoVariable distanceToShrinkSafeAreaDS = new DoubleYoVariable("DistanceToShrinkSafeAreaDS", registry);
   private final DoubleYoVariable distanceToShrinkSafeAreaIfRecoveringDS = new DoubleYoVariable("DistanceToShrinkSafeAreaIfRecoveringDS", registry);
   private final DoubleYoVariable maxIcpError = new DoubleYoVariable("maxIcpError", registry);

   private final BooleanYoVariable usingUpperBodyMomentum = new BooleanYoVariable("usingUpperBodyMomentum", registry);
   private final BooleanYoVariable usingHighMomentumWeight = new BooleanYoVariable("usingHighMomentumWeight", registry);

   private final BooleanYoVariable allowUpperBodyMomentumInSingleSupport = new BooleanYoVariable("allowUpperBodyMomentumInSingleSupport", registry);
   private final BooleanYoVariable allowUpperBodyMomentumInDoubleSupport = new BooleanYoVariable("allowUpperBodyMomentumInDoubleSupport", registry);
   private final BooleanYoVariable allowUsingHighMomentumWeight = new BooleanYoVariable("allowUsingHighMomentumWeight", registry);

   private final DoubleYoVariable maxDistanceCMPSupport = new DoubleYoVariable("maxDistanceCMPSupport", registry);

   private final HighLevelHumanoidControllerToolbox momentumBasedController;

   private boolean icpErrorUpToDate = false;
   private boolean robotSideUpToDate = false;

   private final FrameVector2d icpError = new FrameVector2d();
   private RobotSide supportSide;
   private Footstep nextFootstep;

   private final ConvexPolygonShrinker polygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d extendedSupportPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d safeCMPArea = new FrameConvexPolygon2d();

   public MomentumRecoveryControlModule(HighLevelHumanoidControllerToolbox momentumBasedController, double maxAllowedDistanceCMPSupport, YoVariableRegistry parentRegistry)
   {
      this.momentumBasedController = momentumBasedController;

      distanceToExtendUpcomingFoothold.set(defaultDistanceToExtendUpcomingFoothold);
      distanceToShrinkSafeAreaSS.set(defaultDistanceToShrinkSafeAreaSS);
      distanceToShrinkSafeAreaIfRecoveringSS.set(defaultDistanceToShrinkSafeAreaIfRecoveringSS);
      distanceToShrinkSafeAreaDS.set(defaultDistanceToShrinkSafeAreaDS);
      distanceToShrinkSafeAreaIfRecoveringDS.set(defaultDistanceToShrinkSafeAreaIfRecoveringDS);
      maxIcpError.set(defaultMaxIcpError);

      allowUpperBodyMomentumInSingleSupport.set(false);
      allowUpperBodyMomentumInDoubleSupport.set(false);
      maxDistanceCMPSupport.set(maxAllowedDistanceCMPSupport);

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

      if (nextFootstep != null)
      {
         // TODO: check if the next footstep has expected contact points and use them instead of the default polygon
         momentumBasedController.getDefaultFootPolygon(nextFootstep.getRobotSide(), tempPolygon1);
         tempPolygon2.setIncludingFrameAndUpdate(nextFootstep.getSoleReferenceFrame(), tempPolygon1.getConvexPolygon2d());
         tempPolygon2.changeFrameAndProjectToXYPlane(safeArea.getReferenceFrame());
         polygonShrinker.shrinkConstantDistanceInto(tempPolygon2, -distanceToExtendUpcomingFoothold.getDoubleValue(), tempPolygon1);
         safeArea.addVertices(tempPolygon1);
         safeArea.update();
      }

      safeCMPArea.setIncludingFrameAndUpdate(safeArea);

      // hysteresis:
      // shrink the safe area if we are already using upper body momentum
      if (usingUpperBodyMomentum.getBooleanValue())
      {
         polygonShrinker.shrinkConstantDistanceInto(safeArea, distanceToShrinkSafeAreaIfRecoveringSS.getDoubleValue(), tempPolygon1);
         safeArea.setIncludingFrameAndUpdate(tempPolygon1);
      }
      else
      {
         polygonShrinker.shrinkConstantDistanceInto(safeArea, distanceToShrinkSafeAreaSS.getDoubleValue(), tempPolygon1);
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
      momentumBasedController.getCapturePoint(capturePoint2d);
      capturePoint2d.changeFrameAndProjectToXYPlane(worldFrame);
      FrameConvexPolygon2d support = momentumBasedController.getBipedSupportPolygons().getSupportPolygonInWorld();
      safeArea.setIncludingFrameAndUpdate(support);

      safeCMPArea.setIncludingFrameAndUpdate(safeArea);

      // hysteresis:
      // shrink the safe area if we are already using upper body momentum
      if (usingUpperBodyMomentum.getBooleanValue())
      {
         polygonShrinker.shrinkConstantDistanceInto(safeArea, distanceToShrinkSafeAreaIfRecoveringDS.getDoubleValue(), tempPolygon1);
         safeArea.setIncludingFrameAndUpdate(tempPolygon1);
      }
      else
      {
         polygonShrinker.shrinkConstantDistanceInto(safeArea, distanceToShrinkSafeAreaDS.getDoubleValue(), tempPolygon1);
         safeArea.setIncludingFrameAndUpdate(tempPolygon1);
      }

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

   private void checkIfUpToDate()
   {
      if (!icpErrorUpToDate)
         throw new RuntimeException("ICP error not up to date.");
      icpErrorUpToDate = false;

      if (!robotSideUpToDate)
         throw new RuntimeException("Support side not up to date.");
      robotSideUpToDate = false;
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
   }

   public void getCMPProjectionArea(FrameConvexPolygon2d areaToProjectInto, FrameConvexPolygon2d safeArea)
   {
      supportPolygon.set(momentumBasedController.getBipedSupportPolygons().getSupportPolygonInWorld());

      if (usingUpperBodyMomentum.getBooleanValue())
      {
         polygonShrinker.shrinkConstantDistanceInto(supportPolygon, -maxDistanceCMPSupport.getDoubleValue(), extendedSupportPolygon);
         areaToProjectInto.setIncludingFrameAndUpdate(extendedSupportPolygon);
         safeCMPArea.changeFrameAndProjectToXYPlane(worldFrame);
         safeArea.setIncludingFrameAndUpdate(safeCMPArea);
      }
      else
      {
         areaToProjectInto.setIncludingFrameAndUpdate(supportPolygon);
         safeArea.clearAndUpdate(worldFrame);
      }
   }

   public boolean getUseHighMomentumWeight()
   {
      return usingHighMomentumWeight.getBooleanValue();
   }

}
