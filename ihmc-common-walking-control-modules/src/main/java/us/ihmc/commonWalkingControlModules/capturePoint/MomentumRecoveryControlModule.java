package us.ihmc.commonWalkingControlModules.capturePoint;

import java.awt.Color;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

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
   private static final double defaultMaxIcpError = 0.03;

   private static final boolean showVizByDefault = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoDouble distanceToExtendUpcomingFoothold = new YoDouble("DistanceToExtendUpcomingFoothold", registry);
   private final YoDouble distanceToShrinkSafeAreaSS = new YoDouble("DistanceToShrinkSafeAreaSS", registry);
   private final YoDouble distanceToShrinkSafeAreaIfRecoveringSS = new YoDouble("DistanceToShrinkSafeAreaIfRecoveringSS", registry);
   private final YoDouble distanceToShrinkSafeAreaDS = new YoDouble("DistanceToShrinkSafeAreaDS", registry);
   private final YoDouble distanceToShrinkSafeAreaIfRecoveringDS = new YoDouble("DistanceToShrinkSafeAreaIfRecoveringDS", registry);
   private final YoDouble maxIcpError = new YoDouble("maxIcpError", registry);

   private final YoBoolean usingUpperBodyMomentum = new YoBoolean("usingUpperBodyMomentum", registry);
   private final YoBoolean usingHighMomentumWeight = new YoBoolean("usingHighMomentumWeight", registry);

   private final YoBoolean allowUpperBodyMomentumInSingleSupport = new YoBoolean("allowUpperBodyMomentumInSingleSupport", registry);
   private final YoBoolean allowUpperBodyMomentumInDoubleSupport = new YoBoolean("allowUpperBodyMomentumInDoubleSupport", registry);
   private final YoBoolean allowUsingHighMomentumWeight = new YoBoolean("allowUsingHighMomentumWeight", registry);
   private final YoBoolean alwaysAllowMomentum = new YoBoolean("alwaysAllowMomentum", registry);

   private final YoDouble maxDistanceCMPSupport = new YoDouble("maxDistanceCMPSupport", registry);

   private boolean icpErrorUpToDate = false;
   private boolean robotSideUpToDate = false;
   private boolean capturePointUpToDate = false;
   private boolean supportUpToDate = false;

   private final FrameVector2D icpError = new FrameVector2D();
   private RobotSide supportSide;
   private Footstep nextFootstep;

   private final ConvexPolygonScaler polygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D extendedSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D safeCMPArea = new FrameConvexPolygon2D();

   private final SideDependentList<FrameConvexPolygon2D> defaultFootPolygons;
   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FrameConvexPolygon2D safeArea = new FrameConvexPolygon2D();

   private final FrameConvexPolygon2D tempPolygon1 = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempPolygon2 = new FrameConvexPolygon2D();

   private final YoFrameConvexPolygon2D yoSafeArea;
   private final YoFrameConvexPolygon2D yoSafeCMPArea;
   private final YoFrameConvexPolygon2D yoProjectionArea;
   private final YoFramePoint2D yoCapturePoint;


   public MomentumRecoveryControlModule(SideDependentList<FrameConvexPolygon2D> defaultFootPolygons, double maxAllowedDistanceCMPSupport,
                                        boolean alwaysAllowMomentum, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.defaultFootPolygons = defaultFootPolygons;
      this.alwaysAllowMomentum.set(alwaysAllowMomentum);

      distanceToExtendUpcomingFoothold.set(defaultDistanceToExtendUpcomingFoothold);
      distanceToShrinkSafeAreaSS.set(defaultDistanceToShrinkSafeAreaSS);
      distanceToShrinkSafeAreaIfRecoveringSS.set(defaultDistanceToShrinkSafeAreaIfRecoveringSS);
      distanceToShrinkSafeAreaDS.set(defaultDistanceToShrinkSafeAreaDS);
      distanceToShrinkSafeAreaIfRecoveringDS.set(defaultDistanceToShrinkSafeAreaIfRecoveringDS);
      maxIcpError.set(defaultMaxIcpError);

      allowUpperBodyMomentumInSingleSupport.set(false);
      allowUpperBodyMomentumInDoubleSupport.set(false);
      allowUsingHighMomentumWeight.set(false);
      maxDistanceCMPSupport.set(maxAllowedDistanceCMPSupport);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         String label = getClass().getSimpleName();
         ArtifactList artifacts = new ArtifactList(label);

         yoSafeArea = new YoFrameConvexPolygon2D("SafeArea", worldFrame, 10, registry);
         artifacts.add(new YoArtifactPolygon("Safe Area", yoSafeArea, Color.GREEN, false));

         yoSafeCMPArea = new YoFrameConvexPolygon2D("SafeCMPArea", worldFrame, 10, registry);
         artifacts.add(new YoArtifactPolygon("Safe CMP Area", yoSafeCMPArea, Color.CYAN, false));

         yoCapturePoint = new YoFramePoint2D("CapturePointForMomentum", worldFrame, registry);
         artifacts.add(new YoArtifactPosition("Capture Point For Momentum", yoCapturePoint.getYoX(), yoCapturePoint.getYoY(), GraphicType.BALL, Color.BLUE,
                                              0.01));

         yoProjectionArea = new YoFrameConvexPolygon2D("ProjectionArea", worldFrame, 10, registry);
         artifacts.add(new YoArtifactPolygon("Projection Area", yoProjectionArea, Color.BLUE, false));

         artifacts.setVisible(showVizByDefault);
         yoGraphicsListRegistry.registerArtifactList(artifacts);
      }
      else
      {
         yoSafeArea = null;
         yoSafeCMPArea = null;
         yoCapturePoint = null;
         yoProjectionArea = null;
      }
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
            safeArea.setToNaN();
            capturePoint2d.setToNaN();
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
            safeArea.setToNaN();
            capturePoint2d.setToNaN();
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

      if (yoSafeArea != null)
         yoSafeArea.set(safeArea);
      if (yoCapturePoint != null)
         yoCapturePoint.set(capturePoint2d);
   }

   private void checkIfUseUpperBodyMomentumSingleSupport()
   {
      // icp based fall detection:
      // compute the safe area for the capture point as the support polygon after the step completed
      safeArea.setIncludingFrame(supportPolygon);

      if (nextFootstep != null)
      {
         List<Point2D> predictedContactPoints = nextFootstep.getPredictedContactPoints();
         if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
         {
            tempPolygon1.clear(nextFootstep.getSoleReferenceFrame());
            for (int i = 0; i < predictedContactPoints.size(); i++)
               tempPolygon1.addVertex(predictedContactPoints.get(i));
            tempPolygon1.update();
         }
         else
         {
            tempPolygon1.setIncludingFrame(defaultFootPolygons.get(nextFootstep.getRobotSide()));
         }

         tempPolygon2.setIncludingFrame(nextFootstep.getSoleReferenceFrame(), tempPolygon1);
         tempPolygon2.changeFrameAndProjectToXYPlane(safeArea.getReferenceFrame());
         polygonShrinker.scaleConvexPolygon(tempPolygon2, -distanceToExtendUpcomingFoothold.getDoubleValue(), tempPolygon1);
         safeArea.addVertices(tempPolygon1);
         safeArea.update();
      }

      safeCMPArea.setIncludingFrame(safeArea);

      // hysteresis:
      // shrink the safe area if we are already using upper body momentum
      if (usingUpperBodyMomentum.getBooleanValue())
      {
         polygonShrinker.scaleConvexPolygon(safeArea, distanceToShrinkSafeAreaIfRecoveringSS.getDoubleValue(), tempPolygon1);
         safeArea.setIncludingFrame(tempPolygon1);
      }
      else
      {
         polygonShrinker.scaleConvexPolygon(safeArea, distanceToShrinkSafeAreaSS.getDoubleValue(), tempPolygon1);
         safeArea.setIncludingFrame(tempPolygon1);
      }

      safeArea.changeFrameAndProjectToXYPlane(worldFrame);

      // check if the icp is in the safe area
      capturePoint2d.changeFrameAndProjectToXYPlane(safeArea.getReferenceFrame());
      boolean icpInSafeArea = safeArea.isPointInside(capturePoint2d);

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
      safeArea.setIncludingFrame(supportPolygon);
      safeCMPArea.setIncludingFrame(safeArea);

      // hysteresis:
      // shrink the safe area if we are already using upper body momentum
      if (usingUpperBodyMomentum.getBooleanValue())
      {
         polygonShrinker.scaleConvexPolygon(safeArea, distanceToShrinkSafeAreaIfRecoveringDS.getDoubleValue(), tempPolygon1);
         safeArea.setIncludingFrame(tempPolygon1);
      }
      else
      {
         polygonShrinker.scaleConvexPolygon(safeArea, distanceToShrinkSafeAreaDS.getDoubleValue(), tempPolygon1);
         safeArea.setIncludingFrame(tempPolygon1);
      }

      capturePoint2d.changeFrameAndProjectToXYPlane(safeArea.getReferenceFrame());
      boolean icpInSafeArea = safeArea.isPointInside(capturePoint2d);

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

      if (!capturePointUpToDate)
         throw new RuntimeException("Capture point not up to date.");
      capturePointUpToDate = false;

      if (!supportUpToDate)
         throw new RuntimeException("Support not up to date.");
      supportUpToDate = false;
   }

   public void setICPError(FrameVector2D icpError)
   {
      this.icpError.setIncludingFrame(icpError);
      icpErrorUpToDate = true;
   }

   public void setCapturePoint(FramePoint2D capturePoint2d)
   {
      this.capturePoint2d.setIncludingFrame(capturePoint2d);
      capturePointUpToDate = true;
   }

   public void setSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
      robotSideUpToDate = true;
   }

   public void setSupportPolygon(FrameConvexPolygon2D supportPolygon)
   {
      this.supportPolygon.setIncludingFrame(supportPolygon);
      this.supportPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      supportUpToDate = true;
   }

   public void setNextFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }

   public void getCMPProjectionArea(FrameConvexPolygon2D areaToProjectInto, FrameConvexPolygon2D safeArea)
   {
      if (alwaysAllowMomentum.getBooleanValue() || usingUpperBodyMomentum.getBooleanValue())
      {
         polygonShrinker.scaleConvexPolygon(supportPolygon, -maxDistanceCMPSupport.getDoubleValue(), extendedSupportPolygon);
         areaToProjectInto.setIncludingFrame(extendedSupportPolygon);
         safeCMPArea.changeFrameAndProjectToXYPlane(worldFrame);
         safeArea.setIncludingFrame(safeCMPArea);
      }
      else
      {
         areaToProjectInto.setIncludingFrame(supportPolygon);
         safeArea.clearAndUpdate(worldFrame);
      }

      if (yoSafeCMPArea != null)
         yoSafeCMPArea.set(safeArea);
      if (yoProjectionArea != null)
         yoProjectionArea.set(areaToProjectInto);
   }

   public boolean getUseHighMomentumWeight()
   {
      return usingHighMomentumWeight.getBooleanValue();
   }

}
