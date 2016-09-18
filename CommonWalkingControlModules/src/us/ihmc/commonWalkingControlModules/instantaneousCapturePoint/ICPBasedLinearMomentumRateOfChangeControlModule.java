package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance.Purple;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.WrenchDistributorTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class ICPBasedLinearMomentumRateOfChangeControlModule extends LinearMomentumRateOfChangeControlModule
{
   private final ICPProportionalController icpProportionalController;

   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();

   private final BooleanYoVariable desiredCMPinSafeArea = new BooleanYoVariable("DesiredCMPinSafeArea", registry);

   private final YoFramePoint2d yoUnprojectedDesiredCMP = new YoFramePoint2d("unprojectedDesiredCMP", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoSafeAreaPolygon = new YoFrameConvexPolygon2d("yoSafeAreaPolygon", worldFrame, 10, registry);
   private final YoFrameConvexPolygon2d yoProjectionPolygon = new YoFrameConvexPolygon2d("yoProjectionPolygon", worldFrame, 10, registry);

   private final CMPProjector cmpProjector;
   private final FrameConvexPolygon2d areaToProjectInto = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d safeArea = new FrameConvexPolygon2d();

   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, ICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(referenceFrames, bipedSupportPolygons, controlDT, totalMass, gravityZ, icpControlGains, parentRegistry, yoGraphicsListRegistry, true);
   }

   public ICPBasedLinearMomentumRateOfChangeControlModule(CommonHumanoidReferenceFrames referenceFrames, BipedSupportPolygons bipedSupportPolygons,
         double controlDT, double totalMass, double gravityZ, ICPControlGains icpControlGains, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, boolean use2DProjection)
   {
      super("ICPBased", referenceFrames, bipedSupportPolygons, totalMass, gravityZ, parentRegistry);

      if (use2DProjection)
         cmpProjector = new SmartCMPProjector(yoGraphicsListRegistry, registry);
      else
         cmpProjector = new SmartCMPPlanarProjector(registry);

      icpProportionalController = new ICPProportionalController(icpControlGains, controlDT, registry);

      if (yoGraphicsListRegistry != null)
      {
         String graphicListName = getClass().getSimpleName();
         YoGraphicPosition unprojectedDesiredCMPViz = new YoGraphicPosition("Unprojected Desired CMP", yoUnprojectedDesiredCMP, 0.008, Purple(), GraphicType.BALL_WITH_ROTATED_CROSS);
         YoArtifactPosition artifact = unprojectedDesiredCMPViz.createArtifact();
         artifact.setVisible(false);
         yoGraphicsListRegistry.registerArtifact(graphicListName, artifact);

//         YoArtifactPolygon yoSafeArea = new YoArtifactPolygon("SafeArea", yoSafeAreaPolygon, Color.GREEN, false);
//         yoGraphicsListRegistry.registerArtifact(graphicListName, yoSafeArea);
//
//         YoArtifactPolygon yoProjectionArea = new YoArtifactPolygon("ProjectionArea", yoProjectionPolygon, Color.RED, false);
//         yoGraphicsListRegistry.registerArtifact(graphicListName, yoProjectionArea);
      }
      yoUnprojectedDesiredCMP.setToNaN();
   }


   @Override
   public void computeCMPInternal(FramePoint2d desiredCMPPreviousValue)
   {
      if (supportSide != supportLegPreviousTick.getEnumValue())
      {
         icpProportionalController.reset();
      }

      desiredCMP.set(icpProportionalController.doProportionalControl(desiredCMPPreviousValue, capturePoint, desiredCapturePoint,
            finalDesiredCapturePoint, desiredCapturePointVelocity, perfectCMP, omega0));

      yoUnprojectedDesiredCMP.set(desiredCMP);

      // do projection here:
      if (!areaToProjectInto.isEmpty())
      {
         desiredCMPinSafeArea.set(safeArea.isPointInside(desiredCMP));
         if (safeArea.isPointInside(desiredCMP))
         {
            supportPolygon.setIncludingFrameAndUpdate(bipedSupportPolygons.getSupportPolygonInMidFeetZUp());
            areaToProjectInto.setIncludingFrameAndUpdate(supportPolygon);
         }

         cmpProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, areaToProjectInto, finalDesiredCapturePoint, desiredCMP);
         if (cmpProjector.getWasCMPProjected())
            icpProportionalController.bleedOffIntegralTerm();
      }
   }

   @Override
   public void setCMPProjectionArea(FrameConvexPolygon2d areaToProjectInto, FrameConvexPolygon2d safeArea)
   {
      this.areaToProjectInto.setIncludingFrameAndUpdate(areaToProjectInto);
      this.safeArea.setIncludingFrameAndUpdate(safeArea);

      yoSafeAreaPolygon.setFrameConvexPolygon2d(safeArea);
      yoProjectionPolygon.setFrameConvexPolygon2d(areaToProjectInto);
   }

   // implementing methods to satisfy super class
   public void setDoubleSupportDuration(double doubleSupportDuration)
   {}

   public void setSingleSupportDuration(double singleSupportDuration)
   {}

   public void clearPlan()
   {}

   public void addFootstepToPlan(Footstep footstep)
   {}

   public void initializeForStanding()
   {}

   public void initializeForSingleSupport()
   {}

   public void initializeForTransfer()
   {}

   public boolean getUpcomingFootstepSolution(Footstep footstepToPack)
   {
      return false; // was not adjusted
   }
}
