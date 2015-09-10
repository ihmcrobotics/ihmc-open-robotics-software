package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.AnkleOverRotationControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.HipDamperControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisHeightControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.PelvisOrientationControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualSupportActuatorControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SingleSupportCondition;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class BalanceSupportControlModule
{
   private final DesiredCoPControlModule velocityViaCoPControlModule;
   private final VirtualToePointCalculator virtualToePointCalculator;
   private final LegStrengthCalculator legStrengthCalculator;
   private final PelvisHeightControlModule pelvisHeightControlModule;
   private final PelvisOrientationControlModule pelvisOrientationControlModule;
   private final VirtualSupportActuatorControlModule virtualSupportActuatorControlModule;
   private final KneeDamperControlModule kneeDamperControlModule;
   private final HipDamperControlModule hipDamperControlModule;
   private final AnkleOverRotationControlModule ankleOverRotationControlModule;
   private final OldBipedSupportPolygons bipedSupportPolygons;

   private final CouplingRegistry couplingRegistry;

   private final SideDependentList<Double> legStrengths = new SideDependentList<Double>();
   private final SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
   private final SideDependentList<YoFramePoint> virtualToePointsInWorld = new SideDependentList<YoFramePoint>();
   private final YoVariableRegistry registry = new YoVariableRegistry("BalanceSupportControlModule");


   public BalanceSupportControlModule(DesiredCoPControlModule velocityViaCoPControlModule, VirtualToePointCalculator virtualToePointAndLegStrengthCalculator,
                                      LegStrengthCalculator legStrengthCalculator, PelvisHeightControlModule pelvisHeightControlModule,
                                      PelvisOrientationControlModule pelvisOrientationControlModule,
                                      VirtualSupportActuatorControlModule virtualSupportActuatorControlModule, KneeDamperControlModule kneeDamperControlModule,
                                      HipDamperControlModule hipDamperControlModule, OldBipedSupportPolygons bipedSupportPolygons,
                                      AnkleOverRotationControlModule ankleOverRotationControlModule, CouplingRegistry couplingRegistry,
                                      YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.velocityViaCoPControlModule = velocityViaCoPControlModule;
      this.virtualToePointCalculator = virtualToePointAndLegStrengthCalculator;
      this.legStrengthCalculator = legStrengthCalculator;
      this.pelvisHeightControlModule = pelvisHeightControlModule;
      this.pelvisOrientationControlModule = pelvisOrientationControlModule;
      this.virtualSupportActuatorControlModule = virtualSupportActuatorControlModule;
      this.kneeDamperControlModule = kneeDamperControlModule;
      this.hipDamperControlModule = hipDamperControlModule;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.ankleOverRotationControlModule = ankleOverRotationControlModule;
      this.couplingRegistry = couplingRegistry;

      for (RobotSide robotSide : RobotSide.values)
      {
         String copName = robotSide.getCamelCaseNameForStartOfExpression() + "VTP";
         String listName = "VTPs";
         YoFramePoint vtp = new YoFramePoint(copName, ReferenceFrame.getWorldFrame(), registry);

         if (yoGraphicsListRegistry != null)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(copName, vtp, 0.005, YoAppearance.Navy(), GraphicType.BALL);
            yoGraphicsListRegistry.registerYoGraphic(listName, copViz);
            yoGraphicsListRegistry.registerArtifact(listName, copViz.createArtifact());
            virtualToePointsInWorld.put(robotSide, vtp);
         }
      }

      parentRegistry.addChild(registry);
   }

   /**
    * Computes the leg torques needed to maintain balance in single support.
    * @param supportLegTorquesToPack a LegTorques object that will be packed with the computed torques
    * @param desiredVelocity the desired CoM velocity
    * @param desiredPelvisOrientation the desired orientation of the pelvis link
    * @param upperBodyWrench TODO
    */
   public void doSingleSupportBalance(LegTorques supportLegTorquesToPack, FrameVector2d desiredVelocity, FrameOrientation desiredPelvisOrientation,
                                      Wrench upperBodyWrench, SingleSupportCondition singleSupportCondition, double timeInState)
   {
      RobotSide supportLeg = supportLegTorquesToPack.getRobotSide();

      // compute desired CoP (=VTP) using velocityViaCoPControlModule. Should be inside the foot polygon already at this point.
      FramePoint2d vtpInAnklePitchFrame = velocityViaCoPControlModule.computeDesiredCoPSingleSupport(supportLeg, desiredVelocity, singleSupportCondition,
                                             timeInState);
      couplingRegistry.setDesiredCoP(vtpInAnklePitchFrame);


      // compute desired torques on the pelvis using PelvisOrientationControlModule.
      FrameVector torqueOnPelvisInPelvisFrame = pelvisOrientationControlModule.computePelvisTorque(supportLeg, desiredPelvisOrientation);

      // compute desired z-component of force on the body using PelvisHeightController
      double desiredPelvisHeightInWorld = getDesiredPelvisHeight();
      double fZOnPelvisInPelvisFrame = pelvisHeightControlModule.doPelvisHeightControl(desiredPelvisHeightInWorld, supportLeg);

      // compute joint torques using virtual support actuators
      virtualSupportActuatorControlModule.controlSingleSupport(supportLegTorquesToPack, vtpInAnklePitchFrame, fZOnPelvisInPelvisFrame,
              torqueOnPelvisInPelvisFrame, upperBodyWrench);

      // Add a little knee damping to prevent it from snapping:
      kneeDamperControlModule.addKneeDamping(supportLegTorquesToPack);

      ankleOverRotationControlModule.adjustLegTorquesToPreventOverRotation(supportLegTorquesToPack);
   }

   /**
    * Computes the lower body torques needed to maintain balance in double support.
    * @param lowerBodyTorquesToPack a LowerBodyTorques object that will be packed with the computed torques
    * @param desiredVelocity the desired CoM velocity of the robot
    * @param desiredPelvisOrientation the desired orientation of the pelvis link
    */
   public void doDoubleSupportBalance(LowerBodyTorques lowerBodyTorquesToPack, RobotSide loadingLeg, FrameVector2d desiredVelocity,
                                      FrameOrientation desiredPelvisOrientation)
   {
      ankleOverRotationControlModule.resetForNextStep();

      // compute desired CoP
      FramePoint2d desiredCoP = velocityViaCoPControlModule.computeDesiredCoPDoubleSupport(loadingLeg, desiredVelocity);
      couplingRegistry.setDesiredCoP(desiredCoP);

      // compute VTPs and leg strengths
      virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP, couplingRegistry.getUpcomingSupportLeg());

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint virtualToePoint = virtualToePoints.get(robotSide).toFramePoint();
         virtualToePoint.changeFrame(ReferenceFrame.getWorldFrame());
         virtualToePointsInWorld.get(robotSide).set(virtualToePoint);
      }

      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, desiredCoP);

      // compute desired torques on the pelvis using PelvisOrientationControlModule.
      FrameVector torqueOnPelvisInPelvisFrame = pelvisOrientationControlModule.computePelvisTorque(null, desiredPelvisOrientation);

      // compute desired z-component of force on the body using PelvisHeightController
      double desiredPelvisHeightInWorld = getDesiredPelvisHeight();
      double fZOnPelvisInPelvisFrame = pelvisHeightControlModule.doPelvisHeightControl(desiredPelvisHeightInWorld, null);

      Wrench wrenchDueToUpperBodyLunging = couplingRegistry.getActualUpperBodyLungingWrench();

      if (wrenchDueToUpperBodyLunging != null)
      {
         FrameVector wrenchDueToUpperBodyDynamicsLinearPart = new FrameVector(wrenchDueToUpperBodyLunging.getExpressedInFrame(),
                                                                 wrenchDueToUpperBodyLunging.getLinearPartCopy());
         wrenchDueToUpperBodyDynamicsLinearPart.changeFrame(ReferenceFrame.getWorldFrame());
         double fZupperBodyOffset = wrenchDueToUpperBodyDynamicsLinearPart.getZ();

//       System.out.println("fZupperBodyOffset: " + fZupperBodyOffset);
         fZOnPelvisInPelvisFrame += fZupperBodyOffset;
      }

      // TODO think of a smarter way to not let the robot toe off while lunging sideways.
      if (couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()) != null)
      {
         fZOnPelvisInPelvisFrame *= 0.5;
      }

      double deltaNx = 0.0;    // TODO: Rethink the deltaNx stuff and see what it should be...

      // compute joint torques using virtual support actuators
      virtualSupportActuatorControlModule.controlDoubleSupport(lowerBodyTorquesToPack, virtualToePoints, legStrengths, fZOnPelvisInPelvisFrame,
              torqueOnPelvisInPelvisFrame, deltaNx);

      // Add a little knee damping to prevent it from snapping:
      kneeDamperControlModule.addKneeDamping(lowerBodyTorquesToPack.getLegTorques(RobotSide.LEFT));
      kneeDamperControlModule.addKneeDamping(lowerBodyTorquesToPack.getLegTorques(RobotSide.RIGHT));

      // Add some hip yaw damping on the trailing leg to prevent the leg from rotating
      if (loadingLeg != null)
      {
         hipDamperControlModule.addHipDamping(lowerBodyTorquesToPack.getLegTorques(loadingLeg.getOppositeSide()));
      }
   }

   private double getDesiredPelvisHeight()
   {
      return Double.NaN;
   }
}
