package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
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
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

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
   private final BipedSupportPolygons bipedSupportPolygons;

   private final CouplingRegistry couplingRegistry;
   
   private final SideDependentList<Double> legStrengths = new SideDependentList<Double>();
   private final SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
   private final YoVariableRegistry registry = new YoVariableRegistry("BalanceSupportControlModule");

   public BalanceSupportControlModule(DesiredCoPControlModule velocityViaCoPControlModule,
                                      VirtualToePointCalculator virtualToePointAndLegStrengthCalculator, LegStrengthCalculator legStrengthCalculator,
                                      PelvisHeightControlModule pelvisHeightControlModule, PelvisOrientationControlModule pelvisOrientationControlModule,
                                      VirtualSupportActuatorControlModule virtualSupportActuatorControlModule, KneeDamperControlModule kneeDamperControlModule,
                                      HipDamperControlModule hipDamperControlModule, BipedSupportPolygons bipedSupportPolygons,
                                      AnkleOverRotationControlModule ankleOverRotationControlModule, CouplingRegistry couplingRegistry, YoVariableRegistry parentRegistry)
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

      parentRegistry.addChild(registry);
   }

   /**
    * Computes the leg torques needed to maintain balance in single support.
    * @param supportLegTorquesToPack a LegTorques object that will be packed with the computed torques
    * @param desiredVelocity the desired CoM velocity
    * @param desiredPelvisOrientation the desired orientation of the pelvis link
    * @param upperBodyWrench TODO
    */
   public void doSingleSupportBalance(LegTorques supportLegTorquesToPack, FrameVector2d desiredVelocity, Orientation desiredPelvisOrientation,
                                      Wrench upperBodyWrench, SingleSupportCondition singleSupportCondition, double timeInState)
   {
      virtualToePointCalculator.hideVisualizationGraphics();

      RobotSide supportLeg = supportLegTorquesToPack.getRobotSide();

      // compute desired CoP (=VTP) using velocityViaCoPControlModule. Should be inside the foot polygon already at this point.
      FramePoint2d vtpInAnklePitchFrame = velocityViaCoPControlModule.computeDesiredCoPSingleSupport(supportLeg, desiredVelocity, singleSupportCondition, timeInState);
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
                                      Orientation desiredPelvisOrientation)
   {
      ankleOverRotationControlModule.resetForNextStep();

      // compute desired CoP
      FramePoint2d desiredCoP = velocityViaCoPControlModule.computeDesiredCoPDoubleSupport(loadingLeg, desiredVelocity);
      couplingRegistry.setDesiredCoP(desiredCoP);

      // compute VTPs and leg strengths
      virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP);
      legStrengthCalculator.packLegStrengths(legStrengths, virtualToePoints, desiredCoP);

      // compute desired torques on the pelvis using PelvisOrientationControlModule.
      FrameVector torqueOnPelvisInPelvisFrame = pelvisOrientationControlModule.computePelvisTorque(null, desiredPelvisOrientation);

      // compute desired z-component of force on the body using PelvisHeightController
      double desiredPelvisHeightInWorld = getDesiredPelvisHeight();
      double fZOnPelvisInPelvisFrame = pelvisHeightControlModule.doPelvisHeightControl(desiredPelvisHeightInWorld, null);

      double deltaNx = 0.0; //TODO: Rethink the deltaNx stuff and see what it should be...
      
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
