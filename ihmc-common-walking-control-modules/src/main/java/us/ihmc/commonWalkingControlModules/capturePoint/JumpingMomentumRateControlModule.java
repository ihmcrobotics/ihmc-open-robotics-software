package us.ihmc.commonWalkingControlModules.capturePoint;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRMomentumController;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.Black;
import static us.ihmc.graphicsDescription.appearance.YoAppearance.DarkRed;

public class JumpingMomentumRateControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Vector3DReadOnly linearMomentumRateWeight;
   private final Vector3DReadOnly angularMomentumRateWeight;

   private final YoFrameVector3D controlledCoMAcceleration;

   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private double omega0 = 3.0;
   private double totalMass;
   private double timeInContactPhase;

   private List<Trajectory3D> vrpTrajectories;

   private final ReferenceFrame centerOfMassFrame;
   private final FramePoint2D centerOfMass2d = new FramePoint2D();

   private final CenterOfMassJacobian centerOfMassJacobian;

   private final FixedFramePoint2DBasics achievedCMP = new FramePoint2D();

   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();
   private final FrameVector2D achievedCoMAcceleration2d = new FrameVector2D();

   private final FrameVector3D linearMomentumRateOfChange = new FrameVector3D();

   private final LQRMomentumController lqrMomentumController;

   private final YoFramePoint2D yoAchievedCMP = new YoFramePoint2D("achievedCMP", worldFrame, registry);
   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);

   public JumpingMomentumRateControlModule(CommonHumanoidReferenceFrames referenceFrames,
                                           RigidBodyBasics elevator,
                                           WalkingControllerParameters walkingControllerParameters,
                                           YoRegistry parentRegistry,
                                           YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      linearMomentumRateWeight = new ParameterVector3D("LinearMomentumRateWeight", momentumOptimizationSettings.getLinearMomentumWeight(), registry);
      angularMomentumRateWeight = new ParameterVector3D("AngularMomentumRateWeight", momentumOptimizationSettings.getAngularMomentumWeight(), registry);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      controlledCoMAcceleration = new YoFrameVector3D("ControlledCoMAcceleration", "", centerOfMassFrame, registry);

      centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved CMP", yoAchievedCMP, 0.005, DarkRed(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.006, Black(), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", achievedCMPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", centerOfMassViz.createArtifact());
      }
      yoAchievedCMP.setToNaN();
      yoCenterOfMass.setToNaN();

      lqrMomentumController = new LQRMomentumController(omega0, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      yoAchievedCMP.setToNaN();
      yoCenterOfMass.setToNaN();
   }

   public void setInputFromWalkingStateMachine(JumpingMomentumRateControlModuleInput input)
   {
      this.omega0 = input.getOmega0();
      this.timeInContactPhase = input.getTimeInState();
      this.vrpTrajectories = input.getVrpTrajectories();
   }

   public void setInputFromControllerCore(ControllerCoreOutput controllerCoreOutput)
   {
      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumRateCommand;
   }

   private final DMatrixRMaj currentState = new DMatrixRMaj(6, 1);

   public void computeControllerCoreCommands()
   {
      centerOfMassJacobian.reset();
      yoCenterOfMass.set(centerOfMassJacobian.getCenterOfMass());
      yoCenterOfMassVelocity.set(centerOfMassJacobian.getCenterOfMassVelocity());

      yoCenterOfMass.get(currentState);
      yoCenterOfMassVelocity.get(3, currentState);

      computeICPController();
      computeDesiredLinearMomentumRateOfChange();

      selectionMatrix.setToLinearSelectionOnly();
      momentumRateCommand.setLinearMomentumRate(linearMomentumRateOfChange);
      momentumRateCommand.setSelectionMatrix(selectionMatrix);
      momentumRateCommand.setWeights(angularMomentumRateWeight, linearMomentumRateWeight);
   }

   public void computeAchievedCMP()
   {
      if (achievedLinearMomentumRate.containsNaN())
      {
         yoAchievedCMP.setToNaN();
         return;
      }

      centerOfMass2d.setToZero(centerOfMassFrame);
      centerOfMass2d.changeFrame(worldFrame);

      achievedCoMAcceleration2d.setIncludingFrame(achievedLinearMomentumRate);
      achievedCoMAcceleration2d.scale(1.0 / totalMass);
      achievedCoMAcceleration2d.changeFrame(worldFrame);

      achievedCMP.set(achievedCoMAcceleration2d);
      achievedCMP.scale(-1.0 / (omega0 * omega0));
      achievedCMP.add(centerOfMass2d);

      yoAchievedCMP.set(achievedCMP);
   }

   private void computeICPController()
   {
      lqrMomentumController.setOmega(omega0);
      lqrMomentumController.setVRPTrajectory(vrpTrajectories);
      lqrMomentumController.computeControlInput(currentState, timeInContactPhase);
   }

   private void computeDesiredLinearMomentumRateOfChange()
   {
      controlledCoMAcceleration.set(lqrMomentumController.getU());
      linearMomentumRateOfChange.setIncludingFrame(controlledCoMAcceleration);
      linearMomentumRateOfChange.changeFrame(worldFrame);
      linearMomentumRateOfChange.scale(totalMass);
   }
}
