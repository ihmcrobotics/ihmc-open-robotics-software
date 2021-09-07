package us.ihmc.commonWalkingControlModules.capturePoint;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.capturePoint.lqrControl.LQRJumpMomentumController;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.LinearMomentumRateCostCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingControllerToolbox;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.orientationControl.VariationalLQRController;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DReadOnly;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;

import static us.ihmc.graphicsDescription.appearance.YoAppearance.*;

public class JumpingMomentumRateControlModule
{
   private static final boolean useLinearLQR = false;
   private static final boolean useAngularLQR = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final Vector3DReadOnly linearMomentumRateWeight;
   private final Vector3DReadOnly angularMomentumRateWeight;

   private final YoFrameVector3D controlledCoMAcceleration;

   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
   private final SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

   private boolean inFlight = false;
   private double omega0 = 3.0;
   private double totalMass;
   private double timeInContactPhase;

   private List<? extends Polynomial3DReadOnly> vrpTrajectories;
   private List<? extends ContactStateProvider> contactStateProviders;

   private FrameVector3DReadOnly mpcLinearMomentumRateOfChange;
   private FrameVector3DReadOnly mpcAngularMomentumRateOfChange;

   private final ReferenceFrame centerOfMassFrame;
   private final FramePoint2D centerOfMass2d = new FramePoint2D();

   private final FixedFramePoint2DBasics achievedCMP = new FramePoint2D();

   private final FrameVector3D achievedLinearMomentumRate = new FrameVector3D();
   private final FrameVector2D achievedCoMAcceleration2d = new FrameVector2D();

   private final FrameVector3D linearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D angularMomentumRateOfChange = new FrameVector3D();

   private final LQRJumpMomentumController lqrMomentumController;
   private final VariationalLQRController orientationController;

   private final YoFramePoint2D yoAchievedCMP = new YoFramePoint2D("achievedCMP", worldFrame, registry);
   private final YoFramePoint3D yoDesiredVRP = new YoFramePoint3D("desiredVRP", worldFrame, registry);
   private final YoFramePoint3D yoCenterOfMass = new YoFramePoint3D("centerOfMass", worldFrame, registry);
   private final YoFrameVector3D yoCenterOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);
   private final YoBoolean includeAngularMomentumRate = new YoBoolean("minimizeAngularMomentumRate", registry);

   private final JumpingControllerToolbox controllerToolbox;

   public JumpingMomentumRateControlModule(JumpingControllerToolbox controllerToolbox,
                                           WalkingControllerParameters walkingControllerParameters,
                                           YoRegistry parentRegistry)
   {
      this.totalMass = TotalMassCalculator.computeSubTreeMass(controllerToolbox.getFullRobotModel().getElevator());
      this.controllerToolbox = controllerToolbox;

      MomentumOptimizationSettings momentumOptimizationSettings = walkingControllerParameters.getMomentumOptimizationSettings();
      linearMomentumRateWeight = new ParameterVector3D("LinearMomentumRateWeight1", new Vector3D(10, 10, 10), registry);
      angularMomentumRateWeight = new ParameterVector3D("AngularMomentumRateWeight1", new Vector3D(5e-1, 5e-1, 1e-1), registry);

      includeAngularMomentumRate.set(true);

      centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      controlledCoMAcceleration = new YoFrameVector3D("ControlledCoMAcceleration", worldFrame, registry);

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition achievedCMPViz = new YoGraphicPosition("Achieved CMP", yoAchievedCMP, 0.005, DarkRed(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition desiredVRPViz = new YoGraphicPosition("Desired VRP", yoDesiredVRP, 0.012, Purple(), GraphicType.BALL_WITH_CROSS);

         YoGraphicPosition centerOfMassViz = new YoGraphicPosition("Center Of Mass", yoCenterOfMass, 0.01, Black(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition dcmViz = new YoGraphicPosition("DCM", controllerToolbox.getYoCapturePoint(), 0.01, Blue(), GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", desiredVRPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", achievedCMPViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", centerOfMassViz.createArtifact());
         yoGraphicsListRegistry.registerArtifact("LinearMomentum", dcmViz.createArtifact());
      }
      yoAchievedCMP.setToNaN();
      yoCenterOfMass.setToNaN();

      lqrMomentumController = new LQRJumpMomentumController(controllerToolbox.getOmega0Provider(), totalMass, registry);
      orientationController = new VariationalLQRController();
      orientationController.setInertia(controllerToolbox.getFullRobotModel().getChest().getInertia());

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      yoAchievedCMP.setToNaN();
      yoCenterOfMass.setToNaN();
   }

   public void setInputFromWalkingStateMachine(JumpingMomentumRateControlModuleInput input)
   {
      this.inFlight = input.getInFlight();
      this.omega0 = input.getOmega0();
      this.timeInContactPhase = input.getTimeInState();
      this.vrpTrajectories = input.getVrpTrajectories();
      this.contactStateProviders = input.getContactStateProviders();
      mpcLinearMomentumRateOfChange = input.getDesiredLinearMomentumRateOfChange();
      mpcAngularMomentumRateOfChange = input.getDesiredAngularMomentumRateOfChange();
//      this.minimizeAngularMomentumRate.set(input.getMinimizeAngularMomentumRate());
   }

   public void setInputFromControllerCore(ControllerCoreOutput controllerCoreOutput)
   {
      controllerCoreOutput.getLinearMomentumRate(achievedLinearMomentumRate);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(momentumRateCommand);
//      inverseDynamicsCommandList.addCommand(lqrMomentumController.getMomentumRateCostCommand());
      return inverseDynamicsCommandList;
   }

   private final DMatrixRMaj currentState = new DMatrixRMaj(6, 1);

   public void computeControllerCoreCommands()
   {
      yoCenterOfMass.set(controllerToolbox.getCenterOfMassJacobian().getCenterOfMass());
      yoCenterOfMassVelocity.set(controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity());

      yoCenterOfMass.get(currentState);
      yoCenterOfMassVelocity.get(3, currentState);

      computeDesiredLinearMomentumRateOfChange();
      computeDesiredAngularMomentumRateOfChange();

      selectionMatrix.resetSelection();
      if (!includeAngularMomentumRate.getBooleanValue())
         selectionMatrix.clearAngularSelection();

//      momentumRateCommand.setLinearMomentumRate(linearMomentumRateOfChange);
//      momentumRateCommand.setAngularMomentumRate(angularMomentumRateOfChange);
      momentumRateCommand.setMomentumRate(angularMomentumRateOfChange, linearMomentumRateOfChange);
      momentumRateCommand.setSelectionMatrix(selectionMatrix);
      momentumRateCommand.setWeights(angularMomentumRateWeight, linearMomentumRateWeight);

      LinearMomentumRateCostCommand momentumRateCostCommand = lqrMomentumController.getMomentumRateCostCommand();
      momentumRateCostCommand.setSelectionMatrixToIdentity();
      momentumRateCostCommand.setWeights(linearMomentumRateWeight);
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

   private void computeDesiredLinearMomentumRateOfChange()
   {
      if (useLinearLQR)
      {
         lqrMomentumController.setVRPTrajectory(vrpTrajectories, contactStateProviders);
         lqrMomentumController.computeControlInput(currentState, 0.0);//timeInContactPhase);

         if (inFlight)
            linearMomentumRateOfChange.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -controllerToolbox.getGravityZ());
         else
         {
            //         vrpTrajectories.get(0).compute(0.0);
            //         yoDesiredVRP.set(vrpTrajectories.get(0).getPosition());
            //         linearMomentumRateOfChange.setToZero(worldFrame);
            //         linearMomentumRateOfChange.sub(yoCenterOfMass, yoDesiredVRP);
            //         linearMomentumRateOfChange.scale(omega0 * omega0);
            linearMomentumRateOfChange.setIncludingFrame(ReferenceFrame.getWorldFrame(), lqrMomentumController.getU());
         }

         controlledCoMAcceleration.setMatchingFrame(linearMomentumRateOfChange);

         linearMomentumRateOfChange.changeFrame(worldFrame);
         linearMomentumRateOfChange.scale(totalMass);
         yoDesiredVRP.set(lqrMomentumController.getFeedbackVRPPosition());
      }
      else
      {
         linearMomentumRateOfChange.set(mpcLinearMomentumRateOfChange);

         controlledCoMAcceleration.setMatchingFrame(linearMomentumRateOfChange);
         controlledCoMAcceleration.scale(1.0 / totalMass);

         yoDesiredVRP.set(controlledCoMAcceleration);
         yoDesiredVRP.scale(-1.0 / (omega0 * omega0));
         yoDesiredVRP.add(yoCenterOfMass);
      }
   }

   private final FramePose3D centerOfMass = new FramePose3D();

   private void computeDesiredAngularMomentumRateOfChange()
   {
      if (useAngularLQR)
      {
         MovingReferenceFrame chestFrame = controllerToolbox.getFullRobotModel().getChest().getBodyFixedFrame();
         centerOfMass.setToZero(chestFrame);
         centerOfMass.changeFrame(worldFrame);
         //      orientationController.compute(centerOfMass.getOrientation(), chestFrame.getTwistOfFrame().getAngularPart());

         //      orientationController.getDesiredTorque(angularMomentumRateOfChange);

         angularMomentumRateOfChange.changeFrame(worldFrame);
      }
      else
      {
         angularMomentumRateOfChange.set(mpcAngularMomentumRateOfChange);
      }
   }

}
