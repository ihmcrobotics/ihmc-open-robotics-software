package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Generates a feasible momentum command for jumping depending on the state 
 * @author Apoorv Shrivastava
 */

public class CentroidalMomentumManager implements JumpControlManagerInterface
{
   private final YoVariableRegistry registry;

   private final double gravityZ;
   private final ReferenceFrame controlFrame;
   private final ReferenceFrame comFrame;
   private final MomentumRateCommand momentumCommand;

   private Vector3D linearMomentumWeight = new Vector3D();
   private Vector3D angularMomentumWeight = new Vector3D();
   private final FrameVector3D desiredLinearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D desiredAngularMomentumRateOfChange = new FrameVector3D();

   private final YoFrameVector yoDesiredLinearMomentumRateOfChange;
   private final YoFrameVector yoDesiredAngularMomentumRateOfChange;

   private EnumProvider<JumpStateEnum> currentState;
   private final double totalMass;
   public CentroidalMomentumManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters parameters, YoVariableRegistry registry)
   {
      this.registry = registry;
      controlFrame = ReferenceFrame.getWorldFrame();
      gravityZ = controllerToolbox.getGravityZ();
      comFrame = controllerToolbox.getCenterOfMassFrame();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      totalMass = fullRobotModel.getTotalMass();
      momentumCommand = new MomentumRateCommand();
      yoDesiredAngularMomentumRateOfChange = new YoFrameVector(getClass().getSimpleName() + "DesiredAngularMomentumRateOfChange", controlFrame, registry);
      yoDesiredLinearMomentumRateOfChange = new YoFrameVector(getClass().getSimpleName() + "DesiredLinearMomentumRateOfChange", controlFrame, registry);
      setMomentumCommandWeights();
   }

   @Override
   public void setStateEnumProvider(EnumProvider<JumpStateEnum> stateEnumProvider)
   {
      this.currentState = stateEnumProvider;
   }

   public void setOptimizationWeights(Vector3DReadOnly angularMomentumWeight, Vector3DReadOnly linearMomentumWeight)
   {
      this.angularMomentumWeight.set(angularMomentumWeight);
      this.linearMomentumWeight.set(linearMomentumWeight);
   }

   public void setOptimizationWeights(double angularWeight, double linearWeight)
   {
      this.angularMomentumWeight.set(angularWeight, angularWeight, angularWeight);
      this.linearMomentumWeight.set(linearWeight, linearWeight, linearWeight);
   }

   public void initialize()
   {
      momentumCommand.setSelectionMatrixToIdentity();
      setMomentumCommandWeights();
   }

   private void setMomentumCommandWeights()
   {
      momentumCommand.setAngularWeights(angularMomentumWeight);
      momentumCommand.setLinearWeights(linearMomentumWeight);
   }

   @Override
   public void compute()
   {
      switch (currentState.getValue())
      {
      case STANDING:
         computeMomentumCommandForStandingState(true);
         break;
      case TAKE_OFF:
         throw new RuntimeException("Unimplemented");
      case FLIGHT:
         desiredLinearMomentumRateOfChange.setIncludingFrame(controlFrame, 0.0, 0.0, -gravityZ * totalMass);
         desiredAngularMomentumRateOfChange.setIncludingFrame(controlFrame, 0.0, 0.0, 0.0);
         momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
         yoDesiredAngularMomentumRateOfChange.set(desiredAngularMomentumRateOfChange);
         yoDesiredLinearMomentumRateOfChange.set(desiredLinearMomentumRateOfChange);
         setOptimizationWeights(50.0, 2.0);
         setMomentumCommandWeights();
         momentumCommand.setSelectionMatrixForLinearControl();
         break;
      case LANDING:
         throw new RuntimeException("Unimplemented");
      default:
         throw new RuntimeException("Invalid jump state for centroidal momentum computation");
      }
   }

   private void computeMomentumCommandForStandingState(boolean computePelvisFeedbackControl)
   {
      desiredLinearMomentumRateOfChange.setToZero(controlFrame);
      desiredAngularMomentumRateOfChange.setToZero(controlFrame);
      momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
      yoDesiredAngularMomentumRateOfChange.set(desiredAngularMomentumRateOfChange);
      yoDesiredLinearMomentumRateOfChange.set(desiredLinearMomentumRateOfChange);
      setOptimizationWeights(50.0, 2.0);
      setMomentumCommandWeights();
      momentumCommand.setSelectionMatrixToIdentity();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return momentumCommand;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }
}
