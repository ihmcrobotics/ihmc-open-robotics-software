package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Generates a feasible momentum command for jumping depending on the state 
 * Should typically determine the distribution of the CoM acceleration relative to the root body 
 * and the world frame 
 * @author Apoorv Shrivastava
 */

public class CentroidalMomentumManager
{
   private final YoVariableRegistry registry;

   private final double gravityZ;
   private final ReferenceFrame controlFrame;
   private final ReferenceFrame comFrame;

   private Vector3D linearMomentumWeight = new Vector3D();
   private Vector3D angularMomentumWeight = new Vector3D();

   private final FrameVector3D desiredLinearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D desiredAngularMomentumRateOfChange = new FrameVector3D();

   private final MomentumRateCommand momentumCommand = new MomentumRateCommand();

   private final FrameVector3D desiredCoMLinearAcceleration = new FrameVector3D();
   private final FrameVector3D desiredCoMAngularAcceleration = new FrameVector3D();
   private JumpStateEnum currentState;
   private double totalMass = 0;

   public CentroidalMomentumManager(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.registry = registry;
      controlFrame = ReferenceFrame.getWorldFrame();
      gravityZ = controllerToolbox.getGravityZ();
      comFrame = controllerToolbox.getCenterOfMassFrame();
      setMomentumCommandWeights();
   }

   public void updateState(JumpStateEnum currentState)
   {
      this.currentState = currentState;
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
      ;
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

   public void compute()
   {
      //FIXME This is a hack to confirm that the controller core is working. This should be based on the controller state. Currently hacked to work with flight controller
      desiredLinearMomentumRateOfChange.set(controlFrame, 0.0, 0.0, -gravityZ * totalMass);
      desiredAngularMomentumRateOfChange.set(controlFrame, 0.0, 0.0, 0.0);
      momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
      momentumCommand.setAngularWeightsToZero();
      momentumCommand.setSelectionMatrixForLinearControl();

      // TODO remove this command. Its not needed since the momentum rate of change command captures the free fall effect (does not mean the robot does nothing since it'll try to regularize the overall acceleration)      
      desiredCoMLinearAcceleration.setIncludingFrame(comFrame, 0, 0, 0.0);
   }

   public MomentumRateCommand getMomentumRateCommand()
   {
      return momentumCommand;
   }

   public void setTotalRobotMass(double totalRobotMass)
   {
      this.totalMass = totalRobotMass;
   }
}
