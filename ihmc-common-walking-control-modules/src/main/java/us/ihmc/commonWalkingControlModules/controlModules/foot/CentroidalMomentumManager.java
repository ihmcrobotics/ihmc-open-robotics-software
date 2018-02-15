package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.JumpStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Generates a feasible momentum command for jumping depending on the state 
 * @author Apoorv Shrivastava
 */

public class CentroidalMomentumManager
{
   private final YoVariableRegistry registry;
   
   private final double controlDT;
   private final double gravityZ;
   private final ReferenceFrame controlFrame;

   private Vector3D linearMomentumWeight = new Vector3D();
   private Vector3D angularMomentumWeight = new Vector3D();

   private final FrameVector3D estimatedWholeBodyLinearMomentum = new FrameVector3D();
   private final FrameVector3D estimatedWholeBodyAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredWholeBodyLinearMomentum = new FrameVector3D();
   private final FrameVector3D desireddWholeBodyAngularMomentum = new FrameVector3D();
   private final FrameVector3D desiredLinearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D desiredAngularMomentumRateOfChange = new FrameVector3D();

   private final MomentumRateCommand momentumCommand = new MomentumRateCommand();
   private JumpStateEnum currentState;
   private double totalMass = 0;

   public CentroidalMomentumManager(HighLevelHumanoidControllerToolbox controllerToolbox, YoVariableRegistry registry)
   {
      this.registry = registry;
      controlFrame = ReferenceFrame.getWorldFrame();
      controlDT = controllerToolbox.getControlDT();
      gravityZ = controllerToolbox.getGravityZ();
      setMomentumCommandWeights();
   }

   public void update(JumpStateEnum currentState)
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
      this.linearMomentumWeight.set(linearWeight, linearWeight, linearWeight);;
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
      desiredLinearMomentumRateOfChange.set(controlFrame, 0.0, 0.0, -0.0 * totalMass);
      desiredAngularMomentumRateOfChange.set(controlFrame, 0.0, 0.0, 0.0);
      momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
      momentumCommand.setSelectionMatrixToIdentity();
      //PrintTools.debug("Linear weights: " + linearMomentumWeight.toString() + ", AngularMomentumwWeights: " + angularMomentumWeight.toString());
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
