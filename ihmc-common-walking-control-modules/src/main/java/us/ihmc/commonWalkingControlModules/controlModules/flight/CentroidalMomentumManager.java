package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.ForceTrajectory;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.PositionTrajectory;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Generates a feasible momentum command for jumping depending on the state 
 * @author Apoorv Shrivastava
 */

public class CentroidalMomentumManager implements JumpControlManagerInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
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
   private final YoFrameVector yoDesiredGroundReactionForce;

   private final double totalMass;
   private final ForceTrajectory groundReactionForceProfile = new ForceTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments,
                                                                                  WholeBodyMotionPlanner.numberOfForceCoefficients);
   private final PositionTrajectory plannedCoMPositionTrajectory = new PositionTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments,
                                                                                          WholeBodyMotionPlanner.numberOfForceCoefficients + 2);
   private final FrameVector3D gravitationalForce = new FrameVector3D(worldFrame);
   private final FrameVector3D groundReactionForceToAchieve = new FrameVector3D();

   public CentroidalMomentumManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters parameters, YoVariableRegistry registry)
   {
      this.registry = registry;
      controlFrame = ReferenceFrame.getWorldFrame();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      totalMass = fullRobotModel.getTotalMass();
      gravityZ = controllerToolbox.getGravityZ();
      gravitationalForce.set(0.0, 0.0, totalMass * -gravityZ);
      comFrame = controllerToolbox.getCenterOfMassFrame();
      momentumCommand = new MomentumRateCommand();
      yoDesiredAngularMomentumRateOfChange = new YoFrameVector(getClass().getSimpleName() + "DesiredAngularMomentumRateOfChange", controlFrame, registry);
      yoDesiredLinearMomentumRateOfChange = new YoFrameVector(getClass().getSimpleName() + "DesiredLinearMomentumRateOfChange", controlFrame, registry);
      yoDesiredGroundReactionForce = new YoFrameVector(getClass().getSimpleName() + "DesiredGroundReactionForce", controlFrame, registry);
      setMomentumCommandWeights();
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

   public void computeMomentumRateOfChangeForFreeFall()
   {
      groundReactionForceToAchieve.setToZero(controlFrame);
      desiredLinearMomentumRateOfChange.setIncludingFrame(gravitationalForce);
      desiredLinearMomentumRateOfChange.changeFrame(controlFrame);
      desiredAngularMomentumRateOfChange.setIncludingFrame(controlFrame, 0.0, 0.0, 0.0);
      momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
      updateYoVariables();
      setOptimizationWeights(50.0, 2.0);
      setMomentumCommandWeights();
      momentumCommand.setSelectionMatrixForLinearControl();
   }

   private void updateYoVariables()
   {
      yoDesiredAngularMomentumRateOfChange.set(desiredAngularMomentumRateOfChange);
      yoDesiredLinearMomentumRateOfChange.set(desiredLinearMomentumRateOfChange);
      yoDesiredGroundReactionForce.set(groundReactionForceToAchieve);
   }

   public void computeForZeroMomentumRateOfChange()
   {
      groundReactionForceToAchieve.setIncludingFrame(gravitationalForce);
      groundReactionForceToAchieve.scale(-1.0);
      groundReactionForceToAchieve.changeFrame(controlFrame);
      desiredLinearMomentumRateOfChange.setToZero(controlFrame);
      desiredAngularMomentumRateOfChange.setToZero(controlFrame);
      momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
      updateYoVariables();
      setOptimizationWeights(50.0, 2.0);
      setMomentumCommandWeights();
      momentumCommand.setSelectionMatrixToIdentity();
   }

   public void computeMomentumRateOfChangeFromForceProfile(double time)
   {
      groundReactionForceProfile.update(time, groundReactionForceToAchieve);
      groundReactionForceToAchieve.changeFrame(controlFrame);
      desiredLinearMomentumRateOfChange.add(groundReactionForceToAchieve, gravitationalForce);
      desiredLinearMomentumRateOfChange.changeFrame(controlFrame);
      desiredLinearMomentumRateOfChange.setIncludingFrame(controlFrame, desiredLinearMomentumRateOfChange);
      desiredAngularMomentumRateOfChange.setToZero(controlFrame);
      momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
      updateYoVariables();
      setOptimizationWeights(50.0, 2.0);
      setMomentumCommandWeights();
      momentumCommand.setSelectionMatrixToIdentity();
   }

   public void setGroundReactionForceProfile(ForceTrajectory forceTrajectory)
   {
      this.groundReactionForceProfile.set(forceTrajectory);
   }

   public void setCoMTrajectory(PositionTrajectory positionTrajectory)
   {
      this.plannedCoMPositionTrajectory.set(positionTrajectory);
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
