package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.ForceTrajectory;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.PositionTrajectory;
import us.ihmc.commonWalkingControlModules.configurations.JumpControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.frames.YoFramePoint;
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
   private final MomentumRateCommand momentumCommand;
   
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private Vector3D linearMomentumWeight = new Vector3D();
   private Vector3D angularMomentumWeight = new Vector3D();
   private final FrameVector3D desiredLinearMomentumRateOfChange = new FrameVector3D();
   private final FrameVector3D desiredAngularMomentumRateOfChange = new FrameVector3D();

   private final YoFrameVector yoDesiredLinearMomentumRateOfChange;
   private final YoFrameVector yoDesiredAngularMomentumRateOfChange;
   private final YoFrameVector yoDesiredGroundReactionForce;
   private final YoFrameVector yoPlannedGroundReactionForce;
   private final YoFramePoint yoPlannedCoMPosition;
   private final YoFrameVector yoPlannedCoMVelocity;
   private final YoFramePoint yoEstimatedCoMPosition;
   private final YoFrameVector yoEstimatedCoMVelocity;

   private final double totalMass;
   private final ForceTrajectory groundReactionForceProfile = new ForceTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments,
                                                                                  WholeBodyMotionPlanner.numberOfForceCoefficients);
   private final PositionTrajectory plannedCoMPositionTrajectory = new PositionTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments,
                                                                                          WholeBodyMotionPlanner.numberOfForceCoefficients + 2);
   private final FrameVector3D gravitationalForce = new FrameVector3D(worldFrame);
   private final FrameVector3D desiredGroundReactionForce = new FrameVector3D();
   private final FrameVector3D plannedGroundReactionForce = new FrameVector3D();
   private final FrameVector3D plannedCoMVelocity = new FrameVector3D();
   private final FramePoint3D plannedCoMPosition = new FramePoint3D();
   private final FramePoint3D finalCoMPositionForState = new FramePoint3D();
   private final FrameVector3D finalCoMVelocityForState = new FrameVector3D();
   private final FramePoint3D estimatedCoMPosition = new FramePoint3D();
   private final FrameVector3D estimatedCoMVelocity = new FrameVector3D();
   private final FramePoint3D comPositionError = new FramePoint3D();
   private final FrameVector3D comVelocityError = new FrameVector3D();
   private final double positionErrorGain;
   private final double velocityErrorGain;

   public CentroidalMomentumManager(HighLevelHumanoidControllerToolbox controllerToolbox, JumpControllerParameters parameters, YoVariableRegistry registry)
   {
      this.registry = registry;
      controlFrame = ReferenceFrame.getWorldFrame();
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      totalMass = fullRobotModel.getTotalMass();
      gravityZ = controllerToolbox.getGravityZ();
      gravitationalForce.set(0.0, 0.0, totalMass * -gravityZ);
      this.controllerToolbox = controllerToolbox;
      momentumCommand = new MomentumRateCommand();

      positionErrorGain = parameters.getPositionErrorGain();
      velocityErrorGain = parameters.getVelocityErrorGain();
      
      String namePrefix = getClass().getSimpleName();
      yoDesiredAngularMomentumRateOfChange = new YoFrameVector(namePrefix + "DesiredAngularMomentumRateOfChange", controlFrame, registry);
      yoDesiredLinearMomentumRateOfChange = new YoFrameVector(namePrefix + "DesiredLinearMomentumRateOfChange", controlFrame, registry);
      yoDesiredGroundReactionForce = new YoFrameVector(namePrefix + "DesiredGroundReactionForce", controlFrame, registry);
      yoPlannedGroundReactionForce = new YoFrameVector(namePrefix + "PlannedGroundReactionForce", controlFrame, registry);
      yoPlannedCoMPosition = new YoFramePoint(namePrefix + "PlannedCoMPosition", controlFrame, registry);
      yoPlannedCoMVelocity = new YoFrameVector(namePrefix + "PlannedCoMVelocity", controlFrame, registry);
      yoEstimatedCoMPosition = new YoFramePoint(namePrefix + "EstimatedCoMPosition", controlFrame, registry);
      yoEstimatedCoMVelocity = new YoFrameVector(namePrefix + "EstimatedCoMVelocity", controlFrame, registry);

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
   
   private void updateCoMState()
   {
      controllerToolbox.getCenterOfMassPosition(estimatedCoMPosition);
      controllerToolbox.getCenterOfMassVelocity(estimatedCoMVelocity);
      estimatedCoMPosition.changeFrame(controlFrame);
      estimatedCoMVelocity.changeFrame(controlFrame);
      yoEstimatedCoMPosition.set(estimatedCoMPosition);
      yoEstimatedCoMVelocity.set(estimatedCoMVelocity);
   }

   public void computeMomentumRateOfChangeForFreeFall()
   {
      updateCoMState();
      desiredGroundReactionForce.setToZero(controlFrame);
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
      yoDesiredGroundReactionForce.set(desiredGroundReactionForce);
      yoPlannedGroundReactionForce.set(plannedGroundReactionForce);
      yoPlannedCoMPosition.set(plannedCoMPosition);
      yoPlannedCoMVelocity.set(plannedCoMVelocity);
   }

   public void computeForZeroMomentumRateOfChange()
   {
      updateCoMState();
      desiredGroundReactionForce.setIncludingFrame(gravitationalForce);
      desiredGroundReactionForce.scale(-1.0);
      desiredGroundReactionForce.changeFrame(controlFrame);
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
      updateCoMState();
      computeDesiredGroundReactionForce(time);
      desiredLinearMomentumRateOfChange.add(desiredGroundReactionForce, gravitationalForce);
      desiredLinearMomentumRateOfChange.changeFrame(controlFrame);
      desiredLinearMomentumRateOfChange.setIncludingFrame(controlFrame, desiredLinearMomentumRateOfChange);
      desiredAngularMomentumRateOfChange.setToZero(controlFrame);
      momentumCommand.setMomentumRate(desiredAngularMomentumRateOfChange, desiredLinearMomentumRateOfChange);
      updateYoVariables();
      setOptimizationWeights(50.0, 2.0);
      setMomentumCommandWeights();
      momentumCommand.setSelectionMatrixToIdentity();
   }

   private void computeDesiredGroundReactionForce(double time)
   {
      groundReactionForceProfile.update(time, plannedGroundReactionForce);
      plannedCoMPositionTrajectory.update(time, plannedCoMPosition, plannedCoMVelocity);
      comPositionError.sub(plannedCoMPosition, estimatedCoMPosition);
      comVelocityError.sub(plannedCoMVelocity, estimatedCoMVelocity);
      desiredGroundReactionForce.setIncludingFrame(plannedGroundReactionForce);
      comPositionError.scale(positionErrorGain);
      comVelocityError.scale(velocityErrorGain);
      desiredGroundReactionForce.add(comPositionError);
      desiredGroundReactionForce.add(comVelocityError);
      desiredGroundReactionForce.changeFrame(controlFrame);
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
   
   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return null;
   }

   public double getEstimatedCoMPositionZ()
   {
      return estimatedCoMPosition.getZ();
   }

   public double getEstimatedCoMVelocityZ()
   {
      return estimatedCoMVelocity.getZ();
   }

}
