package us.ihmc.commonWalkingControlModules.controlModules.foot;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.AnkleIKSolver;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class UnloadedAnkleControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final JointDesiredOutputList jointDesiredOutputList;
   private final JointspaceAccelerationCommand accelerationCommand = new JointspaceAccelerationCommand();

   private final RigidBodyBasics foot;
   private final MovingReferenceFrame shinFrame;

   private final BooleanParameter useAnkleIKModule;
   private final YoBoolean enabled = new YoBoolean("UnloadedAnkleControlModuleEnabled", registry);

   private final AnkleIKSolver ankleIKSolver;
   private final DenseMatrix64F jointAngles = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F jointVelocities = new DenseMatrix64F(1, 1);

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAcceleration = new FrameVector3D();
   private final FrameVector3D shinAngularVelocity = new FrameVector3D();
   private final FramePoint3D controlFramePosition = new FramePoint3D();
   private final FrameQuaternion controlFrameOrientation = new FrameQuaternion();
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final FrameVector3D footAngularVelocityInShin = new FrameVector3D();

   public UnloadedAnkleControlModule(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide, AnkleIKSolver ankleIKSolver, YoVariableRegistry parentRegistry)
   {
      this.ankleIKSolver = ankleIKSolver;

      useAnkleIKModule = new BooleanParameter("UseAnkleIKModule" + robotSide.getPascalCaseName(), registry, false);
      foot = fullRobotModel.getFoot(robotSide);
      RigidBodyBasics shin = ScrewTools.goUpBodyChain(foot, ankleIKSolver.getNumberOfJoints());
      shinFrame = shin.getParentJoint().getFrameAfterJoint();
      OneDoFJointBasics[] ankleJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.createJointPath(shin, foot), OneDoFJointBasics.class);
      jointDesiredOutputList = new JointDesiredOutputList(ankleJoints);
      parentRegistry.addChild(registry);
   }

   public void compute(ConstraintType currentConstraintType, AbstractFootControlState currentState)
   {
      boolean footIsInAir = !currentConstraintType.isLoadBearing();
      enabled.set(useAnkleIKModule.getValue() && footIsInAir);

      if (!enabled.getBooleanValue())
      {
         jointDesiredOutputList.clear();
         accelerationCommand.clear();
         return;
      }

      SpatialFeedbackControlCommand feedbackControlCommand = currentState.getFeedbackControlCommand();
      if (feedbackControlCommand.getEndEffector().hashCode() != foot.hashCode())
      {
         throw new RuntimeException("Something got messed up. Expecting a command for " + foot.getName());
      }

      feedbackControlCommand.getControlFramePoseIncludingFrame(controlFramePosition, controlFrameOrientation);
      controlFrameOrientation.changeFrame(foot.getParentJoint().getFrameAfterJoint());
      rotationMatrix.set(controlFrameOrientation);
      if (!rotationMatrix.isIdentity())
      {
         // Fix this by properly transforming the desired values if needed.
         throw new RuntimeException("The foot control frame can not be rotated with respect to the ankle when using this module.");
      }

      feedbackControlCommand.setInverseDynamics(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
      desiredOrientation.changeFrame(shinFrame);
      desiredAngularVelocity.changeFrame(shinFrame);

      shinAngularVelocity.setIncludingFrame(shinFrame.getTwistOfFrame().getAngularPart());
      shinAngularVelocity.checkReferenceFrameMatch(desiredAngularVelocity);
      footAngularVelocityInShin.setToZero(shinFrame);
      footAngularVelocityInShin.sub(desiredAngularVelocity, shinAngularVelocity);

      ankleIKSolver.computeAngles(desiredOrientation, jointAngles);
      ankleIKSolver.computeVelocities(footAngularVelocityInShin, jointAngles, jointVelocities);

      for (int jointIdx = 0; jointIdx < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); jointIdx++)
      {
         jointDesiredOutputList.getJointDesiredOutput(jointIdx).setDesiredPosition(jointAngles.get(jointIdx));
         jointDesiredOutputList.getJointDesiredOutput(jointIdx).setDesiredVelocity(jointVelocities.get(jointIdx));
      }
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return accelerationCommand;
   }

   public JointDesiredOutputList getJointDesiredOutputList()
   {
      return jointDesiredOutputList;
   }

}
