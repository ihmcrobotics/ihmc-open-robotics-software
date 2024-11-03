package us.ihmc.avatar.inverseKinematics;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

public class DDogLegArmIKSolver
{
   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 5;

   private final FullHumanoidRobotModel desiredRobot;
   private final ReferenceFrame handControlDesiredFrame;
   private final GeometricJacobian actualArmJacobian;
   // IK solver iterative working copy
   private final GeometricJacobian workArmJacobian;
   // The resulting solution as output
   private final GeometricJacobian desiredArmJacobian;
   private final DdoglegInverseKinematicsCalculator inverseKinematicsCalculator;

   private final Point3D handCenterOfMassInControlFrame;
   private final FramePose3D desiredHandCoMPose = new FramePose3D();
   private final FramePose3D lastDesiredHandCoMPose = new FramePose3D();
   private boolean ikFoundASolution = false;

   public DDogLegArmIKSolver(RobotSide side,
                             DRCRobotModel robotModel,
                             FullHumanoidRobotModel actualRobot,
                             FullHumanoidRobotModel desiredRobot,
                             ReferenceFrame handControlDesiredFrame)
   {
      this.desiredRobot = desiredRobot;
      this.handControlDesiredFrame = handControlDesiredFrame;

      handCenterOfMassInControlFrame
            = FullRobotModelUtils.getHandCenterOfMassInControlFrame(actualRobot, side, robotModel.getJointMap().getHandControlFrameToWristTransform(side));

      FullHumanoidRobotModel workingRobot = robotModel.createFullRobotModel();
      actualArmJacobian = new GeometricJacobian(actualRobot.getChest(), actualRobot.getHand(side), actualRobot.getHand(side).getBodyFixedFrame());
      desiredArmJacobian = new GeometricJacobian(desiredRobot.getChest(), desiredRobot.getHand(side), desiredRobot.getHand(side).getBodyFixedFrame());
      workArmJacobian = new GeometricJacobian(workingRobot.getChest(), workingRobot.getHand(side), workingRobot.getHand(side).getBodyFixedFrame());

      double convergeTolerance = 4.0e-6;
      double parameterChangePenalty = 0.1;
      double positionCost = 1.0;
      double orientationCost = 0.2;
      int maxIterations = 500;
      boolean solveOrientation = true;
      double toleranceForPositionError = 0.005;
      double toleranceForOrientationError = 0.02;
      inverseKinematicsCalculator = new DdoglegInverseKinematicsCalculator(workArmJacobian,
                                                                           positionCost,
                                                                           orientationCost,
                                                                           maxIterations,
                                                                           solveOrientation,
                                                                           convergeTolerance,
                                                                           toleranceForPositionError,
                                                                           toleranceForOrientationError,
                                                                           parameterChangePenalty);
   }

   public void update()
   {
      desiredArmJacobian.compute();
      actualArmJacobian.compute();

      desiredHandCoMPose.setToZero(handControlDesiredFrame);
      // The IK's solution is to the center of the mass of the hand, let's undo this
      // for the control, so we can control in our desired frame.
      desiredHandCoMPose.getPosition().add(handCenterOfMassInControlFrame);
      desiredHandCoMPose.changeFrame(desiredRobot.getChest().getBodyFixedFrame());
   }

   public boolean getDesiredHandControlPoseChanged()
   {
      boolean desiredHandControlPoseChanged = !desiredHandCoMPose.geometricallyEquals(lastDesiredHandCoMPose, 0.0001);
      lastDesiredHandCoMPose.setIncludingFrame(desiredHandCoMPose);
      return desiredHandControlPoseChanged;
   }

   public void copyActualToWork()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(actualArmJacobian.getJointsInOrder(), workArmJacobian.getJointsInOrder());
   }

   public void solve()
   {
      workArmJacobian.compute();
      ikFoundASolution = false;

      for (int i = 0; i < INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE && !ikFoundASolution; i++)
      {
         ikFoundASolution = inverseKinematicsCalculator.solve(desiredHandCoMPose);
      }
   }

   public void copyWorkToDesired()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(workArmJacobian.getJointsInOrder(), desiredArmJacobian.getJointsInOrder());
   }

   public void setDesiredToCurrent()
   {
      MultiBodySystemMissingTools.copyOneDoFJointsConfiguration(actualArmJacobian.getJointsInOrder(), desiredArmJacobian.getJointsInOrder());
   }
}
