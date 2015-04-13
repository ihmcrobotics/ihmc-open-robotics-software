package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.kinematics.NumericalInverseKinematicsCalculator;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.*;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePose;

/**
 * Created by agrabertilton on 4/13/15.
 */
public class LegJointLimitAvoidanceControlModule
{
   private static final int maxIterationsForIK = 5;
   private static final double lambdaLeastSquares = 0.0009;
   private static final double tolerance = 1e-12;
   private static final double maxStepSize = 0.01;
   private static final double minRandomSearchScalar = 1.0;
   private static final double maxRandomSearchScalar = 1.0;

   private final DoubleYoVariable percentJointRangeForThreshold;
   private FullRobotModel robotModel;
   private RigidBody base;
   private OneDoFJoint[] robotJoints;
   private OneDoFJoint[] ikJoints;
   private NumericalInverseKinematicsCalculator inverseKinematicsCalculator;
   GeometricJacobian jacobian;

   private DoubleYoVariable[] originalDesiredPositions;
   private DoubleYoVariable[] alphas;
   private DoubleYoVariable[] adjustedDesiredPositions;
   private YoFramePose originalDesiredPose;
   private YoFramePose adjustedDesiredPose;
   private RigidBodyTransform desiredTransform;

   public LegJointLimitAvoidanceControlModule(YoVariableRegistry registry, MomentumBasedController momentumBasedController, RobotSide robotSide){
      percentJointRangeForThreshold = new DoubleYoVariable("percentJointRangeForThreshold", registry);
      robotModel = momentumBasedController.getFullRobotModel();
      base = robotModel.getPelvis();
      RigidBody foot = robotModel.getFoot(robotSide);
      robotJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(base, foot), OneDoFJoint.class);
      ikJoints = ScrewTools.filterJoints(ScrewTools.cloneJointPath(robotJoints), OneDoFJoint.class);
      ReferenceFrame cloneOfFootFrame = foot.getBodyFixedFrame();
      jacobian = new GeometricJacobian(ikJoints, cloneOfFootFrame);
      inverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares, tolerance, maxIterationsForIK, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);

      int size = ikJoints.length;
      {
         originalDesiredPositions = new DoubleYoVariable[size];
         alphas = new DoubleYoVariable[size];
         adjustedDesiredPositions = new DoubleYoVariable[size];
         for (int i = 0; i < size; i++){
            originalDesiredPositions[i] = new DoubleYoVariable("originalDesiredPositions" + i, registry);
            alphas[i] = new DoubleYoVariable("alpha" + i, registry);
            adjustedDesiredPositions[i] = new DoubleYoVariable("adjustedDesiredPositions" + i, registry);
         }

         originalDesiredPose = new YoFramePose("originalDesiredPose", ReferenceFrame.getWorldFrame(), registry);
         adjustedDesiredPose = new YoFramePose("adjustedDesiredPose", ReferenceFrame.getWorldFrame(), registry);
      }

      percentJointRangeForThreshold.set(0.05);
   }

   public void correctSwingFootTrajectory(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocityOfOrigin,
                                          FrameVector desiredAngularVelocity, FrameVector desiredLinearAccelerationOfOrigin, FrameVector desiredAngularAcceleration, RigidBody base)
   {
      originalDesiredPose.setPosition(desiredPosition);
      originalDesiredPose.setOrientation(desiredOrientation);
      desiredTransform = originalDesiredPose.getReferenceFrame().getTransformToDesiredFrame(base.getBodyFixedFrame());
      inverseKinematicsCalculator.solve(desiredTransform); //sets the qs of the one-dof ikJoints

      // adjust joints based on joint angle limits
      adjustJointPositions();

      //TODO calcualte the new desired position and orientation

      //TODO calculate the adjusted joint velocities using the alphas, then calculate the adjusted velocities

   }

   private void adjustJointPositions()
   {
      int size = ikJoints.length;
      double upperLimit;
      double lowerLimit;
      double midpointOfLimits;
      double range;
      double rangePercentageForThreshold = percentJointRangeForThreshold.getDoubleValue();
      double lambda = 1- rangePercentageForThreshold;

      for (int i = 0; i < size; i++){
         lowerLimit = ikJoints[i].getJointLimitLower();
         upperLimit = ikJoints[i].getJointLimitUpper();
         midpointOfLimits = (lowerLimit + upperLimit) / 2;
         range = upperLimit - lowerLimit;

         originalDesiredPositions[i].set(ikJoints[i].getqDesired());
         double comparisonValue = Math.abs(2* (originalDesiredPositions[i].getDoubleValue() - midpointOfLimits) / range); //should range between -1 and 1, which are the limits

         double alpha = 0;
         if (comparisonValue > lambda){
            alpha = (comparisonValue - lambda)/rangePercentageForThreshold;
         }
         alphas[i].set(alpha);

         double adjustedPosition = alpha * ikJoints[i].getQ() + (1-alpha) * ikJoints[i].getqDesired();
         adjustedDesiredPositions[i].set(adjustedPosition);
         ikJoints[i].setqDesired(adjustedPosition);
      }
   }
}