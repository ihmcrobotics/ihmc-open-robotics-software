package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.DampedLeastSquaresJacobianSolver;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.Wrench;

public class SwingFullLegJacobian
{
   private final RobotSide robotSide;
   private final GeometricJacobian geometricJacobian;
   private final DenseMatrix64F jointVelocitiesVector;
   private final DampedLeastSquaresJacobianSolver jacobianSolver;

   /**
    * Constructs a new SwingFullLegJacobian, for the given side of the robot
    */
   public SwingFullLegJacobian(RobotSide robotSide, FullRobotModel fullRobotModel)
   {
      this.robotSide = robotSide;
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody foot = fullRobotModel.getFoot(robotSide);
      geometricJacobian = new GeometricJacobian(pelvis, foot, foot.getBodyFixedFrame());
      jointVelocitiesVector = new DenseMatrix64F(geometricJacobian.getNumberOfColumns(), 1);
      jacobianSolver = new DampedLeastSquaresJacobianSolver(geometricJacobian.getNumberOfColumns());
   }

   /**
    * Computes the underlying openChainJacobian and the vtpJacobian
    */
   public void computeJacobian()
   {
      geometricJacobian.compute();
   }

   /**
    * @return the determinant of the Jacobian matrix
    */
   public double det()
   {
      return geometricJacobian.det();
   }

   /**
    * Returns the twist of the ankle pitch frame with respect to the pelvis frame, expressed in the ankle pitch frame,
    * corresponding to the given joint velocities.
    */
   public Twist getTwistOfFootWithRespectToPelvisInFootFrame(LegJointVelocities jointVelocities)
   {
      DenseMatrix64F jointVelocitiesVector = jointVelocities.toDenseMatrix();
      return geometricJacobian.getTwist(jointVelocitiesVector);
   }
   
   /**
    * Packs the joint velocities corresponding to the twist of the foot, with respect to the pelvis, expressed in ankle pitch frame
    * @param anklePitchTwistInAnklePitchFrame
    * @return corresponding joint velocities
    */
   public void packJointVelocitiesGivenTwist(LegJointVelocities legJointVelocitiesToPack, Twist anklePitchTwistInAnklePitchFrame, double alpha)
   {
      jacobianSolver.setAlpha(alpha);
      jacobianSolver.solve(jointVelocitiesVector, geometricJacobian.getJacobianMatrix(), anklePitchTwistInAnklePitchFrame.toMatrix());
      int i = 0;
      for (LegJointName legJointName : legJointVelocitiesToPack.getLegJointNames())
      {
         legJointVelocitiesToPack.setJointVelocity(legJointName, jointVelocitiesVector.get(i++, 0));
      }
   }

   /**
    * Packs a LegTorques object with the torques corresponding to the given wrench on the foot.
    */
   public void packLegTorques(LegTorques legTorquesToPack, Wrench wrenchOnFootInFootFrame)
   {
      // check that the LegTorques object we're packing has the correct RobotSide.
      if (this.robotSide != legTorquesToPack.getRobotSide())
      {
         throw new RuntimeException("legTorques object has the wrong RobotSide");
      }

      // the actual computation
      DenseMatrix64F jointTorques = geometricJacobian.computeJointTorques(wrenchOnFootInFootFrame);

      int i = 0;
      for (LegJointName legJointName : legTorquesToPack.getLegJointNames())
      {
         legTorquesToPack.setTorque(legJointName, jointTorques.get(i++, 0));
      }
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public DenseMatrix64F computeJointAccelerations(SpatialAccelerationVector accelerationOfFootWithRespectToBody, SpatialAccelerationVector jacobianDerivativeTerm, double alpha)
   {
      DenseMatrix64F biasedAccelerations = accelerationOfFootWithRespectToBody.toMatrix();    // unbiased at this point
      DenseMatrix64F bias = jacobianDerivativeTerm.toMatrix();
      CommonOps.subEquals(biasedAccelerations, bias);
      
      DenseMatrix64F ret = new DenseMatrix64F(geometricJacobian.getNumberOfColumns());
      jacobianSolver.setAlpha(alpha);
      jacobianSolver.solve(ret, geometricJacobian.getJacobianMatrix(), biasedAccelerations);
      
      return ret;
   }

   /**
    * For testing purposes only.
    */
   public DenseMatrix64F getJacobian()
   {
      return geometricJacobian.getJacobianMatrix().copy();
   }
   
   public String toString()
   {
      return geometricJacobian.toString();
   }

   public GeometricJacobian getGeometricJacobian()
   {
      return geometricJacobian;
   }
}
