package us.ihmc.commonWalkingControlModules.kinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;

public class SwingFullLegJacobian
{
   private final YoVariableRegistry registry;
   private final RobotSide robotSide;
   private final GeometricJacobian geometricJacobian;
   private final DenseMatrix64F jointVelocitiesVector;
   private final DampedLeastSquaresJacobianSolver jacobianSolver;

   /**
    * Constructs a new SwingFullLegJacobian, for the given side of the robot
    * @param parentRegistry TODO
    */
   public SwingFullLegJacobian(RobotSide robotSide, FullHumanoidRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(robotSide.getSideNameFirstLetter() + getClass().getSimpleName());
      this.robotSide = robotSide;
      RigidBody pelvis = fullRobotModel.getPelvis();
      RigidBody foot = fullRobotModel.getFoot(robotSide);
      geometricJacobian = new GeometricJacobian(pelvis, foot, foot.getBodyFixedFrame());
      jointVelocitiesVector = new DenseMatrix64F(geometricJacobian.getNumberOfColumns(), 1);
      jacobianSolver = new DampedLeastSquaresJacobianSolver(robotSide.getCamelCaseNameForStartOfExpression() + "JacobianSolver", geometricJacobian.getNumberOfColumns(), registry);
      parentRegistry.addChild(registry);
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
      DenseMatrix64F twistMatrix = new DenseMatrix64F(Twist.SIZE, 1);
      CommonOps.mult(getJacobian(), jointVelocitiesVector, twistMatrix);
      Twist ret = new Twist(geometricJacobian.getEndEffectorFrame(), geometricJacobian.getBaseFrame(), geometricJacobian.getJacobianFrame(), twistMatrix);
      return ret;
   }
   
   /**
    * Packs the joint velocities corresponding to the twist of the foot, with respect to the pelvis, expressed in ankle pitch frame
    * @param anklePitchTwistInAnklePitchFrame
    * @return corresponding joint velocities
    */
   public void packJointVelocitiesGivenTwist(LegJointVelocities legJointVelocitiesToPack, Twist anklePitchTwistInAnklePitchFrame, double alpha)
   {
      jacobianSolver.setAlpha(alpha);
      jacobianSolver.setJacobian(geometricJacobian.getJacobianMatrix());
      jacobianSolver.solve(jointVelocitiesVector, anklePitchTwistInAnklePitchFrame.toMatrix());
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
      CommonOps.subtractEquals(biasedAccelerations, bias);
      
      DenseMatrix64F ret = new DenseMatrix64F(geometricJacobian.getNumberOfColumns());
      jacobianSolver.setAlpha(alpha);
      jacobianSolver.setJacobian(geometricJacobian.getJacobianMatrix());
      jacobianSolver.solve(ret, biasedAccelerations);
      
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
