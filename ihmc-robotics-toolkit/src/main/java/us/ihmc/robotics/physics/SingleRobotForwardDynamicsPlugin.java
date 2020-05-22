package us.ihmc.robotics.physics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class SingleRobotForwardDynamicsPlugin
{
   private final MultiBodySystemBasics input;
   private final ForwardDynamicsCalculator forwardDynamicsCalculator;

   private MultiBodySystemStateWriter controllerOutputWriter;

   private final DenseMatrix64F jointVelocityMatrix;

   public SingleRobotForwardDynamicsPlugin(MultiBodySystemBasics input)
   {
      this(input, null);
   }

   public SingleRobotForwardDynamicsPlugin(MultiBodySystemBasics input, MultiBodySystemStateWriter controllerOutputWriter)
   {
      this.input = input;
      this.controllerOutputWriter = controllerOutputWriter;
      forwardDynamicsCalculator = new ForwardDynamicsCalculator(input);
      jointVelocityMatrix = new DenseMatrix64F(MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider()), 1);
   }

   public void doScience(double time, double dt, Vector3DReadOnly gravity)
   {
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      forwardDynamicsCalculator.compute();
   }

   public void readJointVelocities()
   {
      MultiBodySystemTools.extractJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, jointVelocityMatrix);
   }

   public void addJointVelocities(DenseMatrix64F jointVelocityToAdd)
   {
      if (jointVelocityToAdd != null)
         CommonOps.addEquals(jointVelocityMatrix, jointVelocityToAdd);
   }

   public void writeJointVelocities()
   {
      MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.VELOCITY, jointVelocityMatrix);
   }

   public void writeJointAccelerations()
   {
      MultiBodySystemTools.insertJointsState(input.getJointsToConsider(), JointStateType.ACCELERATION, forwardDynamicsCalculator.getJointAccelerationMatrix());
   }

   public void applyControllerOutput()
   {
      if (controllerOutputWriter != null)
         controllerOutputWriter.write();
   }

   public void resetExternalWrenches()
   {
      forwardDynamicsCalculator.setExternalWrenchesToZero();
   }

   public ForwardDynamicsCalculator getForwardDynamicsCalculator()
   {
      return forwardDynamicsCalculator;
   }
}