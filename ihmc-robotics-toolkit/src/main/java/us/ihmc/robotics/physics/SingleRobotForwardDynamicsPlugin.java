package us.ihmc.robotics.physics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

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

   private final DMatrixRMaj jointVelocityMatrix;

   public SingleRobotForwardDynamicsPlugin(MultiBodySystemBasics input)
   {
      this.input = input;
      forwardDynamicsCalculator = new ForwardDynamicsCalculator(input);
      jointVelocityMatrix = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(input.getJointsToConsider()), 1);
   }

   public void setControllerOutputWriter(MultiBodySystemStateWriter controllerOutputWriter)
   {
      this.controllerOutputWriter = controllerOutputWriter;
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

   public void addJointVelocities(DMatrixRMaj jointVelocityToAdd)
   {
      if (jointVelocityToAdd != null)
         CommonOps_DDRM.addEquals(jointVelocityMatrix, jointVelocityToAdd);
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