package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.SpatialVectorMessage;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class CollisionDetection
{
   HashMap<RobotSide, YoDouble> sigmaDot = new HashMap<>(RobotSide.values().length);
   HashMap<RobotSide, YoDouble> sigma = new HashMap<>(RobotSide.values().length);

   private final FullHumanoidRobotModel fullRobotModel;
   private final RigidBodyBasics baseBody;
   private RigidBodyBasics endEffector;

   private final GeometricJacobianCalculator jacobianCalculator = new GeometricJacobianCalculator();
   private DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);
   private DMatrixRMaj jacobianMatrix = new DMatrixRMaj(0, 0);
   private DMatrixRMaj forceMatrix = new DMatrixRMaj(6,1);
   private DMatrixRMaj velocityMatrix = new DMatrixRMaj(0, 0);
   private DMatrixRMaj powerMatrix = new DMatrixRMaj(1, 1);
   private final List<RigidBodyReadOnly> bodyPathFromBaseToEndEffector = new ArrayList<>(12);

   private final HashMap<RobotSide, List<Integer>> jointMap = new HashMap<>(RobotSide.values().length);

   double minSigma;
   double kd;
   double time = Double.NaN;

   CollisionDetection(double minSigma, double kd, FullHumanoidRobotModel fullRobotModel, YoRegistry registry)
   {
      this.minSigma = minSigma;
      this.kd = kd;

      for (RobotSide robotSide : RobotSide.values())
      {
         sigmaDot.put(robotSide, new YoDouble(fullRobotModel.getHand(robotSide) + "SigmaDot", registry));
         sigma.put(robotSide, new YoDouble(fullRobotModel.getHand(robotSide) +"Sigma" + robotSide.getPascalCaseName(), registry));
      }

      jointMap.put(RobotSide.LEFT, Arrays.asList(12, 13, 14, 22, 23, 24, 25, 26, 27, 28));
      jointMap.put(RobotSide.RIGHT, Arrays.asList(12, 13 ,14, 15, 16, 17, 18, 19, 20, 21));

      this.fullRobotModel = fullRobotModel;
      baseBody = fullRobotModel.getElevator().getChildrenJoints().get(0).getSuccessor();

   }

   public boolean detectCollision(HashMap<RobotSide, SpatialVectorMessage> handWrenches, us.ihmc.idl.IDLSequence.Float jointVelocities, double currentTime)
   {
      if (Double.isNaN(time) || currentTime < time)
      {
         time = currentTime;
         return false;
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         endEffector = fullRobotModel.getHand(robotSide);
         jacobianCalculator.clear();
         jacobianCalculator.setKinematicChain(baseBody, endEffector);
         jacobianCalculator.setJacobianFrame(endEffector.getBodyFixedFrame());
         jacobianCalculator.reset();
         MultiBodySystemTools.collectRigidBodyPath(baseBody, endEffector, bodyPathFromBaseToEndEffector);

         jacobianMatrix = jacobianCalculator.getJacobianMatrix();
         jacobianMatrix.reshape(6, jacobianCalculator.getNumberOfDegreesOfFreedom());

         forceMatrix.set(0, 0, handWrenches.get(robotSide).getLinearPart().getX());
         forceMatrix.set(1, 0, handWrenches.get(robotSide).getLinearPart().getY());
         forceMatrix.set(2, 0, handWrenches.get(robotSide).getLinearPart().getZ());
         forceMatrix.set(3, 0, handWrenches.get(robotSide).getAngularPart().getX());
         forceMatrix.set(4, 0, handWrenches.get(robotSide).getAngularPart().getY());
         forceMatrix.set(5, 0, handWrenches.get(robotSide).getAngularPart().getZ());

         getJointValues(robotSide, jointVelocities, velocityMatrix, true);
         tempMatrix.reshape(jacobianMatrix.getNumCols(), forceMatrix.getNumCols());
         CommonOps_DDRM.multTransA(jacobianMatrix, forceMatrix, tempMatrix);

         CommonOps_DDRM.mult(velocityMatrix, tempMatrix, powerMatrix);

         sigmaDot.get(robotSide).set(-kd*kd*sigma.get(robotSide).getDoubleValue() + kd*powerMatrix.get(0, 0));
         sigma.get(robotSide).set(sigma.get(robotSide).getDoubleValue() + sigmaDot.get(robotSide).getDoubleValue()*(currentTime - time));
         LogTools.info(robotSide.getCamelCaseName() + " - Sigma: " + sigma.get(robotSide).getDoubleValue() + " SigmaDot: " + sigmaDot.get(robotSide).getDoubleValue() + " Power: " + powerMatrix.get(0, 0));
         if (Math.abs(sigma.get(robotSide).getDoubleValue()) > minSigma)
         {
            return true;
         }
      }
      time = currentTime;
      return false;
   }

   public void getJointValues(RobotSide robotSide, us.ihmc.idl.IDLSequence.Float jointValues, DMatrixRMaj matrixToPack)
   {
      getJointValues(robotSide, jointValues, matrixToPack, false);
   }

   public void getJointValues(RobotSide robotSide, us.ihmc.idl.IDLSequence.Float jointValues, DMatrixRMaj matrixToPack, boolean transpose)
   {
      if (transpose)
         matrixToPack.reshape(1, jointMap.get(robotSide).size());
      else
         matrixToPack.reshape(jointMap.get(robotSide).size(), 1);
      for (int i = 0; i < jointMap.get(robotSide).size(); i++)
      {
         if (transpose)
            matrixToPack.set(0, i, jointValues.get(jointMap.get(robotSide).get(i)));
         else
            matrixToPack.set(i, 0, jointValues.get(jointMap.get(robotSide).get(i)));
      }
   }
}
