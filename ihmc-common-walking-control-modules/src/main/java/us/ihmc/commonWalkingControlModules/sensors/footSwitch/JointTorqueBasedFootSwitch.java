package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commonWalkingControlModules.touchdownDetector.JointTorqueBasedTouchdownDetector;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointTorqueBasedFootSwitch implements FootSwitchInterface
{
   private final YoRegistry registry;

   private final JointTorqueBasedTouchdownDetector touchdownDetector;
   private final JacobianBasedWrenchCalculator wrenchCalculator;

   private final ReferenceFrame soleFrame;

   public JointTorqueBasedFootSwitch(RobotSide footSide, FullHumanoidRobotModel fullRobotModel, OneDoFJointReadOnly jointToRead, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(jointToRead.getName() + getClass().getSimpleName());

      jointToRead = fullRobotModel.getLegJoint(footSide, LegJointName.KNEE_PITCH);
      touchdownDetector = new JointTorqueBasedTouchdownDetector("", jointToRead, true, registry);

      soleFrame = fullRobotModel.getSoleFrame(footSide);
      wrenchCalculator = new JacobianBasedWrenchCalculator(footSide, fullRobotModel, soleFrame);

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      touchdownDetector.reset();
   }

   @Override
   public void update()
   {
      touchdownDetector.update();

      wrenchCalculator.calculate();
   }

   @Override
   public boolean hasFootHitGroundSensitive()
   {
      return touchdownDetector.hasTouchedDown();
   }

   /**
    * The joint has a really high torque on it.
    */
   @Override
   public boolean hasFootHitGroundFiltered()
   {
      return touchdownDetector.hasForSureTouchedDown();
   }

   @Override
   public double getFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public FramePoint2DReadOnly getCenterOfPressure()
   {
      return null;
   }

   @Override
   public WrenchReadOnly getMeasuredWrench()
   {
      return wrenchCalculator.getWrench();
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return soleFrame;
   }

   private static class JacobianBasedWrenchCalculator
   {
      private final GeometricJacobian footJacobian;
      private final OneDoFJointReadOnly[] legJoints;
      private final boolean isInvertible;

      private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
      private final DMatrixRMaj torqueVector = new DMatrixRMaj(6, 1);
      private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(6);

      private final Wrench wrench = new Wrench();

      public JacobianBasedWrenchCalculator(RobotSide footSide, FullHumanoidRobotModel fullRobotModel, ReferenceFrame soleFrame)
      {
         RigidBodyBasics  foot = fullRobotModel.getFoot(footSide);
         RigidBodyBasics pelvis = fullRobotModel.getPelvis();

         footJacobian = new GeometricJacobian(pelvis, foot, soleFrame);
         wrench.setToZero(ReferenceFrame.getWorldFrame());

         legJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, foot);
         isInvertible = legJoints.length == 6;

         if (!isInvertible)
            throw new RuntimeException("We can't yet use the Jacobian Based Wrench calculator, becuase the Jacobian isn't square. We need to implement this with a pseudo inverse.");
      }

      public void calculate()
      {
         footJacobian.compute();
         DMatrixRMaj jacobianMatrix = footJacobian.getJacobianMatrix();

         for (int i = 0; i < legJoints.length; i++)
            torqueVector.set(i, 0, legJoints[i].getTau());

         solver.setA(jacobianMatrix);
         CommonOps_DDRM.scale(-1.0, torqueVector);

         solver.solve(wrenchVector, torqueVector);

         wrench.setToZero(footJacobian.getJacobianFrame());
         wrench.set(wrenchVector);
      }

      public WrenchReadOnly getWrench()
      {
         return wrench;
      }
   }
}
