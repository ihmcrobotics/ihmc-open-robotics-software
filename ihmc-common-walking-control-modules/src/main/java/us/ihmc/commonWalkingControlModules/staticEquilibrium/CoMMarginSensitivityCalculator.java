package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.inverseKinematics.RobotJointVelocityAccelerationIntegrator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;

// TODO should be removed before finalizing
public class CoMMarginSensitivityCalculator
{
   public static final double DT = 1.0e-2;

   private final StabilityMarginOptimizationModule optimizationModule;
   private final RobotJointVelocityAccelerationIntegrator integrator = new RobotJointVelocityAccelerationIntegrator(DT);
   private final FullHumanoidRobotModel fullRobotModel;
   private final JointBasics[] controlledJoints;
   private final WholeBodyContactState contactState;

   // TODO change to give option to operate on the same robot model?
   public CoMMarginSensitivityCalculator(FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory)
   {
      fullRobotModel = fullHumanoidRobotModelFactory.createFullRobotModel();
      this.optimizationModule = new CenterOfMassStabilityMarginOptimizationModule(fullRobotModel.getTotalMass());
      controlledJoints = HighLevelHumanoidControllerToolbox.computeJointsToOptimizeFor(fullRobotModel);
      OneDoFJointBasics[] controlledOneDoFJoints = MultiBodySystemTools.filterJoints(controlledJoints, OneDoFJointBasics.class);
      contactState = new WholeBodyContactState(controlledOneDoFJoints, fullRobotModel.getRootJoint());
   }

   public void setRobotState(JointBasics[] source)
   {
      for (int jointIndex = 0; jointIndex < source.length; jointIndex++)
      {
         JointReadOnly sourceJoint = source[jointIndex];
         JointBasics destinationJoint = controlledJoints[jointIndex];
         destinationJoint.setJointConfiguration(sourceJoint);
      }

      fullRobotModel.updateFrames();
   }

   public void integrateRobotState(DMatrixRMaj velocity)
   {
      integrator.integrateJointVelocities(controlledJoints, velocity);
      MultiBodySystemTools.insertJointsState(controlledJoints, JointStateType.CONFIGURATION, integrator.getJointConfigurations());
      fullRobotModel.updateFrames();
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public WholeBodyContactState getContactState()
   {
      return contactState;
   }

   public Point2D computeVertex(TIntArrayList basisIndices)
   {
      optimizationModule.updateContactState(contactState);
      return optimizationModule.solveForFixedBasis(basisIndices);
   }
}