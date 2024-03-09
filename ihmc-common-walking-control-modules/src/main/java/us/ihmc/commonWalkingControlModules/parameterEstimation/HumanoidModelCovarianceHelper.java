package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrix;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.InertialParameterManagerFactory.EstimatorType;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParameters;
import us.ihmc.parameterEstimation.inertial.RigidBodyInertialParametersTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.math.YoMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.Map;

public class HumanoidModelCovarianceHelper
{
   private final YoRegistry registry;

   private final int[] floatingBaseJointIndices;
   private final SideDependentList<int[]> legJointIndices = new SideDependentList<>();
   private final int[] spineJointIndices;
   private final SideDependentList<int[]> armJointIndices = new SideDependentList<>();

   private final YoDouble floatingBaseMeasurementCovariance;
   private final SideDependentList<YoDouble> legMeasurementCovariances;
   private final YoDouble spineMeasurementCovariance;
   private final SideDependentList<YoDouble> armMeasurementCovariances;
   private final YoMatrix measurementCovarianceDiagonal;

   HumanoidModelCovarianceHelper(FullHumanoidRobotModel model, InertialEstimationParameters parameters, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      floatingBaseJointIndices = parameters.getFloatingBaseJointIndices();
      spineJointIndices = parameters.getSpineJointIndices();
      for (RobotSide side : RobotSide.values)
      {
         legJointIndices.put(side, parameters.getLegJointIndices(side));
         armJointIndices.put(side, parameters.getArmJointIndices(side));
      }

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());

      // TODO: add name
      measurementCovarianceDiagonal = new YoMatrix("measurementCovarianceDiagonal", nDoFs, 1, registry);
   }

   public void set(int index, double value)
   {
      measurementCovarianceDiagonal.set(covarianceDiagonal);
   }
}