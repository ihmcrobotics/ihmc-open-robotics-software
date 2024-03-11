package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HumanoidModelCovarianceHelper
{
   private final YoRegistry registry;

   private final int[] floatingBaseJointIndices;
   private final SideDependentList<int[]> legJointIndices;
   private final int[] spineJointIndices;
   private final SideDependentList<int[]> armJointIndices;
   private final YoDouble[] floatingBaseMeasurementCovariance;
   private final SideDependentList<YoDouble> legMeasurementCovariances;
   private final YoDouble spineMeasurementCovariance;
   private final SideDependentList<YoDouble> armMeasurementCovariances;
   private final DMatrixRMaj measurementCovariance;

   HumanoidModelCovarianceHelper(FullHumanoidRobotModel model, InertialEstimationParameters parameters, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      floatingBaseJointIndices = parameters.getFloatingBaseJointIndices();
      spineJointIndices = parameters.getSpineJointIndices();
      legJointIndices = parameters.getLegJointIndices();
      armJointIndices = parameters.getArmJointIndices();

      String prefix = "floatingBaseMeasurementCovariance";
      String[] suffixes = new String[] {"wX", "wY", "wZ", "x", "y", "z"};
      floatingBaseMeasurementCovariance = new YoDouble[model.getRootJoint().getDegreesOfFreedom()];
      for (int i = 0; i < floatingBaseMeasurementCovariance.length; i++)
      {
         floatingBaseMeasurementCovariance[i] = new YoDouble(prefix + "_" + suffixes[i], registry);
         floatingBaseMeasurementCovariance[i].set(parameters.getFloatingBaseMeasurementCovariance());
      }
      spineMeasurementCovariance = new YoDouble("spineMeasurementCovariance", registry);
      spineMeasurementCovariance.set(parameters.getSpineMeasurementCovariance());
      legMeasurementCovariances = new SideDependentList<>();
      armMeasurementCovariances = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         legMeasurementCovariances.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "LegMeasurementCovariance", registry));
         legMeasurementCovariances.get(robotSide).set(parameters.getLegMeasurementCovariance());
         armMeasurementCovariances.put(robotSide, new YoDouble(robotSide.getCamelCaseNameForStartOfExpression() + "ArmMeasurementCovariance", registry));
         armMeasurementCovariances.get(robotSide).set(parameters.getArmMeasurementCovariance());
      }

      int nDoFs = MultiBodySystemTools.computeDegreesOfFreedom(model.getRootJoint().subtreeArray());
      measurementCovariance = new DMatrixRMaj(nDoFs, nDoFs);
   }

   /**
    * TODO: note this method will only modify the diagonals and will leave all other elements untouched, be careful about them
    * @param
    * @param
    */
   public DMatrixRMaj getMeasurementCovariance()
   {
      for (int floatingBaseJointIndex : floatingBaseJointIndices)
         measurementCovariance.set(floatingBaseJointIndex, floatingBaseJointIndex, floatingBaseMeasurementCovariance[floatingBaseJointIndex].getValue());

      for (int spineJointIndex : spineJointIndices)
         measurementCovariance.set(spineJointIndex, spineJointIndex, spineMeasurementCovariance.getValue());

      for (RobotSide robotSide : RobotSide.values)
      {
         int[] legIndices = legJointIndices.get(robotSide);
         for (int legJointIndex : legIndices)
            measurementCovariance.set(legJointIndex, legJointIndex, legMeasurementCovariances.get(robotSide).getValue());

         int[] armIndices = armJointIndices.get(robotSide);
         for (int armJointIndex : armIndices)
            measurementCovariance.set(armJointIndex, armJointIndex, armMeasurementCovariances.get(robotSide).getValue());
      }

      return measurementCovariance;
   }
}