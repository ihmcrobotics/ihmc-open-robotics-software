package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.configurations.InertialEstimationParameters;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator.SpatialInertiaBasisOption;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Set;

/**
 * This class is used to manage construction and updating the covariance matrices for a {@link FullHumanoidRobotModel} when performing inertial estimation.
 * <p>
 * In inertial estimation, one must specify process covariances encoding the belief on how the inertial parameters of the rigid bodies being estimated are
 * expected to vary over time, and measurement covariances encoding the belief on the variance of the torque measurements from each joint (including the
 * floating base, which the contact wrenches are mapped to).
 * </p>
 *
 * @author James Foster
 */
public class HumanoidModelCovarianceHelper
{
   private final Set<JointTorqueRegressorCalculator.SpatialInertiaBasisOption>[] basisSets;

   /**
    * YoDoubles for the process covariances. The same covariances are used for each rigid body being estimated, so this YoDouble is 10-dimensional
    * (the size of the inertial parameter vector for one rigid body).
    */
   private final YoDouble[] processCovariances;
   /** Final container for the process covariances, which can be used elsewhere. */
   private final DMatrixRMaj processCovariance;

   private final int[] floatingBaseJointIndices;
   private final SideDependentList<int[]> legJointIndices;
   private final int[] spineJointIndices;
   private final SideDependentList<int[]> armJointIndices;
   /**
    * YoDoubles for the measurement covariances of the floating base joint. Notably, contact wrenches are mapped to this joint through
    * {@link us.ihmc.robotics.screwTheory.GeometricJacobian}
    */
   private final YoDouble[] floatingBaseMeasurementCovariance;
   /** Side-dependent YoDoubles for the measurement covariance of the leg joints. The same covariance is used for each joint in the given side's leg. */
   private final SideDependentList<YoDouble> legMeasurementCovariances;
   /** YoDouble for the measurement covariance of the spine joints. The same covariance is used for each joint in the spine. */
   private final YoDouble spineMeasurementCovariance;
   /** Side-dependent YoDouble arrays for the measurement covariance of the arm joints. The same covariance is used for each joint in the given side's arm. */
   private final SideDependentList<YoDouble> armMeasurementCovariances;
   /** Final container for the measurement covariances, which can be used elsewhere. */
   private final DMatrixRMaj measurementCovariance;

   HumanoidModelCovarianceHelper(FullHumanoidRobotModel model, InertialEstimationParameters parameters, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      parentRegistry.addChild(registry);

      basisSets = parameters.getBasisSets();
      RigidBodyReadOnly[] bodies = model.getRootBody().subtreeArray();
      double[] defaultProcessCovariance = parameters.getProcessCovariance();
      String[] basisNames = parameters.getBasisNames();
      processCovariances = new YoDouble[parameters.getNumberOfParameters()];
      if (basisSets.length != bodies.length)
         throw new RuntimeException("The number of basis sets does not match the number of bodies in the robot model.");
      for (int i = 0; i < bodies.length; i++)
      {
         RigidBodyReadOnly body = bodies[i];
         Set<SpatialInertiaBasisOption> set = basisSets[i];
         int j = 0;
         for (SpatialInertiaBasisOption option : SpatialInertiaBasisOption.values)
         {
            if (!set.contains(option))
               continue;

            processCovariances[j] = new YoDouble(body.getName() + "_processCovariance_" + basisNames[option.ordinal()], registry);
            processCovariances[j].set(defaultProcessCovariance[j]);
            j++;
         }
      }
      processCovariance = new DMatrixRMaj(parameters.getNumberOfParameters(), parameters.getNumberOfParameters());

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
    * Pack and return the process covariance matrix from the associated YoVariables.
    *
    * @return the process covariance matrix.
    */
   public DMatrixRMaj getProcessCovariance()
   {
      int i = 0;
      for (Set<SpatialInertiaBasisOption> set : basisSets)
      {
         if (set.isEmpty())
            continue;

         for (SpatialInertiaBasisOption option : JointTorqueRegressorCalculator.SpatialInertiaBasisOption.values)
         {
            if (set.contains(option))
            {
               processCovariance.set(i, i, processCovariances[option.ordinal()].getValue());
               i++;
            }
         }
      }
      return processCovariance;
   }

   /**
    * Pack and return the measurement covariance matrix from the associated YoVariables.
    *
    * @return the measurement covariance matrix.
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