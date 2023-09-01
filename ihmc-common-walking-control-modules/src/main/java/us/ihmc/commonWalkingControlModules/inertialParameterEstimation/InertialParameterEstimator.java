package us.ihmc.commonWalkingControlModules.inertialParameterEstimation;

import org.ejml.dense.row.MatrixFeatures_DDRM;
import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.spatial.SpatialInertiaBasisOption;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoMultiBodySystem;
import us.ihmc.mecano.yoVariables.multiBodySystem.inertial.YoSpatialInertia;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Random;

public class InertialParameterEstimator
{
   private static final int RANDOM_SEED = 45;
   private static final Random random = new Random(RANDOM_SEED);
   private static final boolean DEBUG = true;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final FullHumanoidRobotModel actualRobotModel;
   private final FullHumanoidRobotModel estimateRobotModel;
   private final YoMultiBodySystem yoEstimateRobotModel;
   private final HashMap<RigidBodyReadOnly, YoSpatialInertia> actualSpatialInertiaHashMap = new LinkedHashMap<>();
   private final HashMap<RigidBodyReadOnly, YoSpatialInertia> estimatedSpatialInertiaHashMap = new LinkedHashMap<>();
   private final HashMap<RigidBodyReadOnly, EnumSet<SpatialInertiaBasisOption>> parametersToEstimateMap = new LinkedHashMap<>();
   private final JointTorqueRegressorCalculator jointTorqueRegressorCalculator;
   private final HashMap<JointReadOnly, YoDouble> jointVelocities = new LinkedHashMap<>();

   public InertialParameterEstimator(FullHumanoidRobotModel fullRobotModel, double controlDT, double gravity, YoRegistry parentRegistry)
   {
      actualRobotModel = fullRobotModel;
      RigidBodyBasics clonedElevator = MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                     actualRobotModel.getModelStationaryFrame(),
                                                                                     "_estimate");
      estimateRobotModel = new FullHumanoidRobotModelWrapper(clonedElevator, true);

      jointTorqueRegressorCalculator = new JointTorqueRegressorCalculator(estimateRobotModel.getElevator());
      jointTorqueRegressorCalculator.setGravitationalAcceleration(gravity);

      for (RigidBodyReadOnly body : actualRobotModel.getRootBody()
                                                    .subtreeArray())  // TODO: might somehow need getElevator here, but that throws NullPointerException for spatial inertia (expectedly)
      {
         actualSpatialInertiaHashMap.put(body, new YoSpatialInertia(body.getInertia(), "actual", registry));
         estimatedSpatialInertiaHashMap.put(body, new YoSpatialInertia(body.getInertia(), "estimated", registry));
         parametersToEstimateMap.put(body, SpatialInertiaBasisOption.generateRandomBasisSetForLink(random));
      }

      for (OneDoFJointReadOnly oneDoFJointReadOnly : estimateRobotModel.getOneDoFJoints())
         jointVelocities.put(oneDoFJointReadOnly, new YoDouble(oneDoFJointReadOnly.getName(), registry));

      parentRegistry.addChild(registry);

      if (DEBUG)
         yoEstimateRobotModel = new YoMultiBodySystem(MultiBodySystemReadOnly.toMultiBodySystemInput(estimateRobotModel.getElevator()),
                                                      estimateRobotModel.getModelStationaryFrame(),
                                                      registry);
      else
         yoEstimateRobotModel = null;
   }

   public void update()
   {
      for (JointStateType type : JointStateType.values())
      {
         MultiBodySystemTools.copyJointsState(actualRobotModel.getRootJoint().subtreeList(),  // TODO generating garbage here
                                              estimateRobotModel.getRootJoint().subtreeList(), type);
      }
      estimateRobotModel.getRootJoint().updateFramesRecursively();

      if (DEBUG)
      {
         for (JointStateType type : JointStateType.values())
         {
            MultiBodySystemTools.copyJointsState(estimateRobotModel.getRootJoint().subtreeList(),  // TODO generating garbage here
                                                 yoEstimateRobotModel.getAllJoints(), type);
         }
      }

      jointTorqueRegressorCalculator.compute();
      System.out.println(MatrixFeatures_DDRM.countNonZero(jointTorqueRegressorCalculator.getJointTorqueRegressorMatrix()));
   }
}
