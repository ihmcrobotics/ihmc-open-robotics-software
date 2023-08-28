package us.ihmc.commonWalkingControlModules.inertialParameterEstimation;

import us.ihmc.mecano.algorithms.JointTorqueRegressorCalculator;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoMultiBodySystem;
import us.ihmc.mecano.yoVariables.multiBodySystem.inertial.YoSpatialInertia;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Arrays;

public class InertialParameterEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final FullHumanoidRobotModel actualRobotModel;
   private final MultiBodySystemBasics estimatedMultiBodySystem;

   private final YoDouble torsoMass = new YoDouble("torsoMass", registry);

   private final YoDouble leftKneeTorque = new YoDouble("leftKneeTorque", registry);

   private static final boolean ENABLE_JOINT_STATE_YOVARIABLES = false;

   private final YoMultiBodySystem multiBodySystem;

   private YoSpatialInertia[] spatialInertias;

   public InertialParameterEstimator(FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.actualRobotModel = fullRobotModel;
      this.estimatedMultiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                                                                  actualRobotModel.getModelStationaryFrame(),
                                                                                                                                  "inertialParameterEstimator"));

      torsoMass.set(fullRobotModel.getChest().getInertia().getMass());
      leftKneeTorque.set(fullRobotModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).getTau());

      multiBodySystem = new YoMultiBodySystem(MultiBodySystemReadOnly.toMultiBodySystemInput(actualRobotModel.getElevator()),
                                              actualRobotModel.getModelStationaryFrame(),
                                              registry);

      for (RigidBodyReadOnly body : actualRobotModel.getElevator().subtreeArray())
      {
         System.out.println(body.toString());
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      System.out.println("Blah");
   }
}
