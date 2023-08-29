package us.ihmc.commonWalkingControlModules.inertialParameterEstimation;

import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialInertiaBasisOption;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.inertial.YoSpatialInertia;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Random;

public class InertialParameterEstimator
{
   private static final int RANDOM_SEED = 45;
   private static final Random random = new Random(RANDOM_SEED);

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final FullHumanoidRobotModel actualRobotModel;
   private final MultiBodySystemBasics estimatedMultiBodySystem;
   private final HashMap<RigidBodyReadOnly, YoSpatialInertia> actualSpatialInertiaHashMap = new LinkedHashMap<>();
   private final HashMap<RigidBodyReadOnly, YoSpatialInertia> estimatedSpatialInertiaHashMap = new LinkedHashMap<>();

   private final HashMap<RigidBodyReadOnly, EnumSet<SpatialInertiaBasisOption>> parametersToEstimateMap = new LinkedHashMap<>();

   public InertialParameterEstimator(FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.actualRobotModel = fullRobotModel;
      this.estimatedMultiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                                                                  actualRobotModel.getModelStationaryFrame(),
                                                                                                                                  "inertialParameterEstimator"));
      for (RigidBodyReadOnly body : actualRobotModel.getRootBody()
                                                    .subtreeArray())  // TODO: might somehow need getElevator here, but that throws NullPointerException for spatial inertia (expectedly)
      {
         actualSpatialInertiaHashMap.put(body, new YoSpatialInertia(body.getInertia(), "actual", registry));
         estimatedSpatialInertiaHashMap.put(body, new YoSpatialInertia(body.getInertia(), "estimated", registry));
         parametersToEstimateMap.put(body, SpatialInertiaBasisOption.generateRandomBasisSetForLink(random));
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      System.out.println("Blah");
   }
}
