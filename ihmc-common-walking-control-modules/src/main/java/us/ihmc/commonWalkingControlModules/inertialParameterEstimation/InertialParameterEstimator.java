package us.ihmc.commonWalkingControlModules.inertialParameterEstimation;

import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.yoVariables.multiBodySystem.inertial.YoSpatialInertia;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.LinkedHashMap;

public class InertialParameterEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final FullHumanoidRobotModel actualRobotModel;
   private final MultiBodySystemBasics estimatedMultiBodySystem;
   private final HashMap<RigidBodyReadOnly, YoSpatialInertia> yoSpatialInertiaHashMap = new LinkedHashMap<>();

   public InertialParameterEstimator(FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this.actualRobotModel = fullRobotModel;
      this.estimatedMultiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(MultiBodySystemFactories.cloneMultiBodySystem(actualRobotModel.getElevator(),
                                                                                                                                  actualRobotModel.getModelStationaryFrame(),
                                                                                                                                  "inertialParameterEstimator"));
      for (RigidBodyReadOnly body : actualRobotModel.getRootBody()
                                                    .subtreeArray())  // TODO: might somehow need getElevator here, but that throws NullPointerException for spatial inertia (expectedly)
      {
         yoSpatialInertiaHashMap.put(body, new YoSpatialInertia(body.getInertia(), registry));
         System.out.println(body.getInertia());
      }

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      System.out.println("Blah");
   }
}
