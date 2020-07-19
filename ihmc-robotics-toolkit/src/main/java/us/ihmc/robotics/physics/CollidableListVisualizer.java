package us.ihmc.robotics.physics;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CollidableListVisualizer
{
   private final static String staticCollidableName = "environment";

   private final String groupName;
   private final AppearanceDefinition appearanceDefinition;
   private final YoRegistry registry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private int staticCollidableCounter = 0;
   private final TObjectIntHashMap<RigidBodyBasics> rigidBodyCollidableCounterMap = new TObjectIntHashMap<>();
   private final Map<Collidable, CollidableVisualizer> collidableVisualizerMap = new HashMap<>();

   public CollidableListVisualizer(String groupName, AppearanceDefinition appearanceDefinition, YoRegistry registry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.groupName = groupName;
      this.appearanceDefinition = appearanceDefinition;
      this.registry = registry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
   }

   public void addCollidable(Collidable collidable)
   {
      String name;

      if (collidable.getRigidBody() == null)
      {
         name = staticCollidableName + Integer.toString(staticCollidableCounter++);
      }
      else
      {
         int counter = rigidBodyCollidableCounterMap.adjustOrPutValue(collidable.getRigidBody(), 1, 0);
         name = collidable.getRigidBody().getName() + Integer.toString(counter);
      }
      CollidableVisualizer collidableVisualizer = new CollidableVisualizer(name, groupName, collidable, appearanceDefinition, registry, yoGraphicsListRegistry);
      collidableVisualizerMap.put(collidable, collidableVisualizer);
   }

   public void update(CollisionListResult collisionListResult)
   {
      Set<Collidable> collidableToShow = collisionListResult.stream().filter(collisionResult -> collisionResult.getCollisionData().areShapesColliding())
                                                            .flatMap(collisionResult -> Stream.of(collisionResult.getCollidableA(),
                                                                                                  collisionResult.getCollidableB()))
                                                            .filter(candidate -> collidableVisualizerMap.containsKey(candidate)).collect(Collectors.toSet());

      for (Entry<Collidable, CollidableVisualizer> entry : collidableVisualizerMap.entrySet())
      {
         if (collidableToShow.contains(entry.getKey()))
         {
            entry.getValue().update();
         }
         else
         {
            entry.getValue().hide();
         }
      }
   }
}
