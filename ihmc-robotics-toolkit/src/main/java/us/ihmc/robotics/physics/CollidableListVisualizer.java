package us.ihmc.robotics.physics;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
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

      try
      {
         CollidableVisualizer collidableVisualizer = new CollidableVisualizer(name,
                                                                              groupName,
                                                                              collidable,
                                                                              appearanceDefinition,
                                                                              registry,
                                                                              yoGraphicsListRegistry);
         collidableVisualizerMap.put(collidable, collidableVisualizer);
      }
      catch (UnsupportedOperationException e)
      {
         LogTools.error(e.getMessage());
      }
   }

   public void update(CollisionListResult collisionListResult)
   {
      update(collisionListResult.stream().filter(collisionResult -> collisionResult.getCollisionData().areShapesColliding())
                                .flatMap(collisionResult -> Stream.of(collisionResult.getCollidableA(), collisionResult.getCollidableB()))
                                .collect(Collectors.toList()));
   }

   public void update(Collection<Collidable> collidablesToShow)
   {
      collidablesToShow = collidablesToShow.stream().filter(candidate -> collidableVisualizerMap.containsKey(candidate)).collect(Collectors.toSet());

      for (Entry<Collidable, CollidableVisualizer> entry : collidableVisualizerMap.entrySet())
      {
         if (collidablesToShow.contains(entry.getKey()))
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
