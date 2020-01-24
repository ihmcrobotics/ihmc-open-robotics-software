package us.ihmc.humanoidBehaviors;

import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.humanoidBehaviors.navigation.NavigationBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

import java.util.ArrayList;

public class BehaviorRegistry
{
   public static final BehaviorRegistry DEFAULT_BEHAVIORS = new BehaviorRegistry();
   static
   {
      DEFAULT_BEHAVIORS.add(StepInPlaceBehavior.STATICS);
      DEFAULT_BEHAVIORS.add(PatrolBehavior.STATICS);
      DEFAULT_BEHAVIORS.add(FancyPosesBehavior.STATICS);
      DEFAULT_BEHAVIORS.add(ExploreAreaBehavior.STATICS);
      DEFAULT_BEHAVIORS.add(NavigationBehavior.STATICS);
   }

   private final ArrayList<BehaviorStatics> entries = new ArrayList<>();

   public static BehaviorRegistry of(BehaviorStatics... entries)
   {
      BehaviorRegistry registry = new BehaviorRegistry();
      for (BehaviorStatics entry : entries)
      {
         registry.add(entry);
      }
      return registry;
   }

   public void add(BehaviorStatics statics)
   {
      entries.add(statics);
   }

   public MessagerAPI constructMessagerAPI()
   {
      MessagerAPI[] behaviorAPIs = new MessagerAPI[entries.size()];
      for (int i = 0; i < entries.size(); i++)
      {
         behaviorAPIs[i] = entries.get(i).getBehaviorAPI();
      }
      return BehaviorModule.API.create(behaviorAPIs);
   }

   public ArrayList<BehaviorStatics> getEntries()
   {
      return entries;
   }
}
