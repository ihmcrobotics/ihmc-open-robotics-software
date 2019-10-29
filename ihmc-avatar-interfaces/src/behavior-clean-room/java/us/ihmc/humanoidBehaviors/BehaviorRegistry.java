package us.ihmc.humanoidBehaviors;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.fancyPoses.FancyPosesBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.ros2.Ros2Node;

public enum BehaviorRegistry
{
   STEP_IN_PLACE(StepInPlaceBehavior::new, StepInPlaceBehavior.API.create()),
   PATROL(PatrolBehavior::new, PatrolBehaviorAPI.create()),
   FANCY_POSES(FancyPosesBehavior::new, FancyPosesBehavior.API.create()),
   EXPLORE(ExploreAreaBehavior::new, ExploreAreaBehavior.ExploreAreaBehaviorAPI.create()),
   ;

   public static final BehaviorRegistry[] values = values();

   private final BehaviorSupplier behaviorSupplier;
   private final MessagerAPI behaviorAPI;

   private BehaviorInterface constructedBehavior;

   BehaviorRegistry(BehaviorSupplier behaviorSupplier, MessagerAPI behaviorAPI)
   {
      this.behaviorSupplier = behaviorSupplier;
      this.behaviorAPI = behaviorAPI;
   }

   public void build(DRCRobotModel robotModel, Messager messager, Ros2Node ros2Node)
   {
      constructedBehavior = behaviorSupplier.build(new BehaviorHelper(robotModel, messager, ros2Node));
   }

   public MessagerAPI getBehaviorAPI()
   {
      return behaviorAPI;
   }

   public BehaviorInterface getConstructedBehavior()
   {
      return constructedBehavior;
   }
}
