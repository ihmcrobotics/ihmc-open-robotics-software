package us.ihmc.behaviors.targetFollowing;

import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.log.LogTools;

import static us.ihmc.behaviors.targetFollowing.TargetFollowingBehaviorAPI.*;

public class TargetFollowingBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Target Following", TargetFollowingBehavior::new, TargetFollowingBehaviorAPI.API);

   private final BehaviorHelper helper;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final TargetFollowingBehaviorParameters parameters;

   public TargetFollowingBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");
      parameters = new TargetFollowingBehaviorParameters();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      addChild(lookAndStepBehavior);
      helper.subscribeViaCallback(Parameters, parameters ->
      {
         helper.getOrCreateStatusLogger().info("Accepting new target following parameters");
         this.parameters.setAllFromStrings(parameters);
      });
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return lookAndStepBehavior.tick();
   }

   @Override
   public void reset()
   {

   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }

   @Override
   public void destroy()
   {
      lookAndStepBehavior.destroy();
   }
}


