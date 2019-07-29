package us.ihmc.footstepPlanning.graphSearch.listeners;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.BodyCollisionNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class HeuristicSearchAndActionPolicyDefinitions
{
   private final AtomicBoolean automaticallyRotate = new AtomicBoolean(false);

   private final RequiredFactoryField<FootstepNodeSnapper> snapper = new RequiredFactoryField<>("snapper");
   private final RequiredFactoryField<FootstepPlannerParametersReadOnly> parameters= new RequiredFactoryField<>("parameters");
   private final OptionalFactoryField<BodyCollisionNodeChecker> collisionNodeChecker = new OptionalFactoryField<>("collisionNodeChecker");
   private final OptionalFactoryField<PlannerGoalRecommendationListener> goalRecommendationListener = new OptionalFactoryField<>("goalRecommendationListener");

   private final List<StartAndGoalListener> startAndGoalListeners = new ArrayList<>();
   private final List<BipedalFootstepPlannerListener> plannerListeners = new ArrayList<>();

   public void setGoalRecommendationListener(PlannerGoalRecommendationListener goalRecommendationListener)
   {
      this.goalRecommendationListener.set(goalRecommendationListener);
   }

   public void setCollisionNodeChecker(BodyCollisionNodeChecker collisionNodeChecker)
   {
      this.collisionNodeChecker.set(collisionNodeChecker);
   }

   public void setAutomaticallyRotate(boolean automaticallyRotate)
   {
      this.automaticallyRotate.set(automaticallyRotate);
   }

   public void setNodeSnapper(FootstepNodeSnapper snapper)
   {
      this.snapper.set(snapper);
   }

   public void setParameters(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters.set(parameters);
   }

   public void build()
   {
      PlannerGoalAdditionActionPolicy plannerGoalAdditionActionPolicy = new PlannerGoalAdditionActionPolicy(snapper.get());

      if (collisionNodeChecker.hasValue() && goalRecommendationListener.hasValue() && automaticallyRotate.get())
      {
         CollisionEndStanceFreeSearchPolicy bodyCollisionFreeSearchPolicy = new CollisionEndStanceFreeSearchPolicy(collisionNodeChecker.get(), snapper.get(), parameters.get());
         BodyCollisionListener bodyCollisionListener = new BodyCollisionListener();

         plannerGoalAdditionActionPolicy.addActionListener(goalRecommendationListener.get());
         bodyCollisionFreeSearchPolicy.attachActionPolicy(plannerGoalAdditionActionPolicy);
         bodyCollisionListener.setHeuristicSearchPolicy(bodyCollisionFreeSearchPolicy);

         plannerListeners.add(bodyCollisionListener);
         startAndGoalListeners.add(bodyCollisionFreeSearchPolicy);
      }
   }

   public List<BipedalFootstepPlannerListener> getPlannerListeners()
   {
      return plannerListeners;
   }

   public List<StartAndGoalListener> getStartAndGoalListeners()
   {
      return startAndGoalListeners;
   }
}
