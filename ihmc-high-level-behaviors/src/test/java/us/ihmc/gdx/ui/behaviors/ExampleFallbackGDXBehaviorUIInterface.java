package us.ihmc.gdx.ui.behaviors;

import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeBasics;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.FallbackNodeBasics;

import java.util.ArrayList;

public class ExampleFallbackGDXBehaviorUIInterface extends ExampleAbstractGDXBehaviorUIInterface implements FallbackNodeBasics
{
   @Override
   public double evaluateUtility()
   {
      return super.evaluateUtility();
   }

   @Override
   public void clock()
   {
      FallbackNodeBasics.super.clock();
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      return super.tick();
   }

   @Override
   public <T extends BehaviorTreeNodeBasics> T addChild(T child)
   {
      return null;
   }

   @Override
   public ArrayList<BehaviorTreeNodeBasics> getChildren()
   {
      return null;
   }

   @Override
   public void setHasBeenClocked(boolean hasBeenClocked)
   {

   }

   @Override
   public boolean getHasBeenClocked()
   {
      return false;
   }
}
