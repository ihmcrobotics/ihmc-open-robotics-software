package us.ihmc.gdx.ui.behaviors;

import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeBasics;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.SequenceNodeBasics;

import java.util.ArrayList;

public class ExampleSequenceGDXBehaviorUIInterface extends ExampleAbstractGDXBehaviorUIInterface implements SequenceNodeBasics
{
   @Override
   public double evaluateUtility()
   {
      return super.evaluateUtility();
   }

   @Override
   public void clock()
   {

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
   public BehaviorTreeNodeStatus tick()
   {
      return super.tick();
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
