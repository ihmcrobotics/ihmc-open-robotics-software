package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNode;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.SequenceNode;
import us.ihmc.tools.Timer;

import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.RUNNING;
import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.SUCCESS;

public class ExploreAreaBehaviorLookAroundNode extends SequenceNode
{
   private final double expectedTickPeriod;
   private Timer deactivationTimer = new Timer();

   public ExploreAreaBehaviorLookAroundNode(double expectedTickPeriod)
   {
      this.expectedTickPeriod = expectedTickPeriod;

      addChild(new LookInADirection(-40.0, -20.0));
      addChild(new LookInADirection(0.0, 0.0));
      addChild(new LookInADirection(40.0, 20.0));
   }

   @Override
   public BehaviorTreeNodeStatus tick()
   {
      if (deactivationTimer.isExpired(expectedTickPeriod * 1.5))
      {
         for (BehaviorTreeNode child : getChildren())
         {
            ((LookInADirection) child).reset();
         }
      }

      deactivationTimer.reset();

      return super.tick();
   }

   class LookInADirection implements BehaviorTreeNode
   {
      private final double chestYaw;
      private final double headPitch;

      private boolean trajectoryStarted = false;
      private boolean trajectoryComplete = false;

      public LookInADirection(double chestYaw, double headPitch)
      {
         this.chestYaw = chestYaw;
         this.headPitch = headPitch;
      }

      @Override
      public BehaviorTreeNodeStatus tick()
      {
         if (!trajectoryStarted)
         {
            trajectoryStarted = true;
            ThreadTools.startAThread(this::commandAndWaitForTrajectory, "LookInADirectionTrajectory");
            return RUNNING;
         }
         else if (!trajectoryComplete)
         {
            return RUNNING;
         }
         else
         {
            return SUCCESS;
         }
      }

      public void reset()
      {
         trajectoryStarted = false;
         trajectoryComplete = false;
      }

      private void commandAndWaitForTrajectory()
      {
         trajectoryComplete = true;
      }
   }
}
