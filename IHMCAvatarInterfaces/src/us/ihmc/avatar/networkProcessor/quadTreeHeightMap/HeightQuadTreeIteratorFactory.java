package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.ArrayDeque;
import java.util.Iterator;

import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeNodeMessage;

public class HeightQuadTreeIteratorFactory
{
   
   public static Iterable<HeightQuadTreeNodeMessage> iterable(HeightQuadTreeMessage heightQuadTree)
   {
      return new BaseIterable(heightQuadTree.root, null);
   }

   public static Iterable<HeightQuadTreeNodeMessage> leafIterable(HeightQuadTreeMessage heightQuadTree)
   {
      return new BaseIterable(heightQuadTree.root, leafSelectionRule());
   }

   private static IteratorSelectionRule leafSelectionRule()
   {
      return new IteratorSelectionRule()
      {
         @Override
         public boolean test(HeightQuadTreeNodeMessage node)
         {
            return node.children == null;
         }
      };
   }

   private static class BaseIterable implements Iterable<HeightQuadTreeNodeMessage>
   {
      private final HeightQuadTreeNodeMessage root;
      private final IteratorSelectionRule rule;

      public BaseIterable(HeightQuadTreeNodeMessage root, IteratorSelectionRule rule)
      {
         this.root = root;
         this.rule = rule;
      }

      @Override
      public Iterator<HeightQuadTreeNodeMessage> iterator()
      {
         return new BaseIterator(root, rule);
      }
   }

   private static class BaseIterator implements Iterator<HeightQuadTreeNodeMessage>
   {
      private final IteratorSelectionRule rule;
      /// Internal recursion stack.
      private final ArrayDeque<HeightQuadTreeNodeMessage> stack = new ArrayDeque<>();

      public BaseIterator(HeightQuadTreeNodeMessage root, IteratorSelectionRule rule)
      {
         this.rule = rule;

         hasNextHasBeenCalled = false;
         stack.clear();

         if (root != null)
         { // tree is not empty
            stack.add(root);
         }
      }

      private HeightQuadTreeNodeMessage next = null;
      private boolean hasNextHasBeenCalled = false;

      @Override
      public boolean hasNext()
      {
         next = null;

         if (stack.isEmpty())
            return false;

         if (!hasNextHasBeenCalled)
         {
            next = searchNextNodePassingRule();
            hasNextHasBeenCalled = true;
         }

         return next != null;
      }

      @Override
      public HeightQuadTreeNodeMessage next()
      {
         if (!hasNextHasBeenCalled)
         {
            if (!hasNext())
               throw new NullPointerException();
         }

         hasNextHasBeenCalled = false;
         HeightQuadTreeNodeMessage ret = next;
         next = null;
         return ret;
      }

      private HeightQuadTreeNodeMessage searchNextNodePassingRule()
      {
         if (stack.isEmpty())
            return null;

         if (rule == null)
            return searchNextNode();

         while (!stack.isEmpty())
         {
            HeightQuadTreeNodeMessage currentNode = searchNextNode();
            if (currentNode == null || rule.test(currentNode))
               return currentNode;
         }
         return null;
      }

      private HeightQuadTreeNodeMessage searchNextNode()
      {
         if (stack.isEmpty())
            return null;

         HeightQuadTreeNodeMessage currentNode = stack.poll();

         if (currentNode.children != null)
         {
            // push on stack in reverse order
            for (int i = 3; i >= 0; i--)
            {
               HeightQuadTreeNodeMessage child = currentNode.children[i];
               if (child != null)
                  stack.add(child);
            }
         }

         return currentNode;
      }
   }

   public static interface IteratorSelectionRule
   {
      public boolean test(HeightQuadTreeNodeMessage node);
   }
}
