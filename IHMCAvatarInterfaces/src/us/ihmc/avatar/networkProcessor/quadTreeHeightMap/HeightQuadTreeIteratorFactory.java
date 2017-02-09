package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.ArrayDeque;
import java.util.Iterator;

public class HeightQuadTreeIteratorFactory
{
   
   public static Iterable<HeightQuadTreeNode> iterable(HeightQuadTree heightQuadTree)
   {
      return new BaseIterable(heightQuadTree.getRoot(), null);
   }

   public static Iterable<HeightQuadTreeNode> leafIterable(HeightQuadTree heightQuadTree)
   {
      return new BaseIterable(heightQuadTree.getRoot(), leafSelectionRule());
   }

   private static IteratorSelectionRule leafSelectionRule()
   {
      return new IteratorSelectionRule()
      {
         @Override
         public boolean test(HeightQuadTreeNode node)
         {
            return !node.hasChildrenArray();
         }
      };
   }

   private static class BaseIterable implements Iterable<HeightQuadTreeNode>
   {
      private final HeightQuadTreeNode root;
      private final IteratorSelectionRule rule;

      public BaseIterable(HeightQuadTreeNode root, IteratorSelectionRule rule)
      {
         this.root = root;
         this.rule = rule;
      }

      @Override
      public Iterator<HeightQuadTreeNode> iterator()
      {
         return new BaseIterator(root, rule);
      }
   }

   private static class BaseIterator implements Iterator<HeightQuadTreeNode>
   {
      private final IteratorSelectionRule rule;
      /// Internal recursion stack.
      private final ArrayDeque<HeightQuadTreeNode> stack = new ArrayDeque<>();

      public BaseIterator(HeightQuadTreeNode root, IteratorSelectionRule rule)
      {
         this.rule = rule;

         hasNextHasBeenCalled = false;
         stack.clear();

         if (root != null)
         { // tree is not empty
            stack.add(root);
         }
      }

      private HeightQuadTreeNode next = null;
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
      public HeightQuadTreeNode next()
      {
         if (!hasNextHasBeenCalled)
         {
            if (!hasNext())
               throw new NullPointerException();
         }

         hasNextHasBeenCalled = false;
         HeightQuadTreeNode ret = next;
         next = null;
         return ret;
      }

      private HeightQuadTreeNode searchNextNodePassingRule()
      {
         if (stack.isEmpty())
            return null;

         if (rule == null)
            return searchNextNode();

         while (!stack.isEmpty())
         {
            HeightQuadTreeNode currentNode = searchNextNode();
            if (currentNode == null || rule.test(currentNode))
               return currentNode;
         }
         return null;
      }

      private HeightQuadTreeNode searchNextNode()
      {
         if (stack.isEmpty())
            return null;

         HeightQuadTreeNode currentNode = stack.poll();

         if (currentNode.hasChildrenArray())
         {
            // push on stack in reverse order
            for (int i = 3; i >= 0; i--)
            {
               HeightQuadTreeNode child = currentNode.getChild(i);
               if (child != null)
                  stack.add(child);
            }
         }

         return currentNode;
      }
   }

   public static interface IteratorSelectionRule
   {
      public boolean test(HeightQuadTreeNode node);
   }
}
