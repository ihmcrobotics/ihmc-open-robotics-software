package us.ihmc.robotics.hyperCubeTree;

import java.util.ArrayList;
import java.util.List;

public class HyperCubeNode<T, D> implements RecursableHyperTreeNode<T, D>
{
   private final int dimensionality;
   private final int childNumber;
   private final OneDimensionalBounds[] bounds;
   private final double[] midPoint;
   private HyperCubeLeaf<T> leaf = null;
   private boolean hasChildren = false;
   private final HyperCubeNode<T, D>[] children;
   protected final HyperCubeTreeListener<T,D> listener;
   protected final String id;
   private D metaData;

   @SuppressWarnings("unchecked")

   // java cannot instantiate arrays of generic types. Silly java.
   protected HyperCubeNode(OneDimensionalBounds[] bounds, String id, HyperCubeTreeListener<T,D> listener)
   {
      this.bounds = bounds;
      this.dimensionality = bounds.length;
      this.midPoint = new double[dimensionality];
      for (int i = 0; i< dimensionality;i++)
      {
         midPoint[i]=bounds[i].midpoint();
      }
      this.id = id;
      this.childNumber = 1 << dimensionality;
      this.children = (HyperCubeNode<T, D>[]) new HyperCubeNode[childNumber];
      this.listener = listener;
      listener.nodeAdded(id, bounds, null);
   }

   public void setMetaData(D metaData)
   {
      this.metaData = metaData;
   }

   public D getMetaData()
   {
      return this.metaData;
   }

   public void clear()
   {
      if (this.hasChildren)
      {
         this.hasChildren = false;

         for (int i = 0; i < this.childNumber; i++)
         {
            if (children[i] != null)
            {
               children[i].clear();
               children[i] = null;
            }
            else
            {
               System.out.println("Trying to clear a null child");
               System.out.println("childNumber = " + childNumber);
               System.out.println("children.length = " + children.length);
            }
         }
      }

      this.leaf = null;
      this.listener.nodeRemoved(this.id);
   }

   public OneDimensionalBounds getBounds(int i)
   {
      return this.bounds[i];
   }

   public OneDimensionalBounds[] getBoundsCopy()
   {
      return this.bounds.clone();
   }

   public int getDimensionality()
   {
      return dimensionality;
   }

   public boolean hasChildren()
   {
      return this.hasChildren;
   }

   public int getChildNumber()
   {
      return this.childNumber;
   }

   public HyperCubeLeaf<T> getLeaf()
   {
      return this.leaf;
   }

   public void setLeaf(HyperCubeLeaf<T> leaf)
   {
      if (this.hasChildren)
         throw new RuntimeException("cannot have a leaf and children in node " + this);
      this.leaf = leaf;
      listener.nodeRemoved(this.id);
      listener.nodeAdded(this.id, this.bounds, this.leaf);
   }
   public void updateMetaDataListeners()
   {
      listener.metaDataUpdated(this.id, this.bounds, this.metaData);
   }

   public void split()
   {
      for (int i = 0; i < childNumber; i++)
      {
         OneDimensionalBounds[] subdividedBounds = this.subdivideBounds(this.toBooleanArray(i));
         children[i] = new HyperCubeNode<T, D>(subdividedBounds, this.id + "." + Integer.toHexString(i), listener);
      }

      this.hasChildren = true;

      if (this.leaf != null)
      {
         boolean[] sides = locatePoint(leaf.getLocation());
         int index = toIndex(sides);
         this.children[index].setLeaf(leaf);
         this.leaf = null;
      }
   }

   public String toString()
   {
      return toString(0);
   }


   public RecursableHyperTreeNode<T, D> getChildAtLocation(double[] location)
   {
      int index = toIndex(locatePoint(location));
      RecursableHyperTreeNode<T, D> hyperCubeNode = this.children[index];
      if (null == hyperCubeNode)
         throw new RuntimeException("child is null!");

      return hyperCubeNode;
   }

   public RecursableHyperTreeNode<T, D> getChild(int number)
   {
      return this.children[number];
   }


   protected List<HyperCubeLeaf<T>> gatherLeavesWithinBounds(OneDimensionalBounds[] gatherBounds)
   {
      List<HyperCubeLeaf<T>> ret = new ArrayList<HyperCubeLeaf<T>>();
      if (this.hasChildren)
      {
         for (int i = 0; i < childNumber; i++)
         {
            ret.addAll(children[i].gatherLeavesWithinBounds(gatherBounds));
         }
      }
      else
      {
         if (this.leaf != null)
         {
            ret.add(this.leaf);
         }
      }

      return ret;
   }

   protected OneDimensionalBounds[] getBounds()
   {
      return bounds;
   }


   boolean getHasChildren()
   {
      return hasChildren;
   }

   boolean[] locatePoint(double[] location)
   {
      boolean[] sides = new boolean[dimensionality];
      for (int i = 0; i < dimensionality; i++)
      {
         sides[i] = bounds[i].maxSide(location[i]);
      }

      return sides;
   }

   OneDimensionalBounds[] subdivideBounds(boolean[] sides)
   {
      OneDimensionalBounds[] subdividedBounds = new OneDimensionalBounds[dimensionality];
      for (int i = 0; i < dimensionality; i++)
      {
         subdividedBounds[i] = bounds[i].subdivide(sides[i]);
      }

      return subdividedBounds;
   }

   boolean[] toBooleanArray(int index)
   {
      boolean[] ret = new boolean[dimensionality];
      for (int i = 0; i < dimensionality; i++)
      {
         ret[i] = ((index & (1 << (dimensionality - i - 1))) != 0);
      }

      return ret;
   }

   int toIndex(boolean[] sides)
   {
      int index = 0;
      for (int i = 0; i < dimensionality; i++)
      {
         index = (index << 1) + (sides[i] ? 1 : 0);
      }

      return index;
   }

   private String toString(int tabs)
   {
      StringBuilder tabsBuilder = new StringBuilder(tabs);
      for (int i = 0; i < tabs; i++)
      {
         tabsBuilder.append("\t");
      }

      String tabString = tabsBuilder.toString();
      StringBuilder ret = new StringBuilder(100);

      ret.append(tabString);
      ret.append("hyper cube node of dimensionality ");
      ret.append(this.dimensionality);

      if (this.hasChildren)
      {
         ret.append(" with the following children\n");

         for (int i = 0; i < this.childNumber; i++)
         {
            ret.append(children[i].toString(tabs + 1));
         }
      }
      else
      {
         ret.append(" with no children ");

         if (null == this.leaf)
         {
            ret.append("and no value.");
         }
         else
         {
            ret.append("and value ");
            ret.append(this.leaf.getValue().toString());
            ret.append(".");
         }

         ret.append("\n");

         for (int i = 0; i < dimensionality; i++)
         {
            ret.append(tabString);
            ret.append("\tdimension ");
            ret.append(i);
            ret.append(": ");
            ret.append(bounds[i].toString());
            ret.append("\n");
         }
      }

      return ret.toString();
   }

   public static boolean withinBounds(OneDimensionalBounds[] bounds, double[] input)
   {
      for (int i = 0; i < bounds.length; i++)
      {
         if (!bounds[i].contains(input[i]))
            return false;
      }

      return true;
   }

   public double[] getMidpoint()
   {
      return this.midPoint;
   }
}
