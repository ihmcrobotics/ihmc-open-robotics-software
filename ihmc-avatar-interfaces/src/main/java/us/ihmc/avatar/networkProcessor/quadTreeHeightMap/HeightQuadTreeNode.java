package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.Vector2D32;

public class HeightQuadTreeNode
{
   private float height;
   private Point2D32 center = new Point2D32();
   private Vector2D32 size = new Vector2D32();
   private HeightQuadTreeNode[] children;

   public HeightQuadTreeNode()
   {
   }

   public boolean hasChildrenArray()
   {
      return children != null;
   }

   public int getNumberOfChildren()
   {
      if (children == null)
         return 0;
      int numberOfChildren = 0;

      for (int childIndex = 0; childIndex < 4; childIndex++)
         numberOfChildren += children[childIndex] == null ? 0 : 1;

      return numberOfChildren;
   }

   public HeightQuadTreeNode getChild(int childIndex)
   {
      return children == null ? null : children[childIndex];
   }

   public void assignChildrenArray()
   {
      children = new HeightQuadTreeNode[4];
   }

   public void setChild(int childIndex, HeightQuadTreeNode child)
   {
      children[childIndex] = child;
   }

   public float getHeight()
   {
      return height;
   }

   public Point2D32 getCenter()
   {
      return center;
   }

   public float getCenterX()
   {
      return center.getX32();
   }

   public float getCenterY()
   {
      return center.getY32();
   }

   public Vector2D32 getSize()
   {
      return size;
   }

   public float getSizeX()
   {
      return size.getX32();
   }

   public float getSizeY()
   {
      return size.getY32();
   }

   public void setHeight(float height)
   {
      this.height = height;
   }

   public void setCenterX(float centerX)
   {
      this.center.setX(centerX);
   }

   public void setCenterY(float centerY)
   {
      this.center.setY(centerY);
   }

   public void setSizeX(float sizeX)
   {
      this.size.setX(sizeX);
   }

   public void setSizeY(float sizeY)
   {
      this.size.setY(sizeY);
   }
}
