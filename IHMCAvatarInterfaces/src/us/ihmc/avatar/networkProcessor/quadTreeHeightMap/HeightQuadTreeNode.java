package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

public class HeightQuadTreeNode
{
   private float height;
   private float centerX;
   private float centerY;
   private float sizeX;
   private float sizeY;
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

   public float getCenterX()
   {
      return centerX;
   }

   public float getCenterY()
   {
      return centerY;
   }

   public float getSizeX()
   {
      return sizeX;
   }

   public float getSizeY()
   {
      return sizeY;
   }

   public void setHeight(float height)
   {
      this.height = height;
   }

   public void setCenterX(float centerX)
   {
      this.centerX = centerX;
   }

   public void setCenterY(float centerY)
   {
      this.centerY = centerY;
   }

   public void setSizeX(float sizeX)
   {
      this.sizeX = sizeX;
   }

   public void setSizeY(float sizeY)
   {
      this.sizeY = sizeY;
   }
}
