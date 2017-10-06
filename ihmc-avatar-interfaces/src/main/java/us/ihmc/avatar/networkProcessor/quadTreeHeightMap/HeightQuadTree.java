package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

public class HeightQuadTree
{
   private HeightQuadTreeNode root;
   private float defaultHeight = Float.NaN;
   private float resolution = Float.NaN;
   private float sizeX = Float.NaN;
   private float sizeY = Float.NaN;

   public HeightQuadTree()
   {
   }

   public HeightQuadTreeNode getRoot()
   {
      return root;
   }

   public float getDefaultHeight()
   {
      return defaultHeight;
   }

   public float getResolution()
   {
      return resolution;
   }

   public float getSizeX()
   {
      return sizeX;
   }

   public float getSizeY()
   {
      return sizeY;
   }

   public void setRoot(HeightQuadTreeNode root)
   {
      this.root = root;
   }

   public void setDefaultHeight(float defaultHeight)
   {
      this.defaultHeight = defaultHeight;
   }

   public void setResolution(float resolution)
   {
      this.resolution = resolution;
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
