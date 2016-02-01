package us.ihmc.robotics.quadTree;

import javax.vecmath.Point3d;

public class QuadTreeForGroundPoint extends Point3d
{
   private static final long serialVersionUID = 8682882199962837728L;
   
   private boolean registered = false;
   private QuadTreeForGroundLeaf parent;

   public QuadTreeForGroundPoint(double x, double y, double z)
   {
      super(x, y, z);
   }

   public void setParent(QuadTreeForGroundLeaf parent)
   {
      this.parent = parent;
   }
   
   public boolean isRegistered()
   {
      return registered;
   }
   
   public void setRegistered(boolean registered)
   {
      this.registered = registered;
   }
   
   public QuadTreeForGroundLeaf getParent()
   {
      return parent;
   }

   public void removeFromParent()
   {
      if(parent != null)
      {
         parent.removePoint(this);
      }
   }

   @Override
   public int hashCode()
   {
      return super.hashCode(); // Ignore the existence of the parent field. Only 2 equal points with different parent will exist at a given time anyway.
   }

   // Make sure to override, otherwise it uses Tuple3d.equals
   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      else
         return false;
   }

}
