package us.ihmc.simulationconstructionset.util.ground.steppingStones;

import java.util.ArrayList;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.simulationconstructionset.util.ground.GroundProfileFromHeightMap;

public class SteppingStonesGroundProfile extends GroundProfileFromHeightMap
{
   private final SteppingStones steppingStones;

   private final BoundingBox3d boundingBox = new BoundingBox3d(-10.0, -10.0, -10.0, 10.0, 10.0, 10.0);
   
   public SteppingStonesGroundProfile(SteppingStones steppingStones)
   {
      this.steppingStones = steppingStones;
   }

   public SteppingStones getSteppingStones()
   {
      return steppingStones;
   }

   private final ArrayList<SteppingStone> tempStonesIntersectingLocation = new ArrayList<SteppingStone>();

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double height = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return height;
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      tempStonesIntersectingLocation.clear();
      steppingStones.getStonesIntersectingLocation(x, y, tempStonesIntersectingLocation);

      if (tempStonesIntersectingLocation.isEmpty())
         return Double.NEGATIVE_INFINITY;
      if (tempStonesIntersectingLocation.size() == 1)
         return tempStonesIntersectingLocation.get(0).getHeight();

      double maxHeight = Double.NEGATIVE_INFINITY;

      for (SteppingStone steppingStone : tempStonesIntersectingLocation)
      {
         if (steppingStone.getHeight() > maxHeight)
            maxHeight = steppingStone.getHeight();
      }

      return maxHeight;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);
   }
   
   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

}
