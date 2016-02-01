package us.ihmc.simulationconstructionset.util.ground.steppingStones;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.util.ground.GroundProfileFromHeightMap;
import us.ihmc.robotics.geometry.BoundingBox3d;

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

   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double height = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return height;
   }
   
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

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      normal.x = 0.0;
      normal.y = 0.0;
      normal.z = 1.0;
   }
   
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

}
