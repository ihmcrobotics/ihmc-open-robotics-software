package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.jMonkeyEngineToolkit.CombinedHeightMapWithNormals;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class CombinedGroundProfile3D implements GroundProfile3D
{
   private GroundProfile3D[] groundProfiles;
   private final BoundingBox3d boundingBox;
   private final CombinedHeightMapWithNormals heightMap;

   public CombinedGroundProfile3D(GroundProfile3D[] groundProfiles)
   {
      this.groundProfiles = groundProfiles;
      this.heightMap = new CombinedHeightMapWithNormals();

      BoundingBox3d boundingBox = null;
      for (GroundProfile3D groundProfile : groundProfiles)
      {
         if (boundingBox == null) boundingBox = groundProfile.getBoundingBox();
         else
         {
            boundingBox = BoundingBox3d.union(boundingBox, groundProfile.getBoundingBox());
         }
         
         HeightMapWithNormals heightMapIfAvailable = groundProfile.getHeightMapIfAvailable();
         if (heightMapIfAvailable != null)
            this.heightMap.addHeightMap(heightMapIfAvailable);
      }
      
      this.boundingBox = boundingBox;
   }

   public CombinedGroundProfile3D(ArrayList<GroundProfile3D> groundProfilesArrayList)
   {
      this.groundProfiles = new GroundProfile3D[groundProfilesArrayList.size()];
      this.heightMap = new CombinedHeightMapWithNormals();

      BoundingBox3d boundingBox = null;
      for (GroundProfile3D groundProfile : groundProfiles)
      {
         if (boundingBox == null) boundingBox = groundProfile.getBoundingBox();
         else
         {
            boundingBox = BoundingBox3d.union(boundingBox, groundProfile.getBoundingBox());
         }
         
         HeightMapWithNormals heightMapIfAvailable = groundProfile.getHeightMapIfAvailable();
         if (heightMapIfAvailable != null)
            this.heightMap.addHeightMap(heightMapIfAvailable);
      }
      this.boundingBox = boundingBox;

      groundProfilesArrayList.toArray(groundProfiles);
   }
   
   public GroundProfile3D[] getGroundProfiles()
   {
      return groundProfiles;
   }

   private final Point3d tempPointToCheck = new Point3d();

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      double smallestDistance = Double.MAX_VALUE;
      Point3d localIntersection = new Point3d();
      Vector3d localNormal = new Vector3d();
      boolean isInside = false;

      tempPointToCheck.set(x, y, z);

      // Pre-set some values, in case no object is close.
      intersectionToPack.set(x, y, 0.0);
      normalToPack.set(0.0, 0.0, 1.0);

      for (GroundProfile3D groundProfile : groundProfiles)
      {
         if (groundProfile.isClose(x, y, z))
         {
            boolean localIsInside = groundProfile.checkIfInside(x, y, z, localIntersection, localNormal);

            if (localIsInside && (tempPointToCheck.distance(localIntersection) < smallestDistance))
            {
               smallestDistance = tempPointToCheck.distance(localIntersection);
               intersectionToPack.set(localIntersection);
               normalToPack.set(localNormal);
               isInside = true;
            }
         }
      }

      // Reset pointToCheck for rewindability tests
      tempPointToCheck.set(0.0, 0.0, 0.0);

      return isInside;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if (boundingBox == null) return false;
      
      return boundingBox.isInside(x, y, z);
   }
   
   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return heightMap;
   }
}

