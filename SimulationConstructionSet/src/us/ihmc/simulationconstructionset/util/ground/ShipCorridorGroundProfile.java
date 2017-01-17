package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox3d;


public class ShipCorridorGroundProfile implements GroundProfile3D, HeightMapWithNormals
{
	private final BoundingBox3d boundingBox;

	private final double zGroundPosition;
	private final double yMaxCorridor;
	private final double yMinCorridor;
	private final double maxWallHeight;
	private final double wallInclination;
	private final double wallEndY=2.0; 

	public ShipCorridorGroundProfile(double xMax, double xMin, double yMax, double yMin, double yMaxCorridor, double yMinCorridor, double zGroundPosition, double maxWallHeight)
	{
		this.yMaxCorridor = yMaxCorridor;
		this.yMinCorridor = yMinCorridor;
		this.zGroundPosition = zGroundPosition;
		this.maxWallHeight = maxWallHeight;
		this.wallInclination = Math.toRadians( 2.0 );//Default inclination of 2.0 degrees (roll)
		
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Double.POSITIVE_INFINITY;
      this.boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
	}

	public ShipCorridorGroundProfile(double xMax, double xMin, double yMax, double yMin, double yMaxCorridor, double yMinCorridor, double zGroundPosition, double maxWallHeight, double wallInclination)
	{
		this.yMaxCorridor = yMaxCorridor;
		this.yMinCorridor = yMinCorridor;
		this.zGroundPosition = zGroundPosition;
		this.maxWallHeight = maxWallHeight;
		this.wallInclination = wallInclination;
		
      double zMin = Double.NEGATIVE_INFINITY;
      double zMax = Double.POSITIVE_INFINITY;
      this.boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
	}

   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, heightAt, normalToPack);
      return heightAt;
   }
   
	public double heightAt(double x, double y, double z)
	{ 
		double height;

		//X-Axis is inverted
		// Left Wall
		if ((y > yMaxCorridor) && (y < wallEndY))
		{
			height = (y - yMaxCorridor) / Math.tan( wallInclination );
			if(height >= maxWallHeight)
			{
				height = maxWallHeight;
			}
		}
		//Corridor Ground
		else if ((y < yMaxCorridor) && (y > yMinCorridor))
		{
			height = zGroundPosition;
		}
		//Right Wall
		else if ((y > -wallEndY) && (y < yMinCorridor))
		{
			height = (-y - yMaxCorridor) / Math.tan( wallInclination );
			if(height >= maxWallHeight)
			{
				height = maxWallHeight;
			}
		} 
		//Rest of the surrounding ground
		else
		{
			height = maxWallHeight;
		}
		return height;
	}

	public boolean isClose(double x, double y, double z)
	{
		return true;
	}

	public void closestIntersectionTo(double x, double y, double z, Point3d intersection)
	{
		/* there are three areas (triangles):
		 * 1. left wall (positive y)
		 * 2. center
		 * 3. right wall (negative y)
		 * 
		 * For the closest intersection with the wall, first the y distance to the wall is computed.
		 * The y and z values of this vector are then added to the y and z value of the point.
		 */
		
		
		double separationAngleFromVertical=Math.PI/2-0.5*(0.5*Math.PI+ wallInclination );

		intersection.setY(y);
		intersection.setZ(heightAt(x,y,z)); // should be zero
		
		// left wall (positive y)
		if ((y>0)&&(y>yMaxCorridor-z*Math.tan(separationAngleFromVertical)))
		{
			double r=Math.cos( wallInclination )*(getAbsWallY(z)-y);
			intersection.setZ(z-r*Math.sin( wallInclination ));
			intersection.setY(y+r*Math.cos( wallInclination ));
		}
		
		// center
		if (Math.abs(y)<z*Math.tan(separationAngleFromVertical))
		{
			// already defined	
		}

		// right wall (negative y)		
		if ((y<0)&&(y<yMinCorridor+z*Math.tan(separationAngleFromVertical)))
		{
			double r=Math.cos( wallInclination )*(getAbsWallY(z)+y);
			intersection.setZ(z-r*Math.sin( wallInclination ));
			intersection.setY(y-r*Math.cos( wallInclination ));
		}
		
		intersection.setX(x);
	}

	/**
	 * returns absolute y position of the wall at z position
	 * if z is not in the wall, returns -1.0
	 */
	public double getAbsWallY(double z)
	{
		double y=-1.0;
		
		if ((z>0)&&(z<maxWallHeight))
		{
			y=yMaxCorridor+ z*Math.tan( wallInclination );
		}

		return y;
	}
	
	public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      double height;
      height = this.heightAt(x, y, z);
      
      //X-Axis is inverted
      // Left Wall
      if ((y > yMaxCorridor) && (y < wallEndY))
      {
         
         normal.setY(-1.0);
         normal.setZ(Math.tan( wallInclination ));
         
         if (height >= maxWallHeight)
         {
            normal.setY(0.0);
            normal.setZ(1.0);
         }
         
      }

      //Corridor Ground
      else if ((y < yMaxCorridor) && (y > yMinCorridor))
      {
         normal.setY(0.0);
         normal.setZ(1.0);
      }
      //Right Wall
      else if ((y > -wallEndY) && (y < yMinCorridor))
      {
         normal.setY(1.0);
         normal.setZ(Math.tan( wallInclination ));
         if (height >= maxWallHeight)
         {
            normal.setY(0.0);
            normal.setZ(1.0);
         }
      }
      //Rest of the surrounding ground
      else
      {
         normal.setY(0.0);
         normal.setZ(1.0);
      }
      normal.setX(0.0);
      normal.normalize();
   }
	
   private Vector3d tempVector = new Vector3d();
   private Vector3d tempVectorTwo = new Vector3d();
	
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      closestIntersectionTo(x, y, z, intersectionToPack);
      surfaceNormalAt(x, y, z, normalToPack);
      
      // TODO: Do the checks in the logic.
      // TODO: Replace all this with TerrainObject3Ds.
      
      tempVector.set(intersectionToPack);
      tempVectorTwo.set(x, y, z);
      
      tempVector.sub(tempVectorTwo);
      double dotProduct = tempVector.dot(normalToPack);
      
      return (dotProduct < 0.0);
   }
   
	public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersection, Vector3d normal)
	{
		closestIntersectionTo(x,y,z,intersection);
		surfaceNormalAt(x, y, z, normal);
	}

	public double getYMaxCorridor()
	{
		return yMaxCorridor;
	}

	public double getYMinCorridor()
	{
		return yMinCorridor;
	}

	public double getWallInclination()
	{
	   return wallInclination;
	}
	
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}
