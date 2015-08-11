package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Ramp3d implements Shape3d
{
   private static final double DEFAULT_EPSILON = 1e-7;
   protected RigidBodyTransform transform = new RigidBodyTransform();
   private RigidBodyTransform inverseTransform = new RigidBodyTransform();
   private RigidBodyTransform transformFromRampToAngledFrame = new RigidBodyTransform();
   private RigidBodyTransform transformFromAngledFrameToRamp = new RigidBodyTransform();

   // for these, the x-axis lies in the ramp plane, and is parallel to the ramp sides
   private double width;
   private double length;
   private double height;
   private double rampLength;
   private double angleOfRampIncline;

   private Vector3d surfaceNormal;

   private final RigidBodyTransform temporaryTransform = new RigidBodyTransform();
   private final Point3d temporaryPoint = new Point3d();

   public Ramp3d(double width, double length, double height)
   {
      this(new RigidBodyTransform(), width, length, height);
   }

   public Ramp3d(RigidBodyTransform transform, double width, double length, double height)
   {
      this.transform.set(transform);
      this.width = width;
      this.length = length;
      this.height = height;
      initializeRamp(transform, length, height);
   }

   public Ramp3d(Ramp3d ramp3d)
   {
      this(ramp3d.transform, ramp3d.width, ramp3d.length, ramp3d.height);
   }

   public void set(Ramp3d ramp3d)
   {
      this.transform.set(ramp3d.transform);
      this.width = ramp3d.width;
      this.length = ramp3d.length;
      this.height = ramp3d.height;
      initializeRamp(this.transform, this.length, this.height);
   }

   private void initializeRamp(RigidBodyTransform transform, double length, double height)
   {
      rampLength = Math.sqrt(height * height + length * length);
      inverseTransform.invert(transform);

      // additional transform where x-axis is plane of the ramp and y-axis is
      // still perpendicular to it's length
      angleOfRampIncline = Math.atan(height / length);
      transformFromAngledFrameToRamp.rotY(-angleOfRampIncline);

      temporaryTransform.multiply(transform, transformFromAngledFrameToRamp);
      surfaceNormal = new Vector3d(0.0, 0.0, 1.0);
      temporaryTransform.transform(surfaceNormal);

      transformFromRampToAngledFrame.invert(transformFromAngledFrameToRamp);
   }

   public double getWidth()
   {
      return width;
   }

   public void setWidth(double width)
   {
      this.width = width;
   }

   public double getHeight()
   {
      return height;
   }

   public void setHeight(double height)
   {
      this.height = height;
   }

   public double getLength()
   {
      return length;
   }

   public void setLength(double length)
   {
      this.length = length;
   }
   

   public double getRampLength()
   {
      return this.rampLength;
   }

   public RigidBodyTransform getTransform()
   {
      return transform;
   }

   public void setTransform(RigidBodyTransform newTransform)
   {
      transform.set(newTransform);
   }

   public Vector3d getSurfaceNormal()
   {
      return surfaceNormal;
   }

   public double getRampIncline()
   {
      return angleOfRampIncline;
   }

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   public void applyTransform(RigidBodyTransform transform)
   {
      tempTransform.set(transform);
      tempTransform.multiply(this.transform);
      this.transform.set(tempTransform);
      inverseTransform.set(this.transform);
      inverseTransform.invert();
   }

   public double distance(Point3d point)
   {
      temporaryPoint.set(point);
      orthogonalProjection(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   public boolean checkIfInside(Point3d pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      boolean isInsideOrOnSurface = isInsideOrOnSurface(pointInWorldToCheck);
      if (isInsideOrOnSurface)
      {
         closestPointToPack.set(pointInWorldToCheck);
         surfaceNormalAtForInsideOrOnSurface(normalToPack, pointInWorldToCheck);     
         return true;
      }
      
      closestPointToPack.set(pointInWorldToCheck);
      orthogonalProjection(closestPointToPack);
      
      surfaceNormalAtForProjectedToSurface(normalToPack, closestPointToPack);     
      return false;
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, DEFAULT_EPSILON);
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck, double epsilon)
   {
      temporaryPoint.set(pointToCheck);
      inverseTransform.transform(temporaryPoint);
      transformFromRampToAngledFrame.transform(temporaryPoint);

      boolean isInsideOrOnSurface = ((temporaryPoint.z <= epsilon) && (Math.abs(temporaryPoint.y) < 0.5 * width + epsilon) && (temporaryPoint.x >= -epsilon)
                                     && (temporaryPoint.x <= rampLength + epsilon));

      transformFromAngledFrameToRamp.transform(temporaryPoint);

      isInsideOrOnSurface = isInsideOrOnSurface && (temporaryPoint.x <= epsilon + length) && (temporaryPoint.x >= -epsilon)
                            && (temporaryPoint.z <= epsilon + height) && (temporaryPoint.z >= -epsilon);

      return isInsideOrOnSurface;
   }

   private boolean projectIntoXYRectangle(Point3d pointToCheckAndPack, double lengthToUse)
   {
      boolean projected = false;
      
      if (pointToCheckAndPack.getX() < 0.0) 
      {
         pointToCheckAndPack.setX(0.0);
         projected = true;
      }
      else if (pointToCheckAndPack.getX() > lengthToUse) 
      {
         pointToCheckAndPack.setX(lengthToUse);
         projected = true;
      }

      if (pointToCheckAndPack.getY() > width/2.0) 
      {
         pointToCheckAndPack.setY(width/2.0);
         projected = true;
      }
      else if (pointToCheckAndPack.getY() < -width/2.0) 
      {
         pointToCheckAndPack.setY(-width/2.0);
         projected = true;
      }
      
      return projected;
   }
   
   
   public void orthogonalProjection(Point3d pointToCheckAndPack)
   {
      inverseTransform.transform(pointToCheckAndPack);
            
      boolean aboveRamp = pointToCheckAndPack.getZ() > this.height;
      
      // If below, then project to bottom and return.
      if (pointToCheckAndPack.getZ() < 0.0) 
      {
         pointToCheckAndPack.setZ(0.0);
         projectIntoXYRectangle(pointToCheckAndPack, this.length);
         transform.transform(pointToCheckAndPack);
         return;
      }
      
      transformFromRampToAngledFrame.transform(pointToCheckAndPack);

      // If above the ramp, then project to ramp and return.
      if (aboveRamp || pointToCheckAndPack.getZ() > 0.0)
      {
         pointToCheckAndPack.setZ(0.0);
         projectIntoXYRectangle(pointToCheckAndPack, this.rampLength);
         transformFromAngledFrameToRamp.transform(pointToCheckAndPack);
         transform.transform(pointToCheckAndPack);
         return;
      }
      
      transformFromAngledFrameToRamp.transform(pointToCheckAndPack);
      if (projectIntoXYRectangle(pointToCheckAndPack, this.length)) return;
 
      // Inside. Just project up to the surface
      transformFromRampToAngledFrame.transform(pointToCheckAndPack);
      pointToCheckAndPack.setZ(0.0);

      transformFromAngledFrameToRamp.transform(pointToCheckAndPack);
      transform.transform(pointToCheckAndPack);
   }

   // point must be in angled frame
   private boolean canBeProjectedDirectlyOntoFaceOfRamp(Point3d pointTransformedOntoRampCoordinates)
   {
      double delta = 0.002;
      
      boolean insideWidth = (pointTransformedOntoRampCoordinates.y <= 0.5 * width - delta) && (pointTransformedOntoRampCoordinates.y >= -0.5 * width + delta);
      boolean insideLength = (pointTransformedOntoRampCoordinates.x >= delta) && (pointTransformedOntoRampCoordinates.x <= rampLength - delta);
      boolean onOrAboveSurface = pointTransformedOntoRampCoordinates.getZ() > -1e-3;
      return (insideWidth && insideLength && onOrAboveSurface);
   }
   
   private boolean canBeProjectedDirectlyOntoBottomOfRamp(Point3d pointTransformedOntoRampCoordinates)
   {
      boolean insideWidth = Math.abs(pointTransformedOntoRampCoordinates.y) <= 0.5 * width;
      boolean insideLength = (pointTransformedOntoRampCoordinates.x >= 0.0) && (pointTransformedOntoRampCoordinates.x <= length);
      boolean onSurface = Math.abs(pointTransformedOntoRampCoordinates.getZ()) < 1e-3;
      return (insideWidth && insideLength && onSurface);
   }
   
   private boolean isPointOnFaceOfRampAlsoOnTheEdges(Point3d point)
   {
      return (Math.abs(point.getX()) < 1e-3) || (Math.abs(point.getX() - rampLength) < 1e-3) || (Math.abs(point.getY() - 0.5 * width) < 1e-3) || (Math.abs(point.getY() + 0.5 * width) < 1e-3);
   }
   

   private boolean isPointOnBottomOfRampAlsoOnTheEdges(Point3d point)
   {
      return (Math.abs(point.getX()) < 1e-3) || (Math.abs(point.getX() - length) < 1e-3) || (Math.abs(point.getY() - 0.5 * width) < 1e-3) || (Math.abs(point.getY() + 0.5 * width) < 1e-3);
   }

   

   private void surfaceNormalAtForProjectedToSurface(Vector3d normalToPack, Point3d pointToCheck)
   {
      surfaceNormalAtForInsideOrOnSurface(normalToPack, pointToCheck);
   }

   private void surfaceNormalAtForInsideOrOnSurface(Vector3d normalToPack, Point3d pointToCheck)
   {
      temporaryPoint.set(pointToCheck);
      inverseTransform.transform(temporaryPoint);
      transformFromRampToAngledFrame.transform(temporaryPoint);

      if (canBeProjectedDirectlyOntoFaceOfRamp(temporaryPoint))
      {
         if (isPointOnFaceOfRampAlsoOnTheEdges(temporaryPoint))
         {
            makeNormalFromCenterOfRampToPointToCheck(normalToPack, pointToCheck);
            return;
         }
         
         normalToPack.set(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
         return;
      }
      
      temporaryPoint.set(pointToCheck);
      inverseTransform.transform(temporaryPoint);
      
      if (canBeProjectedDirectlyOntoBottomOfRamp(temporaryPoint))
      {
         if (isPointOnBottomOfRampAlsoOnTheEdges(temporaryPoint))
         {
            makeNormalFromCenterOfRampToPointToCheck(normalToPack, pointToCheck);
            return;
         }
         
         normalToPack.set(0.0, 0.0, -1.0);
         transform.transform(normalToPack);
         return;
      }
      else
      {
         temporaryPoint.set(pointToCheck);
         orthogonalProjection(temporaryPoint);
         temporaryPoint.sub(pointToCheck, temporaryPoint);
         normalToPack.set(temporaryPoint);
         normalToPack.normalize();
      } 
   }
   

   private void makeNormalFromCenterOfRampToPointToCheck(Vector3d normalToPack, Point3d pointToCheck)
   {
      normalToPack.set(pointToCheck);
      inverseTransform.transform(normalToPack);
      
      normalToPack.sub(new Vector3d(length/2.0, 0.0, height/4.0));
      
      transform.transform(normalToPack);
      normalToPack.normalize();
   }
 

   @Override
   public String toString()
   {
      return "width = " + width + ", length = " + length + ", height = " + height + ", + transform = " + transform.toString(8) + "\n";
   }

   public void transformFromAngledToWorldFrame(Point3d pointToTransform)
   {
//      inverseTransform.transform(pointToTransform);
//      transformFromRampToAngledFrame.transform(pointToTransform);
      
      transformFromAngledFrameToRamp.transform(pointToTransform);
      transform.transform(pointToTransform);
   }


}
