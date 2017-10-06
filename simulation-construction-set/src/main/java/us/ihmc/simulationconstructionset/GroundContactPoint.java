package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class GroundContactPoint extends ExternalForcePoint
{
   private static final long serialVersionUID = 2334921180229856021L;
 
   private final YoFramePoint touchdownLocation;
   
   private final YoDouble fs;    // Foot Switch TODO: YoBoolean or YoEnum
   private final YoFrameVector surfaceNormal;

   private final YoBoolean slip;    // Whether or not it is slipping.
   private final YoInteger collisionCount;

   public GroundContactPoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }
   
   public GroundContactPoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public GroundContactPoint(String name, Vector3D offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }
   
   public GroundContactPoint(String name, Vector3D offset, YoVariableRegistry registry)
   {
      super(name, offset, registry);

      touchdownLocation = new YoFramePoint(name + "_td", "", ReferenceFrame.getWorldFrame(), registry);

      fs = new YoDouble(name + "_fs", "GroundContactPoint foot switch", registry);

      slip = new YoBoolean(name + "_slip", "GroundContactPoint slipping", registry);
      collisionCount = new YoInteger(name + "_coll", "GroundContactPoint colliding", registry);
      
      surfaceNormal = new YoFrameVector(name + "_n", "", ReferenceFrame.getWorldFrame(), registry);
   }

   public boolean isSlipping()
   {
      return slip.getBooleanValue();
   }
   
   public boolean isInContact()
   {
      return (fs.getDoubleValue() > 0.5);
   }
   
   public void disable()
   {
      fs.set(-1.0);
   }
   
   public boolean isDisabled()
   {
      return (fs.getDoubleValue() < -0.5);
   }

   public void setIsSlipping(boolean isSlipping)
   {
      slip.set(isSlipping);
   }
   
   public int getCollisionCount()
   {
      return collisionCount.getIntegerValue();
   }
   
   public void incrementCollisionCount()
   {
      this.collisionCount.increment();
   }
   
   public void setInContact()
   {
      fs.set(1.0);
   }

   public void setNotInContact()
   {
      fs.set(0.0);
   }
   
   public void setIsInContact(boolean isInContact)
   {
      if (isInContact) setInContact();
      else setNotInContact();
   }

   public void getTouchdownLocation(Point3D touchdownLocationToPack)
   {
      touchdownLocation.get(touchdownLocationToPack);
   }
   
   public YoFramePoint getYoTouchdownLocation()
   {
      return touchdownLocation;
   }
   
   public YoDouble getYoFootSwitch()
   {
      return fs;
   }

   public void setTouchdownLocation(Point3D touchdownLocation)
   {
      this.touchdownLocation.set(touchdownLocation);
   }

   public void setTouchdownToCurrentLocation()
   {
      this.touchdownLocation.set(this.getYoPosition());
   }

   
   public void getSurfaceNormal(Vector3D vectorToPack)
   {
      surfaceNormal.get(vectorToPack);
   }

   public void setSurfaceNormal(Vector3D surfaceNormal)
   {
      this.surfaceNormal.set(surfaceNormal);
   }

   public void setSurfaceNormal(double fx, double fy, double fz)
   {
      this.surfaceNormal.set(fx, fy, fz);
   }
   
   public YoFrameVector getYoSurfaceNormal()
   {
	   return this.surfaceNormal;
   }

}
