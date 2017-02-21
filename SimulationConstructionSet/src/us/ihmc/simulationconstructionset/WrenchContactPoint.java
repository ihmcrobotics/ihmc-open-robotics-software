package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

// TODO delete if not used
@SuppressWarnings("serial")
public class WrenchContactPoint extends ExternalForcePoint
{

   //TODO: Change to a YoFramePoint. Make things private instead of public
   public DoubleYoVariable tdx, tdy, tdz;    // Touchdown position
   public DoubleYoVariable fs;    // Foot Switch TODO: BooleanYoVariable or EnumYoVariable

   Link link;
   RigidBodyTransform toWorld = new RigidBodyTransform();
   Vector3D v = new Vector3D();

   public WrenchContactPoint(String name, YoVariableRegistry registry, Link link )
   {
      super(name, new Vector3D(), registry);

      this.link = link;

      tdx = new DoubleYoVariable(name + "_tdX", "WrenchContactPoint x touchdown location", registry);
      tdy = new DoubleYoVariable(name + "_tdY", "WrenchContactPoint y touchdown location", registry);
      tdz = new DoubleYoVariable(name + "_tdZ", "WrenchContactPoint z touchdown location", registry);

      fs = new DoubleYoVariable(name + "_fs", "WrenchContactPoint foot switch", registry);
   }

   public void updateForce() {
      Vector3D force = link.getParentJoint().physics.Z_hat_i.top;
      fs.set(force.length());
      System.out.println("force on sensor: "+fs.getDoubleValue());
   }

   public void updateStatePostPhysicsComputation()
   {
      // compute location in world
      v.set(0, 0, 0);
      link.getParentJoint().getTransformToWorld(toWorld);
      toWorld.transform(v);

      tdx.set(v.getX());
      tdy.set(v.getY());
      tdz.set(v.getZ());
   }
   
   public boolean isInContact()
   {
      return (fs.getDoubleValue() > 0.5);
   }
   
   public void setIsInContact(boolean isInContact)
   {
      if (isInContact)
         fs.set(1.0);
      else
         fs.set(0.0);
   }

   public void getTouchdownLocation(Point3D touchdownLocationToPack)
   {
      touchdownLocationToPack.set(tdx.getDoubleValue(), tdy.getDoubleValue(), tdz.getDoubleValue());
   }

   public void setTouchdownLocation(Point3D touchdownLocation)
   {
      tdx.set(touchdownLocation.getX());
      tdy.set(touchdownLocation.getY());
      tdz.set(touchdownLocation.getZ());
   }
}
