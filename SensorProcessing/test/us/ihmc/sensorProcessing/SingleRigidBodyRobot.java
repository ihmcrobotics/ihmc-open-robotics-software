package us.ihmc.sensorProcessing;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;


public class SingleRigidBodyRobot extends Robot
{
   private static final long serialVersionUID = -7671864179791904256L;
	
   /* L1 and L2 are the link lengths, M1 and M2 are the link masses, and R1 and R2 are the radii of the links, 
    * Iyy1 and Iyy2 are the moments of inertia of the links. The moments of inertia are defined about the COM
    * for each link.
    */
   public static final double
      L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, R1 = 0.1, R2 = 0.05, Iyy1 = 1.0, Iyy2 = 0.33;
   
   private final FloatingJoint bodyJoint;
   private final ExternalForcePoint forcePoint;

   
   public SingleRigidBodyRobot()
   {
      super("DoublePendulum"); // create and instance of Robot
      
      this.setGravity(0.0);

      // Create joints and assign links. Pin joints have a single axis of rotation.
      bodyJoint = new FloatingJoint("joint1", new Vector3d(0.0, 0.0, 0.0), this);
      Link link1 = link1();
      bodyJoint.setLink(link1); // associate link1 with the joint pin1
      this.addRootJoint(bodyJoint);
      
      forcePoint = new ExternalForcePoint("forcePoint", new Vector3d(), this.getRobotsYoVariableRegistry());
      bodyJoint.addExternalForcePoint(forcePoint);
   }


   /**
    * Create the first link for the DoublePendulumRobot.
    */
   private Link link1()
   {
      Link ret = new Link("link1");
      ret.setMass(M1);
      ret.setComOffset(0.0, 0.0, L1/2.0);
      //ret.setComOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(Iyy1, Iyy1, Iyy1/10.0);

      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      //linkGraphics.translate(new Vector3d(0.0, 0.0, -L1 / 2.0));
      linkGraphics.addCylinder(L1, R1);
      
      // associate the linkGraphics object with the link object
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
   
   public void getTransformToWorld(RigidBodyTransform transformToPack)
   {
      bodyJoint.getTransformToWorld(transformToPack);
   }

   public FrameVector getBodyVelocity()
   {
      return new FrameVector(ReferenceFrame.getWorldFrame(), bodyJoint.getQdx().getDoubleValue(), bodyJoint.getQdy().getDoubleValue(), bodyJoint.getQdz().getDoubleValue());
   }

   public FrameVector getBodyAcceleration()
   {
      return new FrameVector(ReferenceFrame.getWorldFrame(), bodyJoint.getQddx().getDoubleValue(), bodyJoint.getQddy().getDoubleValue(), bodyJoint.getQddz().getDoubleValue());
   }
   
   public FrameVector getBodyAngularVelocityInBodyFrame(ReferenceFrame bodyFrame)
   {
      return new FrameVector(ReferenceFrame.getWorldFrame(), bodyJoint.getAngularVelocityInBody());
   }

   // Get Body Angular Acceleration
   public FrameVector getBodyAngularAccelerationInBodyFrame(ReferenceFrame bodyFrame)
   {
      return new FrameVector(ReferenceFrame.getWorldFrame(), bodyJoint.getAngularAccelerationInBody());
   }


   public void setAngularVelocity(Vector3d angularVelocity)
   {
      bodyJoint.setAngularVelocityInBody(angularVelocity);
   }


   public void setLinearVelocity(Vector3d linearVelocity)
   {
      bodyJoint.setVelocity(linearVelocity);
   }


   public void setPosition(Vector3d bodyPosition)
   {
      bodyJoint.setPosition(bodyPosition);
   }


   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      bodyJoint.setYawPitchRoll(yaw, pitch, roll);
   }


   public void setExternalForce(Vector3d force)
   {
      forcePoint.setForce(force);
   }
}