package us.ihmc.atlas.velocityControlEvaluation;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class VelocityControlEvaluationRobot extends Robot
{
   private static final long serialVersionUID = -2717723504360444704L;
   public static final double MASS = 10.0;
   public static final double STICTION = 60.0; 
   
   private final SliderJoint rootJoint;

   public VelocityControlEvaluationRobot()
   {
      super("VelocityControlEvaluationRobot");
      
      rootJoint = new SliderJoint("x", new Vector3d(), this, Axis.X);
      rootJoint.setStiction(STICTION);
      
      Link pointMass = new Link("pointMass");
      pointMass.setMass(MASS);
      pointMass.setMomentOfInertia(0.0, 0.0, 0.0);
      pointMass.setComOffset(new Vector3d());
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(0.03);
      pointMass.setLinkGraphics(linkGraphics);
      
      ExternalForcePoint externalForce = new ExternalForcePoint("externalForce", this.getRobotsYoVariableRegistry());
      rootJoint.addExternalForcePoint(externalForce);
      
      rootJoint.setLink(pointMass);
      this.addRootJoint(rootJoint);
   }

   public void setTau(double tau)
   {
      rootJoint.setTau(tau);
   }

   public double getX()
   {
      return rootJoint.getQYoVariable().getDoubleValue();
   }
   
   public double getXDot()
   {
      return rootJoint.getQDYoVariable().getDoubleValue();
   }

  
}
