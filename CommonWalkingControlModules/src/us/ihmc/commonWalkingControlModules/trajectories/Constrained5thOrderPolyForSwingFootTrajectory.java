package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


/**
 * This class creates a cubic function that connects a point at height z0 at time = t0 to a point at height zf at time = tf.
 * The height of the function will max out at hmax at the midpoint.
 *
 */

public class Constrained5thOrderPolyForSwingFootTrajectory
{
   private final YoVariableRegistry registry;
   
   private final DoubleYoVariable X0, Xf, Vf, T0, Tf, HMAX;
   private double C0, C1, C2, C3, C4, C5;

   public double pos, vel, acc;

   public Constrained5thOrderPolyForSwingFootTrajectory(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(this.registry);
      
      X0 = new DoubleYoVariable(name + "_x0", registry);
      T0 = new DoubleYoVariable(name + "_t0", registry);
      HMAX = new DoubleYoVariable(name + "_hmax", registry);

      Xf = new DoubleYoVariable(name + "_xf", registry);
      Vf = new DoubleYoVariable(name + "_vf", registry);
      Tf = new DoubleYoVariable(name + "_tf", registry);
   }


   public double getStartTime()
   {
      return T0.getDoubleValue();
   }

   public double getFinalTime()
   {
      return Tf.getDoubleValue();
   }

   public double getPosition()
   {
      return pos;
   }

   public double getVelocity()
   {
      return vel;
   }

   public double getAcceleration()
   {
      return acc;
   }


   public void setParams(double X0, double HMAX, double Xf, double Vf, double T0, double Tf)
   {
      this.HMAX.set(HMAX);
      this.X0.set(X0);
      this.Vf.set(Vf);
      this.Xf.set(Xf);
      this.T0.set(T0);
      this.Tf.set(Tf);
      
      computeConstants();
   }

   private void computeConstants()
   {
      double t0 = T0.getDoubleValue();
      double tf = Tf.getDoubleValue();
      double x0 = X0.getDoubleValue();
      double xf = Xf.getDoubleValue();
      double vf = Vf.getDoubleValue();
      double hmax = HMAX.getDoubleValue();
      
      double factor = 1/(Math.pow(t0-tf, 5));
      
      C0 = factor*(16*hmax*Math.pow(t0,2)*(t0 - tf)*Math.pow(tf,2) + Math.pow((t0 + 
            tf),2)*(tf*(Math.pow(t0,2)*(-t0 + tf)*vf + (7*t0 - tf)*tf*x0) + 
                  Math.pow(t0,2)*(t0 - 7*tf)*xf));
      
      C1 = factor*(t0*(t0 + tf)*(32*hmax*tf*(-t0 + tf) + Math.pow(t0,3)*vf + 6*Math.pow(t0,2)*tf*vf - 
            2*Math.pow(tf,2)*(tf*vf + 23*x0 - 7*xf) + t0*tf*(-5*tf*vf - 14*x0 + 46*xf)));
      
      C2 = factor*(16*hmax*(t0 - tf)*(Math.pow(t0,2) + 4*t0*tf + Math.pow(tf,2)) - 6*Math.pow(t0,4)*vf + 
            t0*Math.pow(tf,2)*(11*tf*vf + 129*x0 - 81*xf) + 
            3*Math.pow(t0,2)*tf*(3*tf*vf + 27*x0 - 43*xf) + 
            Math.pow(t0,3)*(-15*tf*vf + 7*x0 - 23*xf) + Math.pow(tf,3)*(tf*vf + 23*x0 - 7*xf));
      
      C3 = factor*(32*hmax*(-Math.pow(t0,2) + Math.pow(tf,2)) + 13*Math.pow(t0,3)*vf + 
            Math.pow(tf,2)*(-5*tf*vf - 66*x0 + 34*xf) + Math.pow(t0,2)*(9*tf*vf - 34*x0 + 66*xf) + 
            t0*tf*(-17*tf*vf - 140*x0 + 140*xf));
      
      C4 = factor*(4*(4*hmax*(t0 - tf) - 3*Math.pow(t0,2)*vf + t0*(tf*vf + 13*x0 - 17*xf) + 
            tf*(2*tf*vf + 17*x0 - 13*xf)));
      
      C5 = factor*(4*(t0*vf - tf*vf - 6*x0 + 6*xf));
   }

   public void computeTrajectory(double time)
   {
      pos = C0 + C1*time + C2*Math.pow(time,2) + C3*Math.pow(time,3) + C4*Math.pow(time,4) + C5*Math.pow(time,5);
      vel = C1 + 2*C2*time + 3*C3*Math.pow(time,2) + 4*C4*Math.pow(time,3) + 5*C5*Math.pow(time,4);
      acc = 2*C2 + 6*C3*time + 12*C4*Math.pow(time,2) + 20*C5*Math.pow(time,3);
   }
}
