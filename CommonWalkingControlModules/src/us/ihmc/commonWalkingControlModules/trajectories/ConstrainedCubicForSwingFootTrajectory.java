package us.ihmc.commonWalkingControlModules.trajectories;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * This class creates a cubic function that connects a point at height z0 at time = t0 to a point at height zf at time = tf.
 * The height of the function will max out at hmax at the midpoint.
 *
 */

public class ConstrainedCubicForSwingFootTrajectory
{
   private final YoVariableRegistry registry;
   
   private final DoubleYoVariable X0, Xf, T0, Tf, HMAX;
   private double C0, C1, C2, C3;

   public double pos, vel, acc;

   public ConstrainedCubicForSwingFootTrajectory(String name, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(getClass().getSimpleName());
      parentRegistry.addChild(this.registry);
      
      X0 = new DoubleYoVariable(name + "_x0", registry);
      T0 = new DoubleYoVariable(name + "_t0", registry);
      HMAX = new DoubleYoVariable(name + "_hmax", registry);

      Xf = new DoubleYoVariable(name + "_xf", registry);
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


   public void setParams(double X0, double HMAX, double Xf, double T0, double Tf)
   {
      this.HMAX.set(HMAX);
      this.X0.set(X0);
      this.Xf.set(Xf);
      this.T0.set(T0);
      this.Tf.set(Tf);
   }

   private void computeConstants()
   {
      double t0 = T0.getDoubleValue();
      double tf = Tf.getDoubleValue();
      double x0 = X0.getDoubleValue();
      double xf = Xf.getDoubleValue();
      double hmax = HMAX.getDoubleValue();
      
      double factor = 1/(Math.pow(t0-tf, 3));
      
      C0 = -factor*(4*hmax*Math.pow(t0,2)*tf - 4*hmax*t0*Math.pow(tf,2) + Math.pow(t0,2)*tf*x0 + 2*t0*Math.pow(tf,2)*x0 + 
            Math.pow(tf,3)*x0 - Math.pow(t0,3)*xf - 2*Math.pow(t0,2)*tf*xf - t0*Math.pow(tf,2)*xf);
      
      C1 = -factor*(-4*hmax*Math.pow(t0,2) + 4*hmax*Math.pow(tf,2) - Math.pow(t0,2)*x0 - 6*t0*tf*x0 - 5*Math.pow(tf,2)*x0 + 
            5*Math.pow(t0,2)*xf + 6*t0*tf*xf + Math.pow(tf,2)*xf);
      
      C2 = -factor*(4*(hmax*t0 - hmax*tf + t0*x0 + 2*tf*x0 - 2*t0*xf - tf*xf));
      
      C3 = factor*(4*(x0 - xf));
   }

   public void computeTrajectory(double time)
   {
      computeConstants();

      if (time < T0.getDoubleValue())
      {
         pos = X0.getDoubleValue();
         vel = 0;
         acc = 0;

         return;
      }
      else if (time > Tf.getDoubleValue())
      {
         pos = Xf.getDoubleValue();
         vel = 0;
         acc = 0;

         return;
      }
      else
      {
         pos = C0 + C1*time + C2*Math.pow(time,2) + C3*Math.pow(time,3);
         vel = C1 + 2*C2*time + 3*C3*Math.pow(time,2);
         acc = 2*C2 + 6*C3*time;
      }
   }
}
