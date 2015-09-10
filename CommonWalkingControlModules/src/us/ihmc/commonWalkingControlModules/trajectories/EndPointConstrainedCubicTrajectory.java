package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class EndPointConstrainedCubicTrajectory {

	private final YoVariableRegistry registry;
	
	private double C0, C1, C2, C3;
	private final DoubleYoVariable t0,tf,X0,Xdot0,Xf,Xdotf;
	private final DoubleYoVariable pos, vel, acc;
	
	public EndPointConstrainedCubicTrajectory(String namePrefix, YoVariableRegistry parentRegistry)
	{
		this.registry = new YoVariableRegistry(getClass().getSimpleName());
		
		parentRegistry.addChild(this.registry);
		this.X0 = new DoubleYoVariable(namePrefix + "InitialPosition", this.registry);
		this.Xf = new DoubleYoVariable(namePrefix + "FinalDesiredPosition", this.registry);
		this.Xdot0 = new DoubleYoVariable(namePrefix + "InitialVelocity", this.registry);
		this.Xdotf = new DoubleYoVariable(namePrefix + "FinalDesiredVelocity", this.registry);
		this.t0 = new DoubleYoVariable(namePrefix + "InitialTime",this.registry);
		this.tf = new DoubleYoVariable(namePrefix + "FinalTime", this.registry);
		
		this.pos = new DoubleYoVariable(namePrefix + "DesiredPosition", this.registry);
		this.vel = new DoubleYoVariable(namePrefix + "DesiredVelocity", this.registry);
		this.acc = new DoubleYoVariable(namePrefix + "DesiredAcceleration", this.registry);
		
	}
	
	public void setParams(double t0, double tf, double X0, double Xdot0, double Xf, double Xdotf)
	{
		this.X0.set(X0);
		this.Xdot0.set(Xdot0);
		this.Xf.set(Xf);
		this.Xdotf.set(Xdotf);
		this.t0.set(t0);
		this.tf.set(tf);
		
		computeCoefficients();
	}
	
	public void computeTrajectory(double time)
	{
		pos.set(C0 + C1*time + C2*Math.pow(time,2) + C3*Math.pow(time,3));
		
		vel.set(C1 + 2*C2*time + 3*C3*Math.pow(time,2));
		
		acc.set(2*C2 + 6*C3*time);
	}
	
	public void computeCoefficients()
	{	
		double z0 = this.X0.getDoubleValue();
		double zdot0 = this.Xdot0.getDoubleValue();
		double zf = this.Xf.getDoubleValue();
		double zdotf = this.Xdotf.getDoubleValue();
		double tf = this.tf.getDoubleValue();
		double t0 = this.t0.getDoubleValue();
		
		double factor = 1/(Math.pow(t0 - tf,3));
		
		C0 = factor*(tf*(-tf*(-3*t0 + tf)*z0 + t0*tf*(-t0 + tf)*zdot0 + 
			    Math.pow(t0,2)*(-t0 + tf)*zdotf) + Math.pow(t0,2)*(t0 - 3*tf)*zf);
		
		C1 = factor*(-Math.pow(tf,3)*zdot0 + Math.pow(t0,3)*zdotf + Math.pow(t0,2)*tf*(2*zdot0 + zdotf) - 
				 t0*tf*(6*z0 + tf*zdot0 + 2*tf*zdotf - 6*zf));
		
		C2 = factor*(-Math.pow(t0,2)*(zdot0 + 2*zdotf) + t0*(3*z0 - tf*zdot0 + tf*zdotf - 3*zf) + 
				 tf*(3*z0 + 2*tf*zdot0 + tf*zdotf - 3*zf));
		
		C3 = factor*(-2*z0 + (t0 - tf)*(zdot0 + zdotf) + 2*zf);
	}
	
	public double getPosition()
	{
		return pos.getDoubleValue();
	}
	
	public double getVelocity()
	{
		return vel.getDoubleValue();
	}
	
	public double getAcceleration()
	{
		return acc.getDoubleValue();
	}
}
