package us.ihmc.acsell.hardware.state;

import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.kinematics.fourbar.FourbarCalculator;
import us.ihmc.robotics.kinematics.fourbar.FourbarProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class AcsellFourbarCalculator extends FourbarCalculator {

	private final YoVariableRegistry registry;
	private final YoDouble beta0;
	private final YoDouble q_out;
	private final YoDouble q_in;
	private final YoDouble N;
	
	
	public AcsellFourbarCalculator(FourbarProperties fourbarProperties, String name, RobotSide side, YoVariableRegistry parentRegistry)
	{
		super(fourbarProperties);
				
		this.registry = new YoVariableRegistry(name + "Fourbar");
		this.beta0 = new YoDouble(name + "Fourbar_OutputOffsetAngle", registry);
		this.q_out = new YoDouble(name + "Fourbar_OutputAngle", registry);
		this.q_in = new YoDouble(name + "Fourbar_InputAngle", registry);
		this.N = new YoDouble(name + "Fourbar_TransmissionRatio", registry);
		
		if(side==RobotSide.LEFT)
			beta0.set(fourbarProperties.getLeftLinkageBeta0());
		else
			beta0.set(fourbarProperties.getRightLinkageBeta0());
		
		
		parentRegistry.addChild(registry);
	}
	
	public void update(AcsellJointCommand joint)
	{
		q_out.set(joint.getQ()*getBeltRatio());
		q_in.set(calculateInputAngleFromOutputAngle(q_out.getDoubleValue() + beta0.getDoubleValue())); //External angle from GroundLink to OutputLink

		N.set(getFourbarRatioBasedOnCalculatedInputAngle());		
	}
	
	public void update(OneDoFJoint joint)
	{
		q_out.set(joint.getQ()*getBeltRatio());
		q_in.set(calculateInputAngleFromOutputAngle(q_out.getDoubleValue() + beta0.getDoubleValue())); //External angle from GroundLink to OutputLink
		N.set(getFourbarRatioBasedOnCalculatedInputAngle());		
	}
	
	public double getBeltRatio()
	{		
		return 4.0/3.0;
	}
	
}
