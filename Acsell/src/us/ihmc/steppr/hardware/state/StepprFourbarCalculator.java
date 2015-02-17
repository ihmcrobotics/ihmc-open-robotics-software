package us.ihmc.steppr.hardware.state;

import us.ihmc.acsell.fourbar.FourbarCalculator;
import us.ihmc.acsell.fourbar.StepprFourbarProperties;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprFourbarCalculator extends FourbarCalculator {

	private final YoVariableRegistry registry;
	private final DoubleYoVariable beta0;
	private final DoubleYoVariable q_out;
	private final DoubleYoVariable q_in;
	private final DoubleYoVariable N;
	private final StepprKneeJointState joint;
	
	
	public StepprFourbarCalculator(RobotSide side, StepprKneeJointState joint, YoVariableRegistry parentRegistry)
	{
		super(new StepprFourbarProperties());
		
		this.joint = joint;
		
		String name = side.getCamelCaseNameForStartOfExpression() + "Fourbar";
		
		this.registry = new YoVariableRegistry(name);
		this.beta0 = new DoubleYoVariable(name + "_OutputOffsetAngle", registry);
		this.q_out = new DoubleYoVariable(name + "_OutputAngle", registry);
		this.q_in = new DoubleYoVariable(name + "_InputAngle", registry);
		this.N = new DoubleYoVariable(name + "_TransmissionRatio", registry);
		
		beta0.set(2.0);
		
		parentRegistry.addChild(registry);
	}
	
	public void update()
	{
		q_out.set(joint.getQ()*getBeltRatio());
		setOutputAngle(q_out.getDoubleValue() + beta0.getDoubleValue()); //External angle from GroundLink to OutputLink
		updateFourbarKinematicEquationsFromOutputAngle();
		q_in.set(getInputAngle());
		N.set(getFourbarRatio());		
	}
	
	public double getBeltRatio()
	{		
		return 4.0/3.0;
	}
	
}
