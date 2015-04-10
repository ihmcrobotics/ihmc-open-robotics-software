package us.ihmc.steppr.hardware.state;

import us.ihmc.acsell.fourbar.FourbarCalculator;
import us.ihmc.acsell.fourbar.StepprFourbarProperties;
import us.ihmc.steppr.hardware.command.StepprJointCommand;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprFourbarCalculator extends FourbarCalculator {

	private final YoVariableRegistry registry;
	private final DoubleYoVariable beta0;
	private final DoubleYoVariable q_out;
	private final DoubleYoVariable q_in;
	private final DoubleYoVariable N;
	
	
	public StepprFourbarCalculator(String name, RobotSide side, YoVariableRegistry parentRegistry)
	{
		super(new StepprFourbarProperties());
				
		this.registry = new YoVariableRegistry(name + "Fourbar");
		this.beta0 = new DoubleYoVariable(name + "Fourbar_OutputOffsetAngle", registry);
		this.q_out = new DoubleYoVariable(name + "Fourbar_OutputAngle", registry);
		this.q_in = new DoubleYoVariable(name + "Fourbar_InputAngle", registry);
		this.N = new DoubleYoVariable(name + "Fourbar_TransmissionRatio", registry);
		
		if(side==RobotSide.LEFT)
			beta0.set(StepprFourbarProperties.LEFT_LINKAGE_BETA0);
		else
			beta0.set(StepprFourbarProperties.RIGHT_LINKAGE_BETA0);
		
		
		parentRegistry.addChild(registry);
	}
	
	public void update(StepprJointCommand joint)
	{
		q_out.set(joint.getQ()*getBeltRatio());
		setOutputAngle(q_out.getDoubleValue() + beta0.getDoubleValue()); //External angle from GroundLink to OutputLink
		updateFourbarKinematicEquationsFromOutputAngle();
		q_in.set(getInputAngle());
		N.set(getFourbarRatio());		
	}
	
	public void update(OneDoFJoint joint)
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
