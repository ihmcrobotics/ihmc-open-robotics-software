package us.ihmc.exampleSimulations.omniWrist;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class OmniWristController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final PDController pdControllerA, pdControllerB;
   private final DoubleYoVariable q_d_jointOneA = new DoubleYoVariable("q_d_jointOneA", registry);
   private final DoubleYoVariable q_d_jointOneB = new DoubleYoVariable("q_d_jointOneB", registry);
   
   private final DoubleYoVariable qd_d_jointOneA = new DoubleYoVariable("qd_d_jointOneA", registry);
   private final DoubleYoVariable qd_d_jointOneB = new DoubleYoVariable("qd_d_jointOneB", registry);
   
   private final YoFunctionGenerator functionGeneratorA, functionGeneratorB;
   
   private final PinJoint jointOneA, jointOneB, jointOneC, jointOneD;
   
   public OmniWristController(Robot robot)
   {
      jointOneA = (PinJoint) robot.getJoint("jointOneA");
      jointOneB = (PinJoint) robot.getJoint("jointOneB");
      jointOneC = (PinJoint) robot.getJoint("jointOneC");
      jointOneD = (PinJoint) robot.getJoint("jointOneD");
      
      pdControllerA = new PDController("ControllerA", registry);
      pdControllerB = new PDController("ControllerB", registry);
      
      pdControllerA.setProportionalGain(1000.0);
      pdControllerA.setDerivativeGain(50.0);
      
      pdControllerB.setProportionalGain(1000.0);
      pdControllerB.setDerivativeGain(50.0);
      
      functionGeneratorA = new YoFunctionGenerator("functionA", robot.getYoTime(), registry);
      functionGeneratorB = new YoFunctionGenerator("functionB", robot.getYoTime(), registry);
      
      functionGeneratorA.setMode(YoFunctionGeneratorMode.SINE);
      functionGeneratorB.setMode(YoFunctionGeneratorMode.SINE);
      
      functionGeneratorA.setAmplitude(Math.PI/8.0);
      functionGeneratorA.setOffset(-0.4);
      functionGeneratorA.setFrequency(1.0 * 0.33);
      
      functionGeneratorB.setAmplitude(Math.PI/8.0);
      functionGeneratorB.setOffset(-0.4);
      functionGeneratorB.setFrequency(1.0 * 1.0);

   }
   
   @Override
   public void initialize()
   {      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      q_d_jointOneA.set(functionGeneratorA.getValue());
      q_d_jointOneB.set(functionGeneratorB.getValue());
      
      qd_d_jointOneA.set(functionGeneratorA.getValueDot());
      qd_d_jointOneB.set(functionGeneratorB.getValueDot());

      jointOneA.setTau(pdControllerA.compute(jointOneA.getQ(), q_d_jointOneA.getDoubleValue(), jointOneA.getQD(), qd_d_jointOneA.getDoubleValue()));
      jointOneB.setTau(pdControllerB.compute(jointOneB.getQ(), q_d_jointOneB.getDoubleValue(), jointOneB.getQD(), qd_d_jointOneB.getDoubleValue()));
      
   }

}
