package us.ihmc.simulationToolkit.controllers;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationToolkit.parameters.SimulatedElasticityParameters;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SpringJointOutputWriter implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   ArrayList<OneDegreeOfFreedomJoint> elasticJoints = new ArrayList<>();
   private final HashMap<OneDegreeOfFreedomJoint, AlphaFilteredYoVariable> filteredDesiredJointAngles = new HashMap<>();
   private final HashMap<OneDegreeOfFreedomJoint, DoubleYoVariable> jointStiffness = new HashMap<>();
   private final HashMap<OneDegreeOfFreedomJoint, DoubleYoVariable> maxDeflections = new HashMap<>();

   public SpringJointOutputWriter(FloatingRootJointRobot robot, SimulatedElasticityParameters elasticityParameters, double dt)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint simulatedJoint : oneDegreeOfFreedomJoints)
      {
         if(elasticityParameters.isSpringJoint(simulatedJoint))
         {
            elasticJoints.add(simulatedJoint);
            String jointName = simulatedJoint.getName();

            double breakFrequency = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(20.0, dt);
            AlphaFilteredYoVariable filteredDesired = new AlphaFilteredYoVariable(jointName + "_filteredQDesired", registry, breakFrequency);
            filteredDesired.update(0.0);
            filteredDesiredJointAngles.put(simulatedJoint, filteredDesired);
            
            DoubleYoVariable stiffness = new DoubleYoVariable(simulatedJoint.getName() + "_stiffness", registry);
            stiffness.set(elasticityParameters.getStiffness(simulatedJoint));
            jointStiffness.put(simulatedJoint, stiffness);
            
            DoubleYoVariable maxDeflection = new DoubleYoVariable(simulatedJoint.getName() + "_maxDeflection", registry);
            maxDeflection.set(elasticityParameters.getMaxDeflection(simulatedJoint));
            maxDeflections.put(simulatedJoint, maxDeflection);
         }
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      for(OneDegreeOfFreedomJoint simulatedJoint : elasticJoints)
      {
         double tau = simulatedJoint.getTau();
         DoubleYoVariable stiffness = jointStiffness.get(simulatedJoint);
         DoubleYoVariable maxDeflection = maxDeflections.get(simulatedJoint);
         AlphaFilteredYoVariable filteredDesired = filteredDesiredJointAngles.get(simulatedJoint);
         double qDesired = -MathTools.clamp(tau / stiffness.getDoubleValue(), maxDeflection.getDoubleValue());
         
         filteredDesired.update(qDesired);
         
         simulatedJoint.setQ(filteredDesired.getDoubleValue());
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return null;
   }
}
