package us.ihmc.simulationConstructionSetTools.simulationTesting;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class BodyPositionSimulationComparer implements SimulationComparer
{
   private final double epsilon;
   public BodyPositionSimulationComparer(double epsilon)
   {
      this.epsilon = epsilon;
   }

   @Override
   public boolean compare(SimulationConstructionSet scs1, SimulationConstructionSet scs2)
   {
      // compare variables
      YoVariable<?> var0 = getRootJoint(scs1).getQx();
      YoVariable<?> var1 = getRootJoint(scs2).getQx();
      return (MathTools.epsilonEquals(var0.getValueAsDouble(), var1.getValueAsDouble(), epsilon));
   }

   private FloatingJoint getRootJoint(SimulationConstructionSet scs0)
   {
      Joint firstRootJoint = scs0.getRobots()[0].getRootJoints().get(0);
      if (firstRootJoint instanceof FloatingJoint)
         return (FloatingJoint) firstRootJoint;
      throw new RuntimeException("first root joint is not a floating joint: " + firstRootJoint);
   }

}
