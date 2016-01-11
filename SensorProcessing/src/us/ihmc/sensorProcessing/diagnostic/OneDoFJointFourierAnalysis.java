package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class OneDoFJointFourierAnalysis implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final OneDoFJoint joint;

   private final Online1DSignalFourierAnalysis velocityFourierAnalysis;
   private final Online1DSignalFourierAnalysis tauFourierAnalysis;
   private final Online1DSignalFourierAnalysis tauDesiredFourierAnalysis;

   public OneDoFJointFourierAnalysis(OneDoFJoint joint, double estimationWindow, double dt, YoVariableRegistry parentRegistry)
   {
      this.joint = joint;
      String jointName = joint.getName();

      registry = new YoVariableRegistry(jointName + "FourierAnalysis");
      parentRegistry.addChild(registry);

      velocityFourierAnalysis = new Online1DSignalFourierAnalysis(jointName + "Velocity", estimationWindow, dt, registry);
      tauFourierAnalysis = new Online1DSignalFourierAnalysis(jointName + "Tau", estimationWindow, dt, registry);
      tauDesiredFourierAnalysis = new Online1DSignalFourierAnalysis(jointName + "TauDesired", estimationWindow, dt, registry);
   }

   public void enable()
   {
      velocityFourierAnalysis.enable();
      tauFourierAnalysis.enable();
      tauDesiredFourierAnalysis.enable();
   }

   public void disable()
   {
      velocityFourierAnalysis.disable();
      tauFourierAnalysis.disable();
      tauDesiredFourierAnalysis.disable();
   }

   @Override
   public void update()
   {
      velocityFourierAnalysis.update(joint.getQd());
      tauFourierAnalysis.update(joint.getTauMeasured());
      tauDesiredFourierAnalysis.update(joint.getTau());
   }
}
