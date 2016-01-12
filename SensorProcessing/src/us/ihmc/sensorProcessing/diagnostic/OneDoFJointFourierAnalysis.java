package us.ihmc.sensorProcessing.diagnostic;

import org.ejml.data.DenseMatrix64F;

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

   public int getOutputSize()
   {
      return velocityFourierAnalysis.getOutputSize();
   }

   public double getFrequency(int index)
   {
      return velocityFourierAnalysis.getFrequency(index);
   }

   public void getFrequencies(double[] frequenciesToPack)
   {
      velocityFourierAnalysis.getFrequencies(frequenciesToPack);
   }

   public double getVelocityMagnitude(int index)
   {
      return velocityFourierAnalysis.getMagnitude(index);
   }

   public void getVelocityMagnitudes(double[] magnitudesToPack)
   {
      velocityFourierAnalysis.getMagnitudes(magnitudesToPack);
   }

   public double getTauMagnitude(int index)
   {
      return tauFourierAnalysis.getMagnitude(index);
   }

   public void getTauMagnitudes(double[] magnitudesToPack)
   {
      tauFourierAnalysis.getMagnitudes(magnitudesToPack);
   }

   public double getTauDesiredMagnitude(int index)
   {
      return tauDesiredFourierAnalysis.getMagnitude(index);
   }

   public void getTauDesiredMagnitudes(double[] magnitudesToPack)
   {
      tauDesiredFourierAnalysis.getMagnitudes(magnitudesToPack);
   }

   public void getVelocityOutput(DenseMatrix64F outputToPack)
   {
      velocityFourierAnalysis.getOutput(outputToPack);
   }

   public void getTauOutput(DenseMatrix64F outputToPack)
   {
      tauFourierAnalysis.getOutput(outputToPack);
   }

   public void getTauDesiredOutput(DenseMatrix64F outputToPack)
   {
      tauDesiredFourierAnalysis.getOutput(outputToPack);
   }

   public boolean hasAnalysisStarted()
   {
      return velocityFourierAnalysis.hasAnalysisStarted();
   }
}
