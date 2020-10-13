package us.ihmc.sensorProcessing.diagnostic;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class OneDoFJointFourierAnalysis implements DiagnosticUpdatable
{
   private final YoRegistry registry;

   private final OneDoFJointBasics joint;
   private final JointDesiredOutputReadOnly output;
   
   private final Online1DSignalFourierAnalysis velocityFourierAnalysis;
   private final Online1DSignalFourierAnalysis tauFourierAnalysis;
   private final Online1DSignalFourierAnalysis tauDesiredFourierAnalysis;

   private final boolean hasYoVariables;
   private final YoDouble velocity;
   private final YoDouble tau;
   private final YoDouble tauDesired;

   public OneDoFJointFourierAnalysis(OneDoFJointBasics joint, JointDesiredOutputReadOnly outputDataToCheck, double estimationWindow, double dt, YoRegistry parentRegistry)
   {
      this(joint, outputDataToCheck, estimationWindow, dt, null, null, null, parentRegistry);
   }

   public OneDoFJointFourierAnalysis(OneDoFJointBasics joint, double estimationWindow, double dt, YoDouble velocity, YoDouble tau, YoDouble tauDesired, YoRegistry parentRegistry)
   {
      this(joint, null, estimationWindow, dt, velocity, tau, tauDesired, parentRegistry);
   }
   private OneDoFJointFourierAnalysis(OneDoFJointBasics joint, JointDesiredOutputReadOnly outputDataToCheck, double estimationWindow, double dt, YoDouble velocity, YoDouble tau, YoDouble tauDesired, YoRegistry parentRegistry)
   {
      this.joint = joint;
      this.output = outputDataToCheck;
      String jointName = joint.getName();

      registry = new YoRegistry(jointName + "FourierAnalysis");
      parentRegistry.addChild(registry);

      velocityFourierAnalysis = new Online1DSignalFourierAnalysis(jointName + "Velocity", estimationWindow, dt, registry);
      tauFourierAnalysis = new Online1DSignalFourierAnalysis(jointName + "Tau", estimationWindow, dt, registry);
      tauDesiredFourierAnalysis = new Online1DSignalFourierAnalysis(jointName + "TauDesired", estimationWindow, dt, registry);

      if (velocity == null || tau == null || tauDesired == null)
      {
         hasYoVariables = false;
         this.velocity = null;
         this.tau = null;
         this.tauDesired = null;
      }
      else
      {
         hasYoVariables = true;
         verifyYoVariableNames(jointName, velocity, tau, tauDesired);
         this.velocity = velocity;
         this.tau = tau;
         this.tauDesired = tauDesired;
      }

   }

   @Override
   public void enable()
   {
      velocityFourierAnalysis.enable();
      tauFourierAnalysis.enable();
      tauDesiredFourierAnalysis.enable();
   }

   @Override
   public void disable()
   {
      velocityFourierAnalysis.disable();
      tauFourierAnalysis.disable();
      tauDesiredFourierAnalysis.disable();
   }

   @Override
   public void update()
   {
      if (hasYoVariables)
      {
         velocityFourierAnalysis.update(velocity.getDoubleValue());
         tauFourierAnalysis.update(tau.getDoubleValue());
         tauDesiredFourierAnalysis.update(tauDesired.getDoubleValue());
      }
      else
      {
         velocityFourierAnalysis.update(joint.getQd());
         tauFourierAnalysis.update(joint.getTau());
         tauDesiredFourierAnalysis.update(output.getDesiredTorque());
      }
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

   public void getVelocityOutput(DMatrixRMaj outputToPack)
   {
      velocityFourierAnalysis.getOutput(outputToPack);
   }

   public void getTauOutput(DMatrixRMaj outputToPack)
   {
      tauFourierAnalysis.getOutput(outputToPack);
   }

   public void getTauDesiredOutput(DMatrixRMaj outputToPack)
   {
      tauDesiredFourierAnalysis.getOutput(outputToPack);
   }

   public boolean hasAnalysisStarted()
   {
      return velocityFourierAnalysis.hasAnalysisStarted();
   }

   private void verifyYoVariableNames(String jointName, YoDouble velocity, YoDouble tau, YoDouble tauDesired)
   {
      if (!velocity.getName().contains(jointName))
         throw new RuntimeException("The velocity variable: " + velocity.getName() + " may not belong to the joint: " + jointName);
      if (!tau.getName().contains(jointName))
         throw new RuntimeException("The tau variable: " + tau.getName() + " may not belong to the joint: " + jointName);
      if (!tauDesired.getName().contains(jointName))
         throw new RuntimeException("The tauDesired variable: " + tauDesired.getName() + " may not belong to the joint: " + jointName);
   }
}
