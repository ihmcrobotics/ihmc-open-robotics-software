package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptionTools;
import us.ihmc.sensorProcessing.signalCorruption.SignalCorruptor;

public abstract class SimulatedSensor<T> extends AbstractControlFlowElement
{
   private final List<SignalCorruptor<T>> signalCorruptors = new ArrayList<SignalCorruptor<T>>();
   private final String name;
   private final DenseMatrix64F covarianceMatrix;
   
   public SimulatedSensor(String name, int size)
   {
      this.name = name;
      covarianceMatrix = new DenseMatrix64F(size, size);
   }
   
   public final void addSignalCorruptor(SignalCorruptor<T> signalCorruptor)
   {
      signalCorruptors.add(signalCorruptor);
   }

   public void getCovarianceMatrix(DenseMatrix64F covarianceMatrixToPack)
   {
      covarianceMatrixToPack.set(covarianceMatrix);
   }
   
   public DenseMatrix64F getCovarianceMatrix()
   {
      return covarianceMatrix;
   }
   
   public void setCovarianceMatrix(DenseMatrix64F covarianceMatrix)
   {
      this.covarianceMatrix.set(covarianceMatrix);
   }
   
   protected final void corrupt(T signal)
   {
      SignalCorruptionTools.corrupt(signal, signalCorruptors);
   }
   
   public String getName()
   {
      return name;
   }
}
