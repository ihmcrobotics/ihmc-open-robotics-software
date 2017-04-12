package us.ihmc.simulationConstructionSetTools.util.graphs;

import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.gui.BodePlotConstructor;

public class FrequencyPlot extends JFreePlot
{
   private static final long serialVersionUID = 256604511093215927L;

   public FrequencyPlot(String name, DataBufferEntry time, DataBufferEntry data)
   {
      this(name, time.getData(), data.getData());
   }

   public FrequencyPlot(String name, double[] time, double[] data)
   {
      super(name);
      
      double avg = 0;
      for(int i = 0; i < data.length; i++)
         avg += data[i];
      avg /= data.length;
      
      double[] dataDivAvg = new double[data.length];
      for(int i = 0; i < data.length; i++)
         dataDivAvg[i] = data[i] - avg;
      
      double[][] freqMagnitudePhaseData = BodePlotConstructor.computeFreqMagPhase(time, dataDivAvg);
      
      double[] freq = freqMagnitudePhaseData[0];
      double[] magnitude = freqMagnitudePhaseData[1];
      
      
      createXYSeries(freq, magnitude);      
      
   }
}
