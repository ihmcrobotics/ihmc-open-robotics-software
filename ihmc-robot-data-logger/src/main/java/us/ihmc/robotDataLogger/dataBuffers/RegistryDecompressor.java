package us.ihmc.robotDataLogger.dataBuffers;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataLogger.rtps.VariableChangedProducer;
import us.ihmc.tools.compression.CompressionImplementation;
import us.ihmc.tools.compression.CompressionImplementationFactory;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

public class RegistryDecompressor
{
   private final List<YoVariable<?>> variables;
   private final List<JointState> jointStates;
   
   private final ByteBuffer decompressBuffer;
   private final CompressionImplementation compressionImplementation;

   
   public RegistryDecompressor(List<YoVariable<?>> variables, List<JointState> jointStates)
   {
      this.variables = variables;
      this.jointStates = jointStates;
      this.decompressBuffer = ByteBuffer.allocate(variables.size() * 8);
      
      this.compressionImplementation = CompressionImplementationFactory.instance();

   }
   
   private void setAndNotify(YoVariable<?> variable, long newValue)
   {
      long previousValue = variable.getValueAsLongBits();
      variable.setValueFromLongBits(newValue, false);
      if (previousValue != newValue)
      {
         ArrayList<VariableChangedListener> changedListeners = variable.getVariableChangedListeners();
         if (changedListeners != null)
         {
            for (int listener = 0; listener < changedListeners.size(); listener++)
            {
               VariableChangedListener changedListener = changedListeners.get(listener);
               if (!(changedListener instanceof VariableChangedProducer.VariableListener))
               {
                  changedListener.notifyOfVariableChange(variable);
               }
            }
         }
      }
   }
   
   public void decompressSegment(RegistryReceiveBuffer buffer, int registryOffset)
   {
      decompressBuffer.clear();
      compressionImplementation.decompress(buffer.getData(), decompressBuffer, buffer.getNumberOfVariables() * 8);      
      decompressBuffer.flip();
      LongBuffer longData = decompressBuffer.asLongBuffer();
      
      // Sanity check
      if(longData.remaining() != buffer.getNumberOfVariables())
      {
         System.err.println("Number of variables in incoming message does not match stated number of variables. Skipping packet.");
         return;
      }
      int numberOfVariables = buffer.getNumberOfVariables();
      
      int offset = registryOffset + buffer.getOffset();
      for(int i = 0; i < numberOfVariables; i++)
      {
         setAndNotify(variables.get(i + offset), longData.get());
      }
      
      double[] jointStateArray = buffer.getJointStates();
      if(jointStateArray.length > 0)
      {
         DoubleBuffer jointStateBuffer = DoubleBuffer.wrap(jointStateArray);
         for(int i = 0; i < jointStates.size(); i++)
         {
            jointStates.get(i).update(jointStateBuffer);
         }         
      }

   }
}
