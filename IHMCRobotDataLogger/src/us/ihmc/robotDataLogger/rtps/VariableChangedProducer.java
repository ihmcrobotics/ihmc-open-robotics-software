package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.List;

import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class VariableChangedProducer
{
   private boolean sendVariableChanges = true;
   private final DataConsumerParticipant participant;
   private final TObjectIntHashMap<YoVariable<?>> variableIdentifiers = new TObjectIntHashMap<>();
   private final VariableListener variableListener = new VariableListener();

   public VariableChangedProducer(DataConsumerParticipant participant)
   {
      this.participant = participant;
   }

   /**
    * Start the variable changed producer listener and add listener to all variableIdentifiers.
    * 
    * @param variables List of variables.
    * @throws IOException if the producer cannot be created
    */
   public void startVariableChangedProducers(Announcement announcement, List<YoVariable<?>> variables) throws IOException
   {
      participant.createVariableChangeProducer(announcement);
      for (int i = 0; i < variables.size(); i++)
      {
         this.variableIdentifiers.put(variables.get(i), i);
         variables.get(i).addVariableChangedListener(variableListener);
      }
   }

   public class VariableListener implements VariableChangedListener
   {

      @Override
      public void variableChanged(YoVariable<?> v)
      {
         if (sendVariableChanges)
         {
            try
            {
               participant.writeVariableChangeRequest(variableIdentifiers.get(v), v.getValueAsDouble());
            }
            catch (IOException e)
            {
               // Do not crash but just show the stack trace. 
               e.printStackTrace();
            }
         }
      }

   }

   public void setSendingChangesEnabled(boolean sendVariableChanges)
   {
      this.sendVariableChanges = sendVariableChanges;
   }
}
