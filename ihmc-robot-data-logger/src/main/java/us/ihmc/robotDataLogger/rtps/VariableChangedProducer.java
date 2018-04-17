package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;
import java.util.List;

import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoVariable;

public class VariableChangedProducer
{
   private final Object lock = new Object();
   
   private DataConsumerSession session = null;
   
   private final TObjectIntHashMap<YoVariable<?>> variableIdentifiers = new TObjectIntHashMap<>();
   private final VariableListener variableListener = new VariableListener();

   public VariableChangedProducer()
   {
   }

   /**
    * Start the variable changed producer listener and add listener to all variableIdentifiers.
    * 
    * @param variables List of variables.
    * @throws IOException if the producer cannot be created
    */
   public void startVariableChangedProducers(List<YoVariable<?>> variables) throws IOException
   {
      for (int i = 0; i < variables.size(); i++)
      {
         this.variableIdentifiers.put(variables.get(i), i);
         variables.get(i).addVariableChangedListener(variableListener);
      }
   }

   public class VariableListener implements VariableChangedListener
   {

      @Override
      public void notifyOfVariableChange(YoVariable<?> v)
      {
         try
         {
            synchronized(lock)
            {
               if(session != null)
               {
                  session.writeVariableChangeRequest(variableIdentifiers.get(v), v.getValueAsDouble());
               }
            }
         }
         catch (IOException e)
         {
            // Do not crash but just show the stack trace. 
            e.printStackTrace();
         }
      }
   }

   void setSession(DataConsumerSession session)
   {
      synchronized(lock)
      {
         this.session = session;
      }
   }
}
