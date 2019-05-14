package us.ihmc.sensorProcessing.sensors;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.tools.lists.PairList;

public class RawJointSensorDataHolderMapCopier
{
   private final PairList<RawJointSensorDataHolder, RawJointSensorDataHolder> originalAndTarget = new PairList<RawJointSensorDataHolder, RawJointSensorDataHolder>();
   
   public RawJointSensorDataHolderMapCopier(RawJointSensorDataHolderMap originalMap, RawJointSensorDataHolderMap targetMap)
   {
      List<RawJointSensorDataHolder> originals = new ArrayList<RawJointSensorDataHolder>(originalMap.values());
      List<RawJointSensorDataHolder> targets = new ArrayList<RawJointSensorDataHolder>(targetMap.values());
      
      if(originals.size() != targets.size())
      {
         throw new RuntimeException("Original and Target are not of equal length");
      }
      
      for(int i = 0; i < originals.size(); i++)
      {
         RawJointSensorDataHolder original = originals.get(i);
         RawJointSensorDataHolder target = targets.get(i); 
         if(!original.getName().equals(target.getName()))
         {
            throw new RuntimeException("Original and Target don't match. Got: " + target.getName() + ", expected: " + original.getName());
         }
         
         originalAndTarget.add(original, target);
      }
   }
   
   public void copy()
   {
      for(int i = 0; i < originalAndTarget.size(); i++)
      {
         RawJointSensorDataHolder original = originalAndTarget.first(i);
         RawJointSensorDataHolder target = originalAndTarget.second(i);
         
         target.set(original);
      }
   }
}
