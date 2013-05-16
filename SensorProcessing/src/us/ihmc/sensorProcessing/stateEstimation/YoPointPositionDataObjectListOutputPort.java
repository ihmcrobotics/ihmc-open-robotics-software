package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.YoPointPositionDataObject;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.AfterJointReferenceFrameNameMap;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointPositionDataObjectListOutputPort extends ControlFlowOutputPort<Set<PointPositionDataObject>>
{
   private final YoVariableRegistry registry;
   private final ArrayList<YoPointPositionDataObject> yoPointPositionDataObjects = new ArrayList<YoPointPositionDataObject>();
   private final Map<YoPointPositionDataObject, BooleanYoVariable> validMap = new LinkedHashMap<YoPointPositionDataObject, BooleanYoVariable>();
   private final String namePrefix;
   private final AfterJointReferenceFrameNameMap referenceFrameMap;  

   public YoPointPositionDataObjectListOutputPort(ControlFlowElement controlFlowElement, String namePrefix, AfterJointReferenceFrameNameMap referenceFrameMap, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      super.setData(new LinkedHashSet<PointPositionDataObject>());
      this.namePrefix = namePrefix;
      this.registry = registry;
      this.referenceFrameMap = referenceFrameMap;
   }

   public Set<PointPositionDataObject> getData()
   {
      Set<PointPositionDataObject> data = super.getData();
      data.clear();

      for(int i = 0; i <  yoPointPositionDataObjects.size(); i++)
      {
         final YoPointPositionDataObject yoPointPositionDataObject = yoPointPositionDataObjects.get(i);
         if (validMap.get(yoPointPositionDataObject).getBooleanValue())
            data.add(yoPointPositionDataObject);
      }

      return data;
   }

   public void setData(Set<PointPositionDataObject> data)
   {
      for (BooleanYoVariable validVariable : validMap.values())
      {
         validVariable.set(false);
      }

      for (PointPositionDataObject pointPositionDataObject : data)
      {
         String referenceFrameName = pointPositionDataObject.getBodyFixedReferenceFrameName();

         YoPointPositionDataObject yoPointPositionDataObjectToUse = null;

         for(int i = 0; i <  yoPointPositionDataObjects.size(); i++)
         {
            YoPointPositionDataObject yoPointPositionDataObject = yoPointPositionDataObjects.get(i);
            boolean frameOK = yoPointPositionDataObject.getBodyFixedReferenceFrameName() == referenceFrameName;
            boolean isAvailable = !validMap.get(yoPointPositionDataObject).getBooleanValue();
            if (frameOK && isAvailable)
            {
               yoPointPositionDataObjectToUse = yoPointPositionDataObject;

               break;
            }
         }

         if (yoPointPositionDataObjectToUse == null)
         {
            int index = yoPointPositionDataObjects.size();
            ReferenceFrame frame = referenceFrameMap.getFrameByName(referenceFrameName);
            yoPointPositionDataObjectToUse = new YoPointPositionDataObject(namePrefix + index, frame, registry);
            yoPointPositionDataObjects.add(yoPointPositionDataObjectToUse);
            validMap.put(yoPointPositionDataObjectToUse, new BooleanYoVariable(namePrefix + "Valid" + index, registry));
         }

         yoPointPositionDataObjectToUse.set(pointPositionDataObject);
         validMap.get(yoPointPositionDataObjectToUse).set(true);
      }
   }

   public int getNumberOfYoPointPositionDataObjects()
   {
      return yoPointPositionDataObjects.size();
   }
}
