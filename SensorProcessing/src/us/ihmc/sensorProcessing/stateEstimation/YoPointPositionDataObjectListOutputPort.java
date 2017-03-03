package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.YoPointPositionDataObject;


/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointPositionDataObjectListOutputPort extends ControlFlowOutputPort<List<PointPositionDataObject>>
{
   private final YoVariableRegistry registry;
   private final ArrayList<YoPointPositionDataObject> yoPointPositionDataObjects = new ArrayList<YoPointPositionDataObject>();
   private final Map<YoPointPositionDataObject, BooleanYoVariable> validMap = new LinkedHashMap<YoPointPositionDataObject, BooleanYoVariable>();
   private final String namePrefix;
   private final AfterJointReferenceFrameNameMap referenceFrameMap;  

   public YoPointPositionDataObjectListOutputPort(ControlFlowElement controlFlowElement, String namePrefix, AfterJointReferenceFrameNameMap referenceFrameMap, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      super.setData(new ArrayList<PointPositionDataObject>());
      this.namePrefix = namePrefix;
      this.registry = registry;
      this.referenceFrameMap = referenceFrameMap;
   }

   public List<PointPositionDataObject> getData()
   {
      List<PointPositionDataObject> data = super.getData();
      data.clear();

      for(int i = 0; i <  yoPointPositionDataObjects.size(); i++)
      {
         final YoPointPositionDataObject yoPointPositionDataObject = yoPointPositionDataObjects.get(i);
         if (validMap.get(yoPointPositionDataObject).getBooleanValue())
            data.add(yoPointPositionDataObject);
      }

      return data;
   }

   public void setData(List<PointPositionDataObject> data)
   {
      for (YoPointPositionDataObject yoPointPositionDataObject : yoPointPositionDataObjects)
      {
         validMap.get(yoPointPositionDataObject).set(false);
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
