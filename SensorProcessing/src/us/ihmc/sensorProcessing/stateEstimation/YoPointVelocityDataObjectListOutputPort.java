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
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.YoPointVelocityDataObject;


/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointVelocityDataObjectListOutputPort extends ControlFlowOutputPort<List<PointVelocityDataObject>>
{
   private final AfterJointReferenceFrameNameMap referenceFrameNameMap;
   
   private final YoVariableRegistry registry;
   private final ArrayList<YoPointVelocityDataObject> yoPointVelocityDataObjects = new ArrayList<YoPointVelocityDataObject>();
   private final Map<YoPointVelocityDataObject, BooleanYoVariable> validMap = new LinkedHashMap<YoPointVelocityDataObject, BooleanYoVariable>();
   private final String namePrefix;

   public YoPointVelocityDataObjectListOutputPort(AfterJointReferenceFrameNameMap referenceFrameNameMap,
         ControlFlowElement controlFlowElement, String namePrefix, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      
      this.referenceFrameNameMap = referenceFrameNameMap;
      super.setData(new ArrayList<PointVelocityDataObject>());
      this.namePrefix = namePrefix;
      this.registry = registry;
   }

   public List<PointVelocityDataObject> getData()
   {
      List<PointVelocityDataObject> data = super.getData();
      data.clear();

      for (YoPointVelocityDataObject yoPointVelocityDataObject : yoPointVelocityDataObjects)
      {
         if (validMap.get(yoPointVelocityDataObject).getBooleanValue())
            data.add(yoPointVelocityDataObject);
      }

      return data;
   }

   public void setData(List<PointVelocityDataObject> data)
   {
      for (YoPointVelocityDataObject yoPointVelocityDataObject : yoPointVelocityDataObjects)
      {
         validMap.get(yoPointVelocityDataObject).set(false);
      }

      for (PointVelocityDataObject pointVelocityDataObject : data)
      {
         ReferenceFrame referenceFrame = referenceFrameNameMap.getFrameByName(pointVelocityDataObject.getBodyFixedReferenceFrameName());

         YoPointVelocityDataObject yoPointVelocityDataObjectToUse = null;

         for(int i = 0; i <  yoPointVelocityDataObjects.size(); i++)
         {
            YoPointVelocityDataObject yoPointVelocityDataObject = yoPointVelocityDataObjects.get(i);
            boolean frameOK = yoPointVelocityDataObject.getReferenceFrame() == referenceFrame;
            boolean isAvailable = !validMap.get(yoPointVelocityDataObject).getBooleanValue();
            if (frameOK && isAvailable)
            {
               yoPointVelocityDataObjectToUse = yoPointVelocityDataObject;

               break;
            }
         }

         if (yoPointVelocityDataObjectToUse == null)
         {
            int index = yoPointVelocityDataObjects.size();
            yoPointVelocityDataObjectToUse = new YoPointVelocityDataObject(namePrefix + index, referenceFrame, registry);
            yoPointVelocityDataObjects.add(yoPointVelocityDataObjectToUse);
            validMap.put(yoPointVelocityDataObjectToUse, new BooleanYoVariable(namePrefix + "Valid" + index, registry));
         }

         yoPointVelocityDataObjectToUse.set(pointVelocityDataObject);
         validMap.get(yoPointVelocityDataObjectToUse).set(true);
      }
   }

   public int getNumberOfYoPointVelocityDataObjects()
   {
      return yoPointVelocityDataObjects.size();
   }
}
