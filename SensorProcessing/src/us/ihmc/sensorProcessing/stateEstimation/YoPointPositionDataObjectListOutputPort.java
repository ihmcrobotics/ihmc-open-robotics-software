package us.ihmc.sensorProcessing.stateEstimation;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import org.apache.commons.lang.mutable.MutableBoolean;
import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.YoPointPositionDataObject;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import java.util.*;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointPositionDataObjectListOutputPort extends ControlFlowOutputPort<Set<PointPositionDataObject>>
{
   private final YoVariableRegistry registry;
   private final List<YoPointPositionDataObject> yoPointPositionDataObjects = new ArrayList<YoPointPositionDataObject>();
   private final Map<YoPointPositionDataObject, BooleanYoVariable> validMap = new LinkedHashMap<YoPointPositionDataObject, BooleanYoVariable>();
   private final String namePrefix;

   public YoPointPositionDataObjectListOutputPort(ControlFlowElement controlFlowElement, String namePrefix, YoVariableRegistry registry)
   {
      super(controlFlowElement);
      super.setData(new LinkedHashSet<PointPositionDataObject>());
      this.namePrefix = namePrefix;
      this.registry = registry;
   }

   public Set<PointPositionDataObject> getData()
   {
      Set<PointPositionDataObject> data = super.getData();
      data.clear();

      for (YoPointPositionDataObject yoPointPositionDataObject : yoPointPositionDataObjects)
      {
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
         ReferenceFrame referenceFrame = pointPositionDataObject.getMeasurementPointInBodyFrame().getReferenceFrame();

         YoPointPositionDataObject yoPointPositionDataObjectToUse = null;

         for (YoPointPositionDataObject yoPointPositionDataObject : yoPointPositionDataObjects)
         {
            boolean frameOK = yoPointPositionDataObject.getMeasurementPointInBodyFrame().getReferenceFrame() == referenceFrame;
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
            yoPointPositionDataObjectToUse = new YoPointPositionDataObject(namePrefix + index, referenceFrame, registry);
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
