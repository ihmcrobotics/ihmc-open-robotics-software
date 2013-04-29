package us.ihmc.sensorProcessing.stateEstimation;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.YoPointPositionDataObject;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointPositionDataObjectListOutputPort extends ControlFlowOutputPort<Set<PointPositionDataObject>>
{
   private final YoVariableRegistry registry;
   private final List<YoPointPositionDataObject> yoPointPositionDataObjects = new ArrayList<YoPointPositionDataObject>();
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
         if (yoPointPositionDataObject.isValid())
            data.add(yoPointPositionDataObject);
      }

      return data;
   }

   public void setData(Set<PointPositionDataObject> data)
   {
      for (YoPointPositionDataObject yoPointPositionDataObject : yoPointPositionDataObjects)
      {
         yoPointPositionDataObject.setValid(false);
      }

      for (PointPositionDataObject pointPositionDataObject : data)
      {
         ReferenceFrame referenceFrame = pointPositionDataObject.getMeasurementPointInBodyFrame().getReferenceFrame();

         YoPointPositionDataObject yoPointPositionDataObjectToUse = null;

         for (YoPointPositionDataObject yoPointPositionDataObject : yoPointPositionDataObjects)
         {
            boolean frameOK = yoPointPositionDataObject.getMeasurementPointInBodyFrame().getReferenceFrame() == referenceFrame;
            boolean isAvailable = !yoPointPositionDataObject.isValid();
            if (frameOK && isAvailable)
            {
               yoPointPositionDataObjectToUse = yoPointPositionDataObject;

               break;
            }
         }

         if (yoPointPositionDataObjectToUse == null)
         {
            yoPointPositionDataObjectToUse = new YoPointPositionDataObject(namePrefix + yoPointPositionDataObjects.size(), referenceFrame, registry);
            yoPointPositionDataObjects.add(yoPointPositionDataObjectToUse);
         }

         yoPointPositionDataObjectToUse.set(pointPositionDataObject);
         yoPointPositionDataObjectToUse.setValid(true);
      }
   }

   public int getNumberOfYoPointPositionDataObjects()
   {
      return yoPointPositionDataObjects.size();
   }
}
