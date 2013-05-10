package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import us.ihmc.controlFlow.ControlFlowElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.YoPointVelocityDataObject;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.AfterJointReferenceFrameNameMap;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointVelocityDataObjectListOutputPort extends ControlFlowOutputPort<Set<PointVelocityDataObject>>
{
   private final RigidBodyToIndexMap estimatorRigidBodyToIndexMap;
   private final AfterJointReferenceFrameNameMap referenceFrameNameMap;
   
   private final YoVariableRegistry registry;
   private final List<YoPointVelocityDataObject> yoPointVelocityDataObjects = new ArrayList<YoPointVelocityDataObject>();
   private final Map<YoPointVelocityDataObject, BooleanYoVariable> validMap = new LinkedHashMap<YoPointVelocityDataObject, BooleanYoVariable>();
   private final String namePrefix;

   public YoPointVelocityDataObjectListOutputPort(RigidBodyToIndexMap estimatorRigidBodyToIndexMap, AfterJointReferenceFrameNameMap referenceFrameNameMap,
         ControlFlowElement controlFlowElement, String namePrefix, YoVariableRegistry registry)
   {
      super(namePrefix, controlFlowElement);
      
      this.estimatorRigidBodyToIndexMap = estimatorRigidBodyToIndexMap;
      this.referenceFrameNameMap = referenceFrameNameMap;
      super.setData(new LinkedHashSet<PointVelocityDataObject>());
      this.namePrefix = namePrefix;
      this.registry = registry;
   }

   public Set<PointVelocityDataObject> getData()
   {
      Set<PointVelocityDataObject> data = super.getData();
      data.clear();

      for (YoPointVelocityDataObject yoPointVelocityDataObject : yoPointVelocityDataObjects)
      {
         if (validMap.get(yoPointVelocityDataObject).getBooleanValue())
            data.add(yoPointVelocityDataObject);
      }

      return data;
   }

   public void setData(Set<PointVelocityDataObject> data)
   {
      for (BooleanYoVariable validVariable : validMap.values())
      {
         validVariable.set(false);
      }

      for (PointVelocityDataObject pointVelocityDataObject : data)
      {
         ReferenceFrame referenceFrame = referenceFrameNameMap.getFrameByName(pointVelocityDataObject.getBodyFixedReferenceFrameName());

         YoPointVelocityDataObject yoPointVelocityDataObjectToUse = null;

         for (YoPointVelocityDataObject yoPointVelocityDataObject : yoPointVelocityDataObjects)
         {
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
            yoPointVelocityDataObjectToUse = new YoPointVelocityDataObject(estimatorRigidBodyToIndexMap, namePrefix + index, referenceFrame, registry);
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
