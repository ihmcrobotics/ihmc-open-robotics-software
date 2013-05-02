package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import us.ihmc.sensorProcessing.stateEstimation.evaluation.RigidBodyToIndexMap;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class YoPointVelocityDataObject extends PointVelocityDataObject
{
   private final RigidBodyToIndexMap rigidBodyToIndexMapOne;
   private final RigidBodyToIndexMap rigidBodyToIndexMapTwo;
   
   private final IntegerYoVariable yoRigidBodyIndex;
   private final YoFramePoint yoMeasurementPointInBodyFrame;
   private final YoFrameVector yoVelocityOfMeasurementPointInWorldFrame;

   public YoPointVelocityDataObject(RigidBodyToIndexMap rigidBodyToIndexMapOne, RigidBodyToIndexMap rigidBodyToIndexMapTwo, 
         String namePrefix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this.rigidBodyToIndexMapOne = rigidBodyToIndexMapOne;
      this.rigidBodyToIndexMapTwo = rigidBodyToIndexMapTwo;
      yoRigidBodyIndex = new IntegerYoVariable(namePrefix + "RigidBodyIndex", registry);
      yoMeasurementPointInBodyFrame = new YoFramePoint(namePrefix + "PointBody", frame, registry);
      yoVelocityOfMeasurementPointInWorldFrame = new YoFrameVector(namePrefix + "PointVelocityWorld", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   public void set(RigidBody rigidBody, FramePoint measurementPointInBodyFrame, FrameVector velocityOfMeasurementPointInWorldFrame)
   {
      yoRigidBodyIndex.set(rigidBodyToIndexMapTwo.lookupIndexOfRigidBody(rigidBody));

      this.yoMeasurementPointInBodyFrame.set(measurementPointInBodyFrame);
      this.yoVelocityOfMeasurementPointInWorldFrame.set(velocityOfMeasurementPointInWorldFrame);
   }

   @Override
   public RigidBody getRigidBody()
   {
      rigidBody = rigidBodyToIndexMapOne.lookupRigidBody(yoRigidBodyIndex.getIntegerValue());
      return rigidBody;
   }

   @Override
   public FrameVector getVelocityOfMeasurementPointInWorldFrame()
   {
      yoVelocityOfMeasurementPointInWorldFrame.getFrameVectorAndChangeFrameOfPackedVector(velocityOfMeasurementPointInWorldFrame);

      return velocityOfMeasurementPointInWorldFrame;
   }

   @Override
   public FramePoint getMeasurementPointInBodyFrame()
   {
      yoMeasurementPointInBodyFrame.getFramePointAndChangeFrameOfPackedPoint(measurementPointInBodyFrame);

      return measurementPointInBodyFrame;
   }
}
