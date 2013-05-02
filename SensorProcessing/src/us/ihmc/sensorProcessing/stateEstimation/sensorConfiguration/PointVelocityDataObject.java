package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class PointVelocityDataObject
{
   protected RigidBody rigidBody;
   protected final FramePoint measurementPointInBodyFrame = new FramePoint();
//   protected final FramePoint positionOfMeasurementPointInWorldFrame = new FramePoint(ReferenceFrame.getWorldFrame());
   protected final FrameVector velocityOfMeasurementPointInWorldFrame = new FrameVector(ReferenceFrame.getWorldFrame());

   public void set(RigidBody rigidBody, FramePoint measurementPointInBodyFrame, 
//         FramePoint positionOfMeasurementPointInWorldFrame, 
         FrameVector velocityOfMeasurementPointInWorldFrame)
   {
      this.rigidBody = rigidBody;
      this.measurementPointInBodyFrame.setAndChangeFrame(measurementPointInBodyFrame);
//      this.positionOfMeasurementPointInWorldFrame.set(positionOfMeasurementPointInWorldFrame);
      this.velocityOfMeasurementPointInWorldFrame.set(velocityOfMeasurementPointInWorldFrame);
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }
   
//   public FramePoint getMeasurementPointInWorldFrame()
//   {
//      return positionOfMeasurementPointInWorldFrame;
//   }
   
   public FrameVector getVelocityOfMeasurementPointInWorldFrame()
   {
      return velocityOfMeasurementPointInWorldFrame;
   }

   public FramePoint getMeasurementPointInBodyFrame()
   {
      return measurementPointInBodyFrame;
   }

   public void set(PointVelocityDataObject other)
   {
      set(other.getRigidBody(), other.getMeasurementPointInBodyFrame(), 
//            other.getMeasurementPointInWorldFrame(), 
            other.getVelocityOfMeasurementPointInWorldFrame());
   }

   public boolean epsilonEquals(PointVelocityDataObject other, double epsilon)
   {
      if (getMeasurementPointInBodyFrame().getReferenceFrame() != other.getMeasurementPointInBodyFrame().getReferenceFrame())
         return false;

      boolean rigidBodyEqual = getRigidBody().getName().equals(other.getRigidBody().getName());
      boolean bodyPointsEqual = getMeasurementPointInBodyFrame().epsilonEquals(other.getMeasurementPointInBodyFrame(), epsilon);
//      boolean worldPointsEqual = getMeasurementPointInWorldFrame().epsilonEquals(other.getMeasurementPointInWorldFrame(), epsilon);
      boolean worldVelocitiesEqual = getVelocityOfMeasurementPointInWorldFrame().epsilonEquals(other.getVelocityOfMeasurementPointInWorldFrame(), epsilon);
      return rigidBodyEqual && bodyPointsEqual 
            //&& worldPointsEqual 
            && worldVelocitiesEqual;
   }


}
