package us.ihmc.exampleSimulations.groundTruthinator;

import us.ihmc.euclid.tuple3D.Point3D;

public class GroundTruthinatorSensor
{
   private final Point3D sensorPositionInWorldFrame = new Point3D();
   private final Point3D attachmentPositionInRobotFrame = new Point3D();
   private double sensedCableLength, estimatedCableLength;
   private double sensedCableVelocity, estimatedCableVelocity;


   public GroundTruthinatorSensor(Point3D sensorPosition, Point3D attachmentPosition)
   {
      sensorPositionInWorldFrame.set(sensorPosition);
      attachmentPositionInRobotFrame.set(attachmentPosition);
   }

   public void setSensorPositionInWorldFrame(Point3D sensorPositionInWorld)
   {
      this.sensorPositionInWorldFrame.set(sensorPositionInWorld);
   }

   public void setAttachmentPositionInRobotFrame(Point3D attachmentPositionInRobotFrame)
   {
      this.attachmentPositionInRobotFrame.set(attachmentPositionInRobotFrame);
   }

   public void setSensedCableLength(double cableLength)
   {
      this.sensedCableLength = cableLength;
   }

   public void setEstimatedCableLength(double cableLength)
   {
      this.estimatedCableLength = cableLength;
   }

   public void setSensedCableVelocity(double cableVelocity)
   {
      this.sensedCableVelocity = cableVelocity;
   }

   public void setEstimatedCableVelocity(double cableVelocity)
   {
      this.estimatedCableVelocity = cableVelocity;
   }

   public void getSensorPositionInWorldFrame(Point3D sensorPositionInWorldToPack)
   {
      sensorPositionInWorldToPack.set(sensorPositionInWorldFrame);
   }

   public void getAttachmentPositionInRobotFrame(Point3D attachmentPositionInRobotFrameToPack)
   {
      attachmentPositionInRobotFrameToPack.set(attachmentPositionInRobotFrame);
   }

   public double getSensedCableLength()
   {
      return sensedCableLength;
   }

   public double getEstimatedCableLength()
   {
      return estimatedCableLength;
   }

   public double getSensedCableVelocity()
   {
      return sensedCableVelocity;
   }

   public double getEstimatedCableVelocity()
   {
      return estimatedCableVelocity;
   }


}
