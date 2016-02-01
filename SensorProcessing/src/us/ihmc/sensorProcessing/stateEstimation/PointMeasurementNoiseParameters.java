package us.ihmc.sensorProcessing.stateEstimation;

public class PointMeasurementNoiseParameters
{
   private final double pointVelocityXYMeasurementStandardDeviation;
   private final double pointVelocityZMeasurementStandardDeviation;

   private final double pointPositionXYMeasurementStandardDeviation;
   private final double pointPositionZMeasurementStandardDeviation;

   public PointMeasurementNoiseParameters(double pointVelocityXYMeasurementStandardDeviation, double pointVelocityZMeasurementStandardDeviation,
           double pointPositionXYMeasurementStandardDeviation, double pointPositionZMeasurementStandardDeviation)
   {
      this.pointVelocityXYMeasurementStandardDeviation = pointVelocityXYMeasurementStandardDeviation;
      this.pointVelocityZMeasurementStandardDeviation = pointVelocityZMeasurementStandardDeviation;

      this.pointPositionXYMeasurementStandardDeviation = pointPositionXYMeasurementStandardDeviation;
      this.pointPositionZMeasurementStandardDeviation = pointPositionZMeasurementStandardDeviation;
   }

   public double getPointVelocityXYMeasurementStandardDeviation()
   {
      return pointVelocityXYMeasurementStandardDeviation;
   }

   public double getPointVelocityZMeasurementStandardDeviation()
   {
      return pointVelocityZMeasurementStandardDeviation;
   }

   public double getPointPositionXYMeasurementStandardDeviation()
   {
      return pointPositionXYMeasurementStandardDeviation;
   }

   public double getPointPositionZMeasurementStandardDeviation()
   {
      return pointPositionZMeasurementStandardDeviation;
   }
}
