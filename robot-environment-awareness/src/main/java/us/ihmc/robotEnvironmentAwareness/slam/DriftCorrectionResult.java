package us.ihmc.robotEnvironmentAwareness.slam;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class DriftCorrectionResult
{
   private boolean success;
   private RigidBodyTransform driftCorrectionTransformer = new RigidBodyTransform();

   private double computationTime;

   private int icpIterations;
   private int numberOfSurfels;

   private double initialDistance;
   private double finalDistance;
   
   public void set(DriftCorrectionResult other)
   {
      success = other.success;
      driftCorrectionTransformer.set(other.driftCorrectionTransformer);
      computationTime = other.computationTime;
      icpIterations = other.icpIterations;
      numberOfSurfels = other.numberOfSurfels;
      initialDistance = other.initialDistance;
      finalDistance = other.finalDistance;
   }
   
   public void setDefault()
   {
      setComputationTime(0.0);
      setDriftCorrectionTransformer(new RigidBodyTransform());
      setFinalDistance(0.0);
      setIcpIterations(0);
      setInitialDistance(0.0);
      setNumberOfSurfels(0);
      setSuccess(true);
   }

   public boolean isSuccess()
   {
      return success;
   }

   public void setSuccess(boolean success)
   {
      this.success = success;
   }

   public RigidBodyTransformReadOnly getDriftCorrectionTransformer()
   {
      return driftCorrectionTransformer;
   }

   public void setDriftCorrectionTransformer(RigidBodyTransformReadOnly driftCorrectionTransformer)
   {
      this.driftCorrectionTransformer.set(driftCorrectionTransformer);
   }

   public double getComputationTime()
   {
      return computationTime;
   }

   public void setComputationTime(double computationTime)
   {
      this.computationTime = computationTime;
   }

   public int getIcpIterations()
   {
      return icpIterations;
   }

   public void setIcpIterations(int icpIterations)
   {
      this.icpIterations = icpIterations;
   }

   public int getNumberOfSurfels()
   {
      return numberOfSurfels;
   }

   public void setNumberOfSurfels(int numberOfSurfels)
   {
      this.numberOfSurfels = numberOfSurfels;
   }

   public double getInitialDistance()
   {
      return initialDistance;
   }

   public void setInitialDistance(double initialDistance)
   {
      this.initialDistance = initialDistance;
   }

   public double getFinalDistance()
   {
      return finalDistance;
   }

   public void setFinalDistance(double finalDistance)
   {
      this.finalDistance = finalDistance;
   }
}
