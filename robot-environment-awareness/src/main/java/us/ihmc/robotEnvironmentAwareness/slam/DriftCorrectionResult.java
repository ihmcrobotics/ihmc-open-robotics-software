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
   private int numberOfCorrespondances;

   private double initialDistance;
   private double finalDistance;
   
   public void set(DriftCorrectionResult other)
   {
      success = other.success;
      driftCorrectionTransformer.set(other.driftCorrectionTransformer);

      computationTime = other.computationTime;

      icpIterations = other.icpIterations;
      numberOfSurfels = other.numberOfSurfels;
      numberOfCorrespondances = other.numberOfCorrespondances;

      initialDistance = other.initialDistance;
      finalDistance = other.finalDistance;
   }
   
   public void setDefault()
   {
      setSuccess(true);
      setDriftCorrectionTransformer(new RigidBodyTransform());

      setComputationTime(0.0);

      setIcpIterations(0);
      setNumberOfSurfels(0);
      setNumberOfCorrespondances(0);

      setInitialDistance(0.0);
      setFinalDistance(0.0);
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

   public int getNumberOfCorrespondances()
   {
      return numberOfCorrespondances;
   }

   public void setNumberOfCorrespondances(int numberOfCorrespondances)
   {
      this.numberOfCorrespondances = numberOfCorrespondances;
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
