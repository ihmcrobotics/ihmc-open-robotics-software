package us.ihmc.valkyrieRosControl.dataHolders;

import java.util.HashMap;

import us.ihmc.rosControl.wholeRobot.IMUHandle;
import us.ihmc.valkyrie.imu.MicroStrainData;

/**
 * Created by dstephen on 12/14/15.
 */
public class SwitchableFilterModeIMUHandle implements IMUHandle
{
   private MicroStrainData.MicrostrainFilterType filterTypeToUse = MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER;

   private final HashMap<MicroStrainData.MicrostrainFilterType, IMUHandle> imuHandles = new HashMap<>();
   private final String name;

   public SwitchableFilterModeIMUHandle(String name, IMUHandle complimentaryFilterHandle, IMUHandle extendedKalmanFilterHandle)
   {
      this.name = name;
      this.imuHandles.put(MicroStrainData.MicrostrainFilterType.COMPLIMENTARY_FILTER, complimentaryFilterHandle);
      this.imuHandles.put(MicroStrainData.MicrostrainFilterType.ADAPTIVE_EKF, extendedKalmanFilterHandle);
   }

   public MicroStrainData.MicrostrainFilterType getFilterTypeToUse()
   {
      return filterTypeToUse;
   }

   public void setFilterTypeToUse(MicroStrainData.MicrostrainFilterType filterTypeToUse)
   {
      this.filterTypeToUse = filterTypeToUse;
   }

   @Override
   public void getLinearAccelerationCovariance(double[] linearAccelerationCovariance)
   {
      imuHandles.get(filterTypeToUse).getLinearAccelerationCovariance(linearAccelerationCovariance);
   }

   @Override
   public double getZdd()
   {
      return imuHandles.get(filterTypeToUse).getZdd();
   }

   @Override
   public double getYdd()
   {
      return imuHandles.get(filterTypeToUse).getYdd();
   }

   @Override
   public double getXdd()
   {
      return imuHandles.get(filterTypeToUse).getXdd();
   }

   @Override
   public void getAngularVelocityCovariance(double[] angularVelocityCovariance)
   {
      imuHandles.get(filterTypeToUse).getAngularVelocityCovariance(angularVelocityCovariance);
   }

   @Override
   public double getTheta_z()
   {
      return imuHandles.get(filterTypeToUse).getTheta_z();
   }

   @Override
   public double getTheta_y()
   {
      return imuHandles.get(filterTypeToUse).getTheta_y();
   }

   @Override
   public double getTheta_x()
   {
      return imuHandles.get(filterTypeToUse).getTheta_x();
   }

   @Override
   public void getOrientationCovariance(double[] orientationCovariance)
   {
      imuHandles.get(filterTypeToUse).getOrientationCovariance(orientationCovariance);
   }

   @Override
   public double getQ_w()
   {
      return imuHandles.get(filterTypeToUse).getQ_w();
   }

   @Override
   public double getQ_z()
   {
      return imuHandles.get(filterTypeToUse).getQ_z();
   }

   @Override
   public double getQ_y()
   {
      return imuHandles.get(filterTypeToUse).getQ_y();
   }

   @Override
   public double getQ_x()
   {
      return imuHandles.get(filterTypeToUse).getQ_x();
   }

   @Override
   public String getName()
   {
      return this.name;
   }
}
