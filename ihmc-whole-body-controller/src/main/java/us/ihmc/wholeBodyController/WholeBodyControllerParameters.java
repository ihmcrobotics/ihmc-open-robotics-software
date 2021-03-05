package us.ihmc.wholeBodyController;

import java.io.InputStream;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

public interface WholeBodyControllerParameters<E extends Enum<E> & RobotSegment<E>>
{
   public double getControllerDT();

   public StateEstimatorParameters getStateEstimatorParameters();

   public CoPTrajectoryParameters getCoPTrajectoryParameters();

	public WalkingControllerParameters getWalkingControllerParameters();

	public RobotContactPointParameters<E> getContactPointParameters();

   public HumanoidRobotSensorInformation getSensorInformation();


   /**
    * Get the parameter XML file for the controller.
    *
    * Each call to this method should return a new InputStream.
    * If null is returned the default values for the parameters are used.
    *
    * @return new InputStream with the controller parameters
    */
   public InputStream getWholeBodyControllerParametersFile();

   /**
    * Allows to overwrite parameters specified in the {@link #getWholeBodyControllerParametersFile()}
    * XML file. This is useful in unit tests when parameters need to be modified.
    *
    * @return InputStream with the parameters that need to be overwritten.
    */
   public default InputStream getParameterOverwrites()
   {
      return null;
   }
   
   public default String getParameterFileName()
   {
      return "not implemented";
   }
}

