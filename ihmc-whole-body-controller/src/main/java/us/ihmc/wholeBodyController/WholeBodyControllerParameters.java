package us.ihmc.wholeBodyController;

import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

import java.io.InputStream;

public interface WholeBodyControllerParameters<E extends Enum<E> & RobotSegment<E>>
{
   double getControllerDT();

   default double getSimulatedHandControlDT()
   {
      return getControllerDT();
   }

   default double getFeedbackControllerDT()
   {
      return 0.0;
   }

   StateEstimatorParameters getStateEstimatorParameters();

   CoPTrajectoryParameters getCoPTrajectoryParameters();

   default SplitFractionCalculatorParameters getSplitFractionCalculatorParameters()
   {
      return new SplitFractionCalculatorParameters();
   }

   WalkingControllerParameters getWalkingControllerParameters();

   RobotContactPointParameters<E> getContactPointParameters();

   HumanoidRobotSensorInformation getSensorInformation();

   /**
    * Get the parameter XML file for the controller.
    * <p>
    * Each call to this method should return a new InputStream.
    * If null is returned the default values for the parameters are used.
    * </p>
    *
    * @return new InputStream with the controller parameters
    */
   InputStream getWholeBodyControllerParametersFile();

   /**
    * Allows to overwrite parameters specified in the {@link #getWholeBodyControllerParametersFile()}
    * XML file. This is useful in unit tests when parameters need to be modified.
    *
    * @return InputStream with the parameters that need to be overwritten.
    */
   default InputStream getParameterOverwrites()
   {
      return null;
   }

   default String getParameterFileName()
   {
      return "not implemented";
   }
}

