package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.net.URI;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.PPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;

import com.jme3.math.Transform;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;

public interface DRCRobotModel
{
   //TODO: RobotBoundingBoxes.java

   public boolean isRunningOnRealRobot();
   
   public ArmControllerParameters getArmControllerParameters();

   public WalkingControllerParameters getWalkingControlParameters();

   public WalkingControllerParameters getMultiContactControllerParameters();
   
   public WalkingControllerParameters getDrivingControllerParameters();
   
   public StateEstimatorParameters getStateEstimatorParameters();
   
   public DRCRobotContactPointParameters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly);

   public DRCRobotPhysicalProperties getPhysicalProperties();

   public DRCRobotJointMap getJointMap();
   
   public DRCRobotSensorInformation getSensorInformation();

   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw);

   public ScsCollisionConfigure getPhysicsConfigure( SDFRobot robotModel );

   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter drcOutputWriter, boolean runningOnRealRobot);
   
   public void setJointDamping(SDFRobot simulatedRobot);
   
   public HandModel getHandModel();
   
   public Transform getOffsetHandFromWrist(RobotSide side);

   public SDFFullRobotModel createFullRobotModel();
   
   public SDFRobot createSdfRobot(boolean createCollisionMeshes);
   
   public double getSimulateDT();
   
   public double getEstimatorDT();
   
   public double getControllerDT();

   public GeneralizedSDFRobotModel getGeneralizedRobotModel();

   public PPSTimestampOffsetProvider getPPSTimestampOffsetProvider();

   public DRCSensorSuiteManager getSensorSuiteManager(URI rosCoreURI);
   
}
