package us.ihmc.valkyrie;

import java.io.InputStream;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactPointInformation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCContactPointInformationFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandType;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.valkyrie.io.ValkyrieOutputWriterWithAccelerationIntegration;
import us.ihmc.valkyrie.models.ModelRoot;
import us.ihmc.valkyrie.paramaters.ValkyrieArmControllerParameters;
import us.ihmc.valkyrie.paramaters.ValkyrieContactPointParamaters;
import us.ihmc.valkyrie.paramaters.ValkyrieJointMap;
import us.ihmc.valkyrie.paramaters.ValkyriePhysicalProperties;
import us.ihmc.valkyrie.paramaters.ValkyrieStateEstimatorParameters;
import us.ihmc.valkyrie.paramaters.ValkyrieWalkingControllerParameters;

import com.jme3.math.Transform;

public class ValkyrieRobotModel implements DRCRobotModel
{
   private static Class<ModelRoot> valModelRoot = ModelRoot.class;
   private static String[] resourceDirectories;
   private final ArmControllerParameters armControllerParameters;
   private final DRCRobotPhysicalProperties physicalProperties;
   private DRCRobotJointMap jointMap;
   private final String robotName = "VALKYRIE";
         private final String modelName = "V1";
   private StateEstimatorParameters stateEstimatorParamaters;
   private double estimatorDT;
   

   private final boolean runningOnRealRobot;
   
   public ValkyrieRobotModel()
   {
      this(false);
   }
   
   public ValkyrieRobotModel(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.armControllerParameters = new ValkyrieArmControllerParameters(runningOnRealRobot);
      this.physicalProperties = new ValkyriePhysicalProperties();
//      this.jointMap = new ValkyrieJointMap(); 
   }
   
   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return armControllerParameters;
   }

   /**
    * Returns a new instance of the WalkingControllerParameters as it is mutable
    */
   @Override
   public WalkingControllerParameters getWalkingControlParameters()
   {
      return new ValkyrieWalkingControllerParameters(runningOnRealRobot);
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters(double estimatorDT)
   {
      if(stateEstimatorParamaters == null || this.estimatorDT != estimatorDT)
      {
         this.estimatorDT = estimatorDT;
         stateEstimatorParamaters = new ValkyrieStateEstimatorParameters(runningOnRealRobot, estimatorDT);
      }
      return stateEstimatorParamaters;
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return physicalProperties;
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      if(jointMap == null)
      {
         jointMap = new ValkyrieJointMap(); 
      }
      return jointMap;
   }

   @Override
   public boolean hasIRobotHands()
   {
      return false;
   }

   @Override
   public boolean hasArmExtensions()
   {
      return false;
   }

   @Override
   public boolean hasHookHands()
   {
      return false;
   }

   @Override
   public DRCHandType getHandType()
   {
      return DRCHandType.NONE;
   }

   @Override
   public String getModelName()
   {
      return modelName;
   }

   @Override
   public Transform getOffsetHandFromWrist(RobotSide side)
   {
      return new Transform();
   }

   public String getSdfFile()
   {
      return "V1/sdf/V1_sim.sdf";
   }

   public String[] getResourceDirectories()
   {

      if (resourceDirectories == null)
      {
         resourceDirectories = new String[] {
               valModelRoot.getResource("").getFile(),
               valModelRoot.getResource("V1/").getFile(),
               valModelRoot.getResource("V1/meshes/").getFile(),
               valModelRoot.getResource("V1/meshes/2013_05_16/").getFile(),
               };
      }
      return resourceDirectories;
   }

   public InputStream getSdfFileAsStream()
   {
      return valModelRoot.getResourceAsStream(getSdfFile());
   }

   public String toString()
   {
      return robotName;
   }

   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new ValkyrieInitialSetup(groundHeight, initialYaw);
   }

   /**
    * Returns a new instance of the WalkingControllerParameters as it is mutable
    */
   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return new ValkyrieWalkingControllerParameters();
   }

   @Override
   public DRCRobotContactPointParamaters getContactPointParamaters(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return new ValkyrieContactPointParamaters(getJointMap());
   }

   @Override
   public ContactPointInformation getContactPointInformation(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      return DRCContactPointInformationFactory.createContactPointInformation(getJointMap(),getContactPointParamaters(addLoadsOfContactPoints, addLoadsOfContactPointsToFeetOnly)) ;
   }

   @Override
   public DRCOutputWriter getOutputWriterWithAccelerationIntegration(DRCOutputWriter valkyrieOutputWriter, double controlDT, boolean runningOnRealRobot)
   {
      ValkyrieOutputWriterWithAccelerationIntegration valkyrieOutputWriterWithAccelerationIntegration =
            new ValkyrieOutputWriterWithAccelerationIntegration(valkyrieOutputWriter, controlDT, runningOnRealRobot);

      valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredVelocity(0.98);
      valkyrieOutputWriterWithAccelerationIntegration.setAlphaDesiredPosition(0.0);
      valkyrieOutputWriterWithAccelerationIntegration.setVelocityGains(15.0);
      valkyrieOutputWriterWithAccelerationIntegration.setPositionGains(0.0);


      return valkyrieOutputWriterWithAccelerationIntegration;
   }

   //For Sim Only
   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for Valkyrie. ValkyrieRobotModel setJointDamping!");
   }

   @Override
   public HandModel getHandModel()
   {
	   return null;
   }

}
