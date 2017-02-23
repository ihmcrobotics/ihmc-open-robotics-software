package us.ihmc.avatar.initialSetup;

import java.util.ArrayList;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.simulationconstructionset.DynamicIntegrationMethod;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCSCSInitialSetup
{
   private static final boolean SHOW_WORLD_COORDINATE_FRAME = false;
   private final double simulateDT;// = 0.0001;    // 0.00005; //
   private int recordFrequency = 50;    // 10;
   private boolean drawGroundProfile = false;

   private int simulationDataBufferSize = 16000;
   private Vector3D gravity = new Vector3D(0.0, 0.0,-9.81);
   private boolean runMultiThreaded = true;
   
//   private SensorNoiseParameters simulatedSensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersZeroNoise();
   private SensorNoiseParameters simulatedSensorNoiseParameters = null; // Same as zero noise, but doesn't create sensor corruptors

   private boolean usePerfectSensors = false;
   private boolean initializeEstimatorToActual = false;
   
//   private final CommonTerrain commonTerrain;
   private final GroundProfile3D groundProfile3D;
   
   private DynamicIntegrationMethod dynamicIntegrationMethod = DynamicIntegrationMethod.EULER_DOUBLE_STEPS;
   
   public DRCSCSInitialSetup(GroundProfile3D groundProfile, double simulateDT)
   {      
      this.groundProfile3D = groundProfile;
      this.simulateDT = simulateDT;
   }

   public DRCSCSInitialSetup(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, double simulateDT)
   {
      TerrainObject3D terrainObject3D = commonAvatarEnvironmentInterface.getTerrainObject3D();

      this.groundProfile3D = terrainObject3D;
      this.simulateDT = simulateDT;
   }

   
   public void setRunMultiThreaded(boolean runMultiThreaded)
   {
      this.runMultiThreaded = runMultiThreaded;
   }
   
   public boolean getRunMultiThreaded()
   {
      return runMultiThreaded;
   }
   
   

   public void initializeRobot(Robot robot, DRCRobotModel robotModel, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      robot.setGravity(gravity);

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, robot.getRobotsYoVariableRegistry());
      robotModel.getContactPointParameters().setupGroundContactModelParameters(groundContactModel);

//      if ((commonTerrain.getSteppingStones() != null) && (yoGraphicsListRegistry != null))
//         commonTerrain.registerSteppingStonesArtifact(yoGraphicsListRegistry);

//      groundContactModel.setGroundProfile(commonTerrain.getGroundProfile());
      if (groundProfile3D != null) groundContactModel.setGroundProfile3D(groundProfile3D);
      
      // TODO: change this to scs.setGroundContactModel(groundContactModel);
      robot.setGroundContactModel(groundContactModel);
   }

   public DynamicIntegrationMethod getDynamicIntegrationMethod()
   {
      return dynamicIntegrationMethod;
   }
   
   public void setDynamicIntegrationMethod(DynamicIntegrationMethod dynamicIntegrationMethod)
   {
      this.dynamicIntegrationMethod = dynamicIntegrationMethod;
   }
   
   public double getDT()
   {
      return simulateDT;
   }

   public int getSimulationDataBufferSize()
   {
      return simulationDataBufferSize;
   }

   private ArrayList<Graphics3DObject> createGroundLinkGraphicsFromGroundProfile(GroundProfile3D groundProfile)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject texturedGroundLinkGraphics = new Graphics3DObject();

      HeightMap heightMap = null;
      
      if (groundProfile != null) heightMap = groundProfile.getHeightMapIfAvailable();
      else if (groundProfile3D != null) heightMap = groundProfile3D.getHeightMapIfAvailable();
      
      texturedGroundLinkGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.DarkGreen());
      ret.add(texturedGroundLinkGraphics);

      return ret;
   }
   
   public void initializeSimulation(SimulationConstructionSet scs)
   {
      scs.setDT(simulateDT, recordFrequency);

      if (drawGroundProfile)
      {
//         boolean drawGroundBelow = false;
         
//         ArrayList<Graphics3DObject> groundLinkGraphics = commonTerrain.createLinkGraphics(drawGroundBelow);
         ArrayList<Graphics3DObject> groundLinkGraphics = createGroundLinkGraphicsFromGroundProfile(groundProfile3D);
         scs.addStaticLinkGraphics(groundLinkGraphics);
      }
      
      if (SHOW_WORLD_COORDINATE_FRAME)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.3);
         scs.addStaticLinkGraphics(linkGraphics);
      }

      /*
       * This makes sure that the initial values of all YoVariables that are added to the scs (i.e. at index 0 of the data buffer)
       * are properly stored in the data buffer
       */
      scs.getDataBuffer().copyValuesThrough();

   }

   public void setSimulationDataBufferSize(int simulationDataBufferSize)
   {
      this.simulationDataBufferSize = simulationDataBufferSize;
   }

   public void setRecordFrequency(int recordFrequency)
   {
      this.recordFrequency = recordFrequency;
   }

   public void setTimePerRecordTick(double timePerRecordTick)
   {
      int recordFrequency = (int) Math.round(timePerRecordTick / simulateDT);
      if (recordFrequency < 1) recordFrequency = 1;
      setRecordFrequency(recordFrequency);
   }

   public int getRecordFrequency()
   {
      return recordFrequency;
   }

   public Vector3D getGravity()
   {
      return gravity;
   }

   public SensorNoiseParameters getSimulatedSensorNoiseParameters()
   {
      return simulatedSensorNoiseParameters;
   }

   public void setSimulatedSensorNoiseParameters(SensorNoiseParameters simulatedSensorNoiseParameters)
   {
      this.simulatedSensorNoiseParameters = simulatedSensorNoiseParameters;
   }

   public boolean getInitializeEstimatorToActual()
   {
      return initializeEstimatorToActual;
   }
   
   public void setInitializeEstimatorToActual(boolean initializeEstimatorToActual)
   {
      this.initializeEstimatorToActual = initializeEstimatorToActual;
   }

   public boolean usePerfectSensors()
   {
      return usePerfectSensors;
   }

   public void setUsePerfectSensors(boolean usePerfectSensors)
   {
      this.usePerfectSensors = usePerfectSensors;
   }

   public GroundProfile3D getGroundProfile3D()
   {
      return groundProfile3D;
   }
   
   public HeightMap getHeightMap()
   {
      HeightMap ret = null;
      
      if (groundProfile3D != null)
      {
         ret = groundProfile3D.getHeightMapIfAvailable();
      }
      
      return ret;
   }

//   public SteppingStones getSteppingStones()
//   {
//      return commonTerrain.getSteppingStones();
//   }
   
   public boolean getDrawGroundProfile()
   {
      return drawGroundProfile;
   }
   
   public void setDrawGroundProfile(boolean drawGroundProfile)
   {
      this.drawGroundProfile = drawGroundProfile;
   }

public void setGravity(Vector3D gravity) 
{
	this.gravity = new Vector3D(gravity);
	
}


}
