package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.terrain.CommonTerrain;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.initialSetup.ScsInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.HeightMap;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

import com.yobotics.simulationconstructionset.DynamicIntegrationMethod;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.physics.CollisionHandler;
import com.yobotics.simulationconstructionset.physics.ScsCollisionConfigure;
import com.yobotics.simulationconstructionset.physics.ScsCollisionDetector;
import com.yobotics.simulationconstructionset.physics.ScsPhysics;
import com.yobotics.simulationconstructionset.physics.collision.SpringCollisionHandler;
import com.yobotics.simulationconstructionset.physics.collision.bullet.JBulletCollisionDetector;
import com.yobotics.simulationconstructionset.physics.visualize.DefaultCollisionVisualize;
import com.yobotics.simulationconstructionset.util.LinearGroundContactModel;
import com.yobotics.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCSCSInitialSetup implements ScsInitialSetup
{
   private static final boolean SHOW_WORLD_COORDINATE_FRAME = false;
   private final double simulateDT;// = 0.0001;    // 0.00005; //
   private int recordFrequency = 50;    // 10;
   private boolean drawGroundProfile = false;

   private int simulationDataBufferSize = 16000;
   private double gravity = -9.81;
   
//   private SensorNoiseParameters simulatedSensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createSensorNoiseParametersZeroNoise();
   private SensorNoiseParameters simulatedSensorNoiseParameters = null; // Same as zero noise, but doesn't create sensor corruptors
   private boolean initializeEstimatorToActual = false;
   
//   private final CommonTerrain commonTerrain;
   private final GroundProfile3D groundProfile3D;
   
   private DynamicIntegrationMethod dynamicIntegrationMethod = DynamicIntegrationMethod.EULER_DOUBLE_STEPS;
   
   public DRCSCSInitialSetup(GroundProfile3D groundProfile, double simulateDT)
   {      
      this.groundProfile3D = groundProfile;
      this.simulateDT = simulateDT;
   }

   public DRCSCSInitialSetup(TerrainType terrainType, double simulateDT)
   {
      this(CommonTerrain.setUpTerrain(terrainType), simulateDT);
   }

   public DRCSCSInitialSetup(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, double simulateDT)
   {
      TerrainObject3D terrainObject3D = commonAvatarEnvironmentInterface.getTerrainObject3D();

      this.groundProfile3D = terrainObject3D;
      this.simulateDT = simulateDT;
   }

   public ScsPhysics createPhysics(ScsCollisionConfigure collisionConfigure, YoVariableRegistry registry)
   {
      ScsCollisionDetector collision = new JBulletCollisionDetector(registry,10000);
      CollisionHandler handler = new SpringCollisionHandler(1,1000,10.0,registry);
      collision.initialize(handler);

      DefaultCollisionVisualize visualize = new DefaultCollisionVisualize();

      return new ScsPhysics(collisionConfigure,collision,visualize);
   }

   public void initializeRobot(Robot robot, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      robot.setGravity(gravity);

      double groundKz, groundBz, groundKxy, groundBxy;

      groundKz = 2000.0;
      groundBz = 1500.0;
      groundKxy = 50000.0;
      groundBxy = 2000.0;

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, groundKxy, groundBxy, groundKz, groundBz,
            robot.getRobotsYoVariableRegistry());

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

   public double getGravity()
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
}
