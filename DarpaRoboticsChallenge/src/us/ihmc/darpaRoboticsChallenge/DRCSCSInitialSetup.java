package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.commonAvatarInterfaces.CommonAvatarEnvironmentInterface;
import us.ihmc.commonWalkingControlModules.terrain.CommonTerrain;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.darpaRoboticsChallenge.initialSetup.ScsInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;

import com.yobotics.simulationconstructionset.DynamicIntegrationMethod;
import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.util.LinearGroundContactModel;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.ground.steppingStones.SteppingStones;

public class DRCSCSInitialSetup implements ScsInitialSetup
{
   private static final boolean SHOW_WORLD_COORDINATE_FRAME = false;
   private final double simulateDT;// = 0.0001;    // 0.00005; //
   private int recordFrequency = 50;    // 10;
   private boolean drawGroundProfile = false;

   private int simulationDataBufferSize = 16000;
   private double gravity = -9.81;
   
   private boolean initializeEstimatorToActual = false;
   private final boolean useSoftGroundContactGains;
   
   private final CommonTerrain commonTerrain;

   private DynamicIntegrationMethod dynamicIntegrationMethod = DynamicIntegrationMethod.EULER_DOUBLE_STEPS;
   
   public DRCSCSInitialSetup(GroundProfile groundProfile, double simulateDT, boolean useSoftGroundContactGains)
   {
      commonTerrain = new CommonTerrain(groundProfile);
      this.simulateDT = simulateDT;
      this.useSoftGroundContactGains = useSoftGroundContactGains;
   }
   
   public DRCSCSInitialSetup(GroundProfile groundProfile, double simulateDT)
   {
      this(groundProfile, simulateDT, false);
   }

   public DRCSCSInitialSetup(TerrainType terrainType, double simulateDT)
   {
      this(CommonTerrain.setUpTerrain(terrainType), simulateDT, false);
   }

   public DRCSCSInitialSetup(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, double simulateDT, boolean useSoftGroundContactGains)
   {
      this(commonAvatarEnvironmentInterface.getTerrainObject(), simulateDT, useSoftGroundContactGains);
   }

   public DRCSCSInitialSetup(CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface, double simulateDT)
   {
      this(commonAvatarEnvironmentInterface, simulateDT, false);
   }

   public void initializeRobot(Robot robot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      robot.setGravity(gravity);

      double groundKz, groundBz, groundKxy, groundBxy;
      
//      groundKz = 500.0;
//      groundBz = 300.0;
//      groundKxy = 50000.0;
//      groundBxy = 2000.0;
      
      if (useSoftGroundContactGains)
      {
         groundKz = 800.0;
         groundBz = 800.0;
         groundKxy = 10000.0;
         groundBxy = 800.0;
      }
      else
      {
         groundKz = 2000.0;
         groundBz = 1500.0;
         groundKxy = 50000.0;
         groundBxy = 2000.0;
      }

//      double alphaStick = 1.0;
//      double alphaSlip = 0.5;
//      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(robot, groundKxy, groundBxy, groundKz, groundBz, alphaSlip,
//                                                                alphaStick, robot.getRobotsYoVariableRegistry());

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, groundKxy, groundBxy, groundKz, groundBz,
            robot.getRobotsYoVariableRegistry());


      if ((commonTerrain.getSteppingStones() != null) && (dynamicGraphicObjectsListRegistry != null))
         commonTerrain.registerSteppingStonesArtifact(dynamicGraphicObjectsListRegistry);

      groundContactModel.setGroundProfile(commonTerrain.getGroundProfile());

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

   public void initializeSimulation(SimulationConstructionSet scs)
   {
      scs.setDT(simulateDT, recordFrequency);

      if (drawGroundProfile)
      {
         boolean drawGroundBelow = false;
         
         ArrayList<Graphics3DObject> groundLinkGraphics = commonTerrain.createLinkGraphics(drawGroundBelow);
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

   public int getRecordFrequency()
   {
      return recordFrequency;
   }

   public double getGravity()
   {
      return gravity;
   }

   public boolean getInitializeEstimatorToActual()
   {
      return initializeEstimatorToActual;
   }
   
   public void setInitializeEstimatorToActual(boolean initializeEstimatorToActual)
   {
      this.initializeEstimatorToActual = initializeEstimatorToActual;
   }
   
   public GroundProfile getGroundProfile()
   {
      return commonTerrain.getGroundProfile();
   }

   public SteppingStones getSteppingStones()
   {
      return commonTerrain.getSteppingStones();
   }
   
   public boolean getDrawGroundProfile()
   {
      return drawGroundProfile;
   }
   
   public void setDrawGroundProfile(boolean drawGroundProfile)
   {
      this.drawGroundProfile = drawGroundProfile;
   }
}
