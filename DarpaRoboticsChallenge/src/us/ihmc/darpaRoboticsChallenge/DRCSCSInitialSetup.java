package us.ihmc.darpaRoboticsChallenge;

import javax.media.j3d.BranchGroup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.terrain.CommonTerrain;
import us.ihmc.commonWalkingControlModules.terrain.TerrainType;
import us.ihmc.graphics3DAdapter.graphics.LinkGraphics;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.GroundProfile;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.LinearGroundContactModel;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class DRCSCSInitialSetup
{
   private static final boolean SHOW_WORLD_COORDINATE_FRAME = false;

   private double simulateDT = 0.0001; //0.00005; //
   private int recordFrequency = 10; //10;

   
   private int simulationDataBufferSize = 16000;
   private double gravity = -9.81;
   private final CommonTerrain commonTerrain;

   
   private final YoVariableRegistry groundRegistry = new YoVariableRegistry("Ground");
   private final DoubleYoVariable groundKz = new DoubleYoVariable("groundKz", groundRegistry); // LinearGroundContactModel z spring Constant
   private final DoubleYoVariable groundBz = new DoubleYoVariable("groundBz", groundRegistry); // LinearGroundContactModel z damping Constant
   private final DoubleYoVariable groundKxy = new DoubleYoVariable("groundKxy", groundRegistry); // LinearGroundContactModel x and y spring constant
   private final DoubleYoVariable groundBxy = new DoubleYoVariable("groundBxy", groundRegistry); // LinearGroundContactModel x and y damping constant

   
   public DRCSCSInitialSetup(TerrainType terrainType)
   {     
      System.out.println("terrainType = " + terrainType);
      commonTerrain = new CommonTerrain(terrainType, groundRegistry);
   }
   
//   public DRCRobotInitialSetup(SteppingStonePattern steppingStonePattern)
//   {
//      System.out.println("steppingStonePattern = " + steppingStonePattern);
//      boolean useSteppingStoneGroundModel = false;
//
//      commonTerrain = new CommonTerrain(footPolygon, steppingStonePattern, useSteppingStoneGroundModel, groundRegistry);
//   }
   
   public DRCSCSInitialSetup(GroundProfile groundProfile)
   {
      commonTerrain = new CommonTerrain(groundProfile);
   }
   
   public void initializeRobot(SDFRobot robot, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      robot.setGravity(gravity);

      setGroundParameters();

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, groundKxy, groundBxy, groundKz, groundBz, groundRegistry);

      if (commonTerrain.getSteppingStones() != null && dynamicGraphicObjectsListRegistry != null)
         commonTerrain.registerSteppingStonesArtifact(dynamicGraphicObjectsListRegistry);

      groundContactModel.setGroundProfile(commonTerrain.getGroundProfile());

      boolean drawGround = false;
      if (drawGround)
      {
         boolean drawGroundBelow = false;
         BranchGroup branchGroup = commonTerrain.createBranchGroup(drawGroundBelow);
         robot.addStaticBranchGroup(branchGroup);
      }

      // TODO: change this to scs.setGroundContactModel(groundContactModel);
      robot.setGroundContactModel(groundContactModel);
      robot.getRobotsYoVariableRegistry().addChild(groundRegistry);
   }

   private void setGroundParameters()
   {
      groundKz.set(165.0);
      groundBz.set(55.0);
      groundKxy.set(27500.0);
      groundBxy.set(1100.0);
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
      if (SHOW_WORLD_COORDINATE_FRAME)
      {
         LinkGraphics linkGraphics = new LinkGraphics();
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
}
