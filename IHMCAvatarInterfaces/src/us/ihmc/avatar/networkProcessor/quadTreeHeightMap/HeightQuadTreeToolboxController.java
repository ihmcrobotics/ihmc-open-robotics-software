package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.List;

import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.HeightQuadTreeToolboxRequestCommand;
import us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command.PointCloud3DCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.tools.io.printing.PrintTools;

public class HeightQuadTreeToolboxController extends ToolboxController
{
   private static final boolean DEBUG = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double QUAD_TREE_EXTENT = 200;

   private final QuadTreeForGroundHeightMap quadTree;

   private float quadtreeHeightThreshold = 0.02f;
   private float quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2f;
   private int maxSameHeightPointsPerNode = 4;
   private double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
   private int maximumNumberOfPoints = maxSameHeightPointsPerNode * 75000;

   private final CommandInputManager commandInputManager;

   public HeightQuadTreeToolboxController(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.commandInputManager = commandInputManager;

      Box bounds = new Box(-QUAD_TREE_EXTENT, -QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, QUAD_TREE_EXTENT);
      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(DepthDataFilterParameters.GRID_RESOLUTION,
            quadtreeHeightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
            maxAllowableXYDistanceForAPointToBeConsideredClose, maximumNumberOfPoints);

      quadTree = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);
   }
   
   @Override
   protected boolean initialize()
   {
      return true;
   }

   private final FramePoint scanPoint = new FramePoint();

   @Override
   protected void updateInternal()
   {
      if (commandInputManager.isNewCommandAvailable(HeightQuadTreeToolboxRequestCommand.class))
      {
         HeightQuadTreeToolboxRequestCommand command = commandInputManager.pollNewestCommand(HeightQuadTreeToolboxRequestCommand.class);
         if (command.isClearQuadTreeRequested())
         {
            clearQuadTree();
            commandInputManager.flushAllCommands();
            return;
         }
      }

      if (!commandInputManager.isNewCommandAvailable(PointCloud3DCommand.class))
      {
         return;
      }

      boolean hasQuadTreeChanged = false;

      List<PointCloud3DCommand> newPointClouds = commandInputManager.pollNewCommands(PointCloud3DCommand.class);

      if (DEBUG)
         PrintTools.debug("Received new point cloud. Number of points: " + newPointClouds.get(0).getNumberOfPoints());

      for (int pointCloudIndex = 0; pointCloudIndex < newPointClouds.size(); pointCloudIndex++)
      {
         PointCloud3DCommand pointCloud3D = newPointClouds.get(pointCloudIndex);

         for (int pointIndex = 0; pointIndex < pointCloud3D.getNumberOfPoints(); pointIndex++)
         {
            pointCloud3D.getFramePoint(pointIndex, scanPoint);
            scanPoint.changeFrame(worldFrame);
            double x = scanPoint.getX();
            double y = scanPoint.getY();
            double z = scanPoint.getZ();
            hasQuadTreeChanged |= quadTree.addPoint(x, y, z);
         }
      }

      if (DEBUG)
         PrintTools.debug("Done updating the QuadTree.");

      if (hasQuadTreeChanged)
      {
         if (DEBUG)
            PrintTools.debug("QuadTree has changed, sending packet");
         reportMessage(HeightQuadTreeMessageConverter.convertQuadTreeForGround(quadTree));
      }
   }

   private void clearQuadTree()
   {
      quadTree.clear();
   }

   @Override
   protected boolean isDone()
   {
      return false;
   }
}
