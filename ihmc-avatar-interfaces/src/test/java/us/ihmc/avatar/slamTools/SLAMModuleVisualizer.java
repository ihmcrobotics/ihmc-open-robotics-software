package us.ihmc.avatar.slamTools;

import java.io.File;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SLAMModuleVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime;
   private final double dt = 1.0;
   private final int recordFrequency = 1;
   private final int bufferSize;

   private static final String DATA_PATH = "C:\\PointCloudData\\Data\\20200601_LidarWalking_ComplexArea\\PointCloud\\";
   private static final int NUMBER_OF_POINTS_TO_VISUALIZE = 500;

   private final static double OCTREE_RESOLUTION = 0.02;

   private final SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(OCTREE_RESOLUTION);

   public SLAMModuleVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      File pointCloudFile = new File(DATA_PATH);
      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      int numberOfFrames = messages.size();
      slam.addKeyFrame(messages.get(0));
      slam.updatePlanarRegionsMap();

      SLAMFrameYoGraphicsManager[] frameGraphicsManagers = new SLAMFrameYoGraphicsManager[numberOfFrames - 1];
      for (int i = 1; i < numberOfFrames; i++)
      {
         System.out.println();
         System.out.println(" ## add frame " + i);
         slam.addFrame(messages.get(i));
         slam.updatePlanarRegionsMap();

         frameGraphicsManagers[i - 1] = new SLAMFrameYoGraphicsManager("Frame_"
               + (i - 1), slam.getLatestFrame(), NUMBER_OF_POINTS_TO_VISUALIZE, YoAppearance.randomColor(new Random()), registry, yoGraphicsListRegistry);
      }

      trajectoryTime = (int) numberOfFrames;
      bufferSize = (int) (trajectoryTime / dt / recordFrequency + 3);
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setGroundVisible(false);

      for (double t = 0.0; t <= trajectoryTime + dt; t += dt)
      {
         robot.getYoTime().set(t);

         // update viz.
         for (int i = 0; i < frameGraphicsManagers.length - 1; i++)
         {
            if (i != (int) t)
            {
               frameGraphicsManagers[i].hide();
            }
            else
            {
               if (i > 0)
                  frameGraphicsManagers[i - 1].updateGraphics();
               frameGraphicsManagers[i].updateGraphics();
            }
         }

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new SLAMModuleVisualizer();
   }
}
