package us.ihmc.robotEnvironmentAwareness.simulation;

import java.io.IOException;
import java.util.EnumMap;
import java.util.Random;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.util.RealtimeTools;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class LidarFastSimulation
{
   private enum GroundType
   {
      OBSTACLE_COURSE, FLAT, NOTHING, CINDER_BLOCKS, BLOCK, BLOCKS, BLOCKS2, L_SHAPE
   }

   public static final int POINT_CLOUD_PUBLISHING_PERIOD_MILLSECONDS = 100;
   public static final double DEFAULT_SPIN_VELOCITY = 0.3;
   public static final boolean VISUALIZE_GPU_LIDAR = false;
   private static final GroundType DEFAULT_GROUND = GroundType.OBSTACLE_COURSE;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "lidarScanPublisherNode");

   public LidarFastSimulation() throws IOException
   {
   }

   public void startSimulation() throws IOException
   {
      SimpleLidarRobot robot = new SimpleLidarRobot();
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, RealtimeTools.nextPowerOfTwo(200000));
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      double simDT = 0.0001;
      double controlDT = 0.01;
      scs.setDT(simDT, 10);
      scs.setSimulateDoneCriterion(() -> false);

      Graphics3DAdapter graphics3dAdapter = scs.getGraphics3dAdapter();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SimpleLidarRobotController controller = new SimpleLidarRobotController(robot, controlDT, ros2Node, graphics3dAdapter,
                                                                             yoGraphicsListRegistry);
      robot.setController(controller, (int) (controlDT / simDT));
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      createGroundTypeListener(scs);

      scs.setGroundVisible(false);
      scs.startOnAThread();
      scs.simulate();
   }

   private void createGroundTypeListener(final SimulationConstructionSet scs)
   {
      final YoEnum<GroundType> groundType = new YoEnum<>("GroundType", scs.getRootRegistry(), GroundType.class, false);
      groundType.set(DEFAULT_GROUND);

      final EnumMap<GroundType, Graphics3DObject> environmentsGraphics = new EnumMap<>(GroundType.class);
      environmentsGraphics.put(GroundType.OBSTACLE_COURSE, new DefaultCommonAvatarEnvironment().getTerrainObject3D().getLinkGraphics());
      environmentsGraphics.put(GroundType.FLAT, new FlatGroundEnvironment().getTerrainObject3D().getLinkGraphics());
      environmentsGraphics.put(GroundType.NOTHING, new Graphics3DObject());
      environmentsGraphics.put(GroundType.CINDER_BLOCKS, new CinderBlockFieldEnvironment().getTerrainObject3D().getLinkGraphics());
      environmentsGraphics.put(GroundType.BLOCK, createBlock());
      environmentsGraphics.put(GroundType.BLOCKS, createBlocks());
      environmentsGraphics.put(GroundType.BLOCKS2, createBlocks2());
      environmentsGraphics.put(GroundType.L_SHAPE, createLShapedGround());

      YoVariableChangedListener listener = new YoVariableChangedListener()
      {
         private Graphics3DNode groundGraphicsNode = null;

         @Override
         public void changed(YoVariable v)
         {
            if (groundGraphicsNode != null)
               scs.removeGraphics3dNode(groundGraphicsNode);
            groundGraphicsNode = scs.addStaticLinkGraphics(environmentsGraphics.get(groundType.getEnumValue()));
         }
      };
      groundType.addListener(listener);
      listener.changed(null);
   }

   private Graphics3DObject createLShapedGround()
   {
      CombinedTerrainObject3D terrain = new CombinedTerrainObject3D("LShapedGround");

      terrain.addBox(-0.5, -0.5, 1.0, 0.0, 0.05, YoAppearance.DarkCyan());
      terrain.addBox(1.0, -0.5, 1.5, 1.5, 0.05, YoAppearance.DarkCyan());

      return terrain.getLinkGraphics();
   }

   private Graphics3DObject createBlock()
   {
      CombinedTerrainObject3D blocksTerrain = new CombinedTerrainObject3D("Block");

      AxisAngle axisAngle = new AxisAngle(0.0, 0.0, 1.0, Math.PI / 4.0);
      RigidBodyTransform configuration = new RigidBodyTransform(axisAngle, new Vector3D(0.65, 0.0, 0.5));
      blocksTerrain.addRotatableBox(configuration, 0.5, 0.5, 0.5, YoAppearance.DarkGray());

      Vector3D offset = new Vector3D(0.0, -0.5, 0.25);
      axisAngle.transform(offset);
      RigidBodyTransform configuration2 = new RigidBodyTransform(configuration);
      configuration2.getTranslation().set(configuration2.getM03() + offset.getX(), configuration2.getM13() + offset.getY(), configuration2.getM23() + offset.getZ());
      blocksTerrain.addRotatableBox(configuration2, 0.5, 0.5, 1.0, YoAppearance.DarkCyan());

      offset = new Vector3D(0.5, 0.0, 0.25);
      axisAngle.transform(offset);
      RigidBodyTransform configuration3 = new RigidBodyTransform(configuration);
      configuration3.getTranslation().set(configuration3.getM03() + offset.getX(), configuration3.getM13() + offset.getY(), configuration3.getM23() + offset.getZ());
      blocksTerrain.addRotatableBox(configuration3, 0.5, 0.5, 1.0, YoAppearance.DarkCyan());

      return blocksTerrain.getLinkGraphics();
   }

   private Graphics3DObject createBlocks()
   {
      CombinedTerrainObject3D blocksTerrain = new CombinedTerrainObject3D("Blocks");

      Random random = new Random(453L);
      double lx = 0.5;
      double ly = 0.5;
      double maxHeight = 0.7;
      int numberOfHeights = 8;

      double[] heights = new double[numberOfHeights];
      for (int i = 0; i < heights.length; i++)
         heights[i] = i * maxHeight / (numberOfHeights - 1);

      for (double x = -2.0; x <= 5.0; x += lx)
      {
         for (double y = -4.0; y <= 4.0; y += ly)
         {
            double height = heights[random.nextInt(numberOfHeights)];
            blocksTerrain.addBox(x - lx / 2.0, y - ly / 2.0, x + lx / 2.0, y + ly / 2.0, height, YoAppearance.DarkGrey());
         }
      }

      return blocksTerrain.getLinkGraphics();
   }

   private Graphics3DObject createBlocks2()
   {
      CombinedTerrainObject3D blocksTerrain = new CombinedTerrainObject3D("Blocks");

      double lx = 0.5;
      double ly = 0.5;
      double zOffset = -0.5;
      double maxHeight = 2.0;

      for (double x = 0.0; x <= 5.0; x += lx)
      {
         for (double y = -5.0; y <= 5.0; y += ly)
         {
            double r = Math.sqrt(x * x + y * y) / 5.0;
            double height = r * maxHeight;
            blocksTerrain.addBox(x - lx / 2.0, y - ly / 2.0, x + lx / 2.0, y + ly / 2.0, zOffset, height + zOffset, YoAppearance.DarkGrey());
         }
      }

      return blocksTerrain.getLinkGraphics();
   }

   public static void main(String[] args) throws IOException
   {
      LidarFastSimulation lidarFastSimulation = new LidarFastSimulation();
      lidarFastSimulation.startSimulation();
   }
}
