package us.ihmc.simulationconstructionset;

import java.io.Serializable;
import java.util.ArrayList;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.physics.CollisionArbiter;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.simple.DoNothingCollisionArbiter;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class Simulation implements YoVariableHolder, Serializable // Runnable,
{
   private static final long serialVersionUID = 8438645717978048239L;

   private Graphics3DAdapter myGraphics;

   private final SimulationSynchronizer simulationSynchronizer;

   // //////////////////////////////////////////////////
   // private double DEFAULT_DT = 0.0004;
   // private int DEFAULT_RECORD_FREQ = 50;

   private double SIMULATION_DT = 0.0004;
   private int RECORD_FREQ = 1;

   private Robot[] robots;

   // private final YoVariable time;
   private Simulator mySimulator;
   private DataBuffer myDataBuffer;

   private YoVariableList myCombinedVarList = new YoVariableList("Combined");

   private ArrayList<SimulationDoneListener> simulateDoneListeners = new ArrayList<SimulationDoneListener>();
   private ArrayList<SimulationDoneCriterion> simulateDoneCriterions;

   // private VarList robVarList, gcVarList; //controllerVarList,
   // private ArrayList<VarList> controllerVarLists = new ArrayList<VarList>();

   public void initPhysics(ScsPhysics physics)
   {
      mySimulator.setCollisions(physics.collisionDetector, physics.collisionArbiter, physics.collisionHandler, physics.visualize);

      for (Robot robot : robots)
      {
         if (physics.collisionConfigure != null)
            physics.collisionConfigure.setup(robot, physics.collisionDetector, physics.collisionHandler);
      }
   }

   public void addForceSensor(WrenchContactPoint sensor)
   {
      mySimulator.addForceSensor(sensor);
   }

   public ArrayList<WrenchContactPoint> getForceSensors()
   {
      return mySimulator.getForceSensors();
   }

   public double getDT()
   {
      if (mySimulator != null)
         return mySimulator.getDT();
      else
         return 1.0;
   }

   public void setDT(double simulateDT, int recordFrequency)
   {
      if (mySimulator != null)
         mySimulator.setDT(simulateDT);

      SIMULATION_DT = simulateDT;
      RECORD_FREQ = recordFrequency;

      // recomputeTiming();
   }

   public void setRecordDT(double recordDT)
   {
      RECORD_FREQ = (int) Math.round(recordDT / mySimulator.getDT());
      if (RECORD_FREQ < 1)
         RECORD_FREQ = 1;

      // recomputeTiming();
   }

   public long getRecordFreq()
   {
      return RECORD_FREQ;
   }

   public void addScript(Script script)
   {
      this.mySimulator.addScript(script);
   }

   @Override
   public ArrayList<YoVariable<?>> getAllVariables()
   {
      return myDataBuffer.getAllVariables();
   }

   @Override
   public YoVariable<?>[] getAllVariablesArray()
   {
      return myDataBuffer.getAllVariablesArray();
   }

   @Override
   public YoVariable<?> getVariable(String varname)
   {
      return myDataBuffer.getVariable(varname);
   }

   @Override
   public boolean hasUniqueVariable(String varname)
   {
      return myDataBuffer.hasUniqueVariable(varname);
   }

   @Override
   public YoVariable<?> getVariable(String nameSpace, String varname)
   {
      return myDataBuffer.getVariable(nameSpace, varname);
   }

   @Override
   public boolean hasUniqueVariable(String nameSpace, String varname)
   {
      return myDataBuffer.hasUniqueVariable(nameSpace, varname);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String nameSpace, String varname)
   {
      return myDataBuffer.getVariables(nameSpace, varname);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String varname)
   {
      return myDataBuffer.getVariables(varname);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(NameSpace nameSpace)
   {
      return myDataBuffer.getVariables(nameSpace);
   }

   public void registerVariable(YoVariable<?> variable)
   {
      throw new RuntimeException("Do not register variables with a Simulation.java!");
   }

   public ArrayList<YoVariable<?>> getVariablesThatContain(String searchString, boolean caseSensitive)
   {
      return myDataBuffer.getVariablesThatContain(searchString, caseSensitive, getAllVariables());
   }

   public ArrayList<YoVariable<?>> getVariablesThatStartWith(String searchString)
   {
      return myDataBuffer.getVariablesThatStartWith(searchString);
   }

   public ArrayList<YoVariable<?>> getVars(String[] varNames, String[] regularExpressions)
   {
      return myDataBuffer.getVars(varNames, regularExpressions);
   }

   // public Simulation(int dataBufferSize)
   // {
   //   this(((Robot) null), dataBufferSize);
   // }

   public Simulation(Robot robot, int dataBufferSize)
   {
      this(new Robot[] { robot }, dataBufferSize);
   }

   public Simulation(Robot[] robots, int dataBufferSize)
   {
      this.simulationSynchronizer = new SimulationSynchronizer();

      // Make sure robots actually has some robots in it
      if ((robots != null) && (robots[0] == null))
         robots = null;

      // Create a data buffer:
      myDataBuffer = new DataBuffer(dataBufferSize);
      myDataBuffer.setWrapBuffer(true);

      if (robots != null)
      {
         for (Robot robot : robots)
         {
            YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
            ArrayList<RewoundListener> simulationRewoundListners = registry.getAllSimulationRewoundListeners();

            myDataBuffer.attachSimulationRewoundListeners(simulationRewoundListners);
         }
      }

      setRobots(robots);
   }

   public void closeAndDispose()
   {
      myDataBuffer.closeAndDispose();

      myDataBuffer = null;
      mySimulator = null;
   }

   public void setRobots(Robot[] robots)
   {
      this.robots = robots;
      mySimulator = new Simulator(simulationSynchronizer, robots, SIMULATION_DT);
      this.setDT(SIMULATION_DT, RECORD_FREQ);

      if (robots != null)
      {
         for (Robot robot : robots)
         {
            addVariablesFromARobot(robot);
         }
      }

      myDataBuffer.copyValuesThrough();
      updateRobots(robots);
   }
   
   public void addRobot(Robot robot)
   {
      Robot[] newRobots;
      if (robots == null)
      {
         newRobots = new Robot[]{robot};
      }
      else
      {
         newRobots = new Robot[robots.length + 1];
         for (int i=0; i<robots.length; i++)
         {
            newRobots[i] = robots[i];
         }
         
         newRobots[newRobots.length - 1] = robot;
      }

      this.robots = newRobots;
      
      if (mySimulator == null)
      {
         mySimulator = new Simulator(simulationSynchronizer, robots, SIMULATION_DT);
      }
      else
      {
         mySimulator.setRobots(robots);
      }
      
      this.setDT(SIMULATION_DT, RECORD_FREQ);
      addVariablesFromARobot(robot);

      myDataBuffer.copyValuesThrough();
      updateRobots(robots);
   }

   private void updateRobots(Robot[] robots)
   {
      if (robots != null)
      {
         for (Robot robot : robots)
         {
            robot.update();
         }
      }
   }

   private void addVariablesFromARobot(Robot robot)
   {
      try
      {
         myDataBuffer.addVariables(robot.getAllVariables());
         myCombinedVarList.addVariables(robot.getAllVariables());
      }
      catch (RepeatDataBufferEntryException ex)
      {
         System.err.println("Found repeat YoVariable in the robot Variables");
         ex.printStackTrace();
         System.exit(-1);
      }
   }

   public DataBuffer getDataBuffer()
   {
      return myDataBuffer;
   }

   public Robot[] getRobots()
   {
      return robots;
   }

   public YoVariableList getCombinedVarList()
   {
      return myCombinedVarList;
   }

   public synchronized void setSimulateDoneCriterion(SimulationDoneCriterion criterion)
   {
      //@TODO: Rename this add, not set.
      if (criterion == null)
         return;

      if (simulateDoneCriterions == null)
         simulateDoneCriterions = new ArrayList<SimulationDoneCriterion>();
      simulateDoneCriterions.add(criterion);
   }

   public synchronized void addSimulateDoneListener(SimulationDoneListener listener)
   {
      simulateDoneListeners.add(listener);
   }

   public synchronized void removeSimulateDoneListener(SimulationDoneListener listener)
   {
      simulateDoneListeners.remove(listener);
   }

   public void notifySimulateDoneListeners()
   {
      for (int i = 0; i < simulateDoneListeners.size(); i++)
      {
         simulateDoneListeners.get(i).simulationDone();
      }
   }

   public void notifySimulateDoneListenersOfException(Throwable throwable)
   {
      for (int i = 0; i < simulateDoneListeners.size(); i++)
      {
         simulateDoneListeners.get(i).simulationDoneWithException(throwable);
      }
   }

   public boolean checkSimulateDoneCriterion()
   {
      if (simulateDoneCriterions == null)
         return false;

      for (int i = 0; i < simulateDoneCriterions.size(); i++)
      {
         if (simulateDoneCriterions.get(i).isSimulationDone())
            return true;
      }

      return false;
   }

   protected void simulate() throws UnreasonableAccelerationException
   {
      mySimulator.simulate();
   }

   public synchronized void simulate(int numTicks) throws UnreasonableAccelerationException
   {
      while (numTicks > 0)
      {
         for (int i = 0; i < RECORD_FREQ; i++)
         {
            mySimulator.simulate();
            {
               if (checkSimulateDoneCriterion())
               {
                  numTicks = -1;
               }
            }
         }

         myDataBuffer.tickAndUpdate();
         numTicks -= RECORD_FREQ;
      }

      // Notify all the listeners that the simulation stopped...
      for (int i = 0; i < simulateDoneListeners.size(); i++)
      {
         (simulateDoneListeners.get(i)).simulationDone();
      }
   }

   protected void doControl()
   {
      mySimulator.doControl();
   }

   protected void doDynamicsAndIntegrate() throws UnreasonableAccelerationException
   {
      mySimulator.doDynamicsAndIntegrate();
   }

   protected void forceClassLoading()
   {
      mySimulator.forceClassLoading();
   }

   public void tickAndUpdate()
   {
      myDataBuffer.tickAndUpdate();
   }

   public synchronized void simulate(double simulationTime) throws UnreasonableAccelerationException
   {
      simulate((int) (simulationTime / mySimulator.getDT()));
   }

   public void setupSimulationGraphics(ArrayList<GraphicsRobot> graphicsRobotsToUpdate)
   {
      // 3D Canvass Stuff goes here...
      // myGraphics = new StandardSimulationGraphics(this.rob, this.myCombinedVarList, null);
      if (this.robots.length > 0)
      {
         GroundContactModel groundContactModel = this.robots[0].getGroundContactModel();
         //         GroundProfile groundProfile = null;
         HeightMap heightMap = null;

         if (groundContactModel != null)
         {
            heightMap = HeightMapFromGroundContactModel.getHeightMap(groundContactModel);
         }
         if (heightMap == null)
            heightMap = new FlatGroundProfile();

         myGraphics = SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true);

         //         HeightMapFromGroundProfile heightMap = new HeightMapFromGroundProfile(groundProfile);
         myGraphics.setHeightMap(heightMap);
         for (GraphicsRobot graphicsRobot : graphicsRobotsToUpdate)
         {
            myGraphics.addRootNode(graphicsRobot.getRootNode());
         }

      }

      // End of 3D Stuff...
   }

   public Graphics3DAdapter getSimulationGraphics()
   {
      return this.myGraphics;
   }

   public void addVarList(YoVariableList newVarList) throws RepeatDataBufferEntryException
   {
      myCombinedVarList.addVariables(newVarList);
      myDataBuffer.addVariables(newVarList.getVariables());
   }

   public SimulationSynchronizer getSimulationSynchronizer()
   {
      return simulationSynchronizer;
   }

   public void initializeCollisionDetectionAndHandling(DefaultCollisionVisualizer collisionVisualizer, CollisionHandler collisionHandler)
   {
      ScsCollisionDetector collisionDetector = createCollisionShapesFromLinks(robots, collisionHandler);

      if (collisionVisualizer != null) collisionHandler.addListener(collisionVisualizer);

      collisionDetector.initialize();
      
      CollisionArbiter collisionArbiter = new DoNothingCollisionArbiter();
//      CollisionArbiter collisionArbiter = new ExperimentalCollisionArbiter();

      this.initPhysics(new ScsPhysics(null, collisionDetector, collisionArbiter, collisionHandler, collisionVisualizer));
   }

   private static ScsCollisionDetector createCollisionShapesFromLinks(Robot[] robots, CollisionHandler collisionHandler)
   {
      ScsCollisionDetector collisionDetector;

      //      collisionDetector = new GdxCollisionDetector(100.0);
      collisionDetector = new SimpleCollisionDetector();
//      ((SimpleCollisionDetector) collisionDetector).setUseSimpleSpeedupMethod();

      CollisionShapeFactory collisionShapeFactory = collisionDetector.getShapeFactory();
      collisionShapeFactory.setMargin(0.002);

      for (Robot robot : robots)
      {
         createCollisionShapesFromLinks(robot, collisionShapeFactory, collisionHandler, robot.getRobotsYoVariableRegistry());
      }

      return collisionDetector;
   }

   private static void createCollisionShapesFromLinks(Robot robot, CollisionShapeFactory collisionShapeFactory, CollisionHandler collisionHandler, YoVariableRegistry registry)
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      for (Joint rootJoint : rootJoints)
      {
         createCollisionShapesFromLinksRecursively(rootJoint, collisionShapeFactory, collisionHandler, registry);
      }
   }

   private static void createCollisionShapesFromLinksRecursively(Joint joint, CollisionShapeFactory collisionShapeFactory, CollisionHandler collisionHandler, YoVariableRegistry registry)
   {
      Link link = joint.getLink();
      ArrayList<CollisionMeshDescription> collisionMeshDescriptions = link.getCollisionMeshDescriptions();

      if (collisionMeshDescriptions != null)
      {
         int estimatedNumberOfContactPoints = 0;

         for (int i=0; i<collisionMeshDescriptions.size(); i++)
         {
            CollisionMeshDescription collisionMesh = collisionMeshDescriptions.get(i);
            collisionShapeFactory.addCollisionMeshDescription(link, collisionMesh);
            estimatedNumberOfContactPoints += collisionMesh.getEstimatedNumberOfContactPoints();
         }

         link.enableCollisions(estimatedNumberOfContactPoints, collisionHandler, registry);
      }

      ArrayList<Joint> childrenJoints = joint.getChildrenJoints();
      for (Joint childJoint : childrenJoints)
      {
         createCollisionShapesFromLinksRecursively(childJoint, collisionShapeFactory, collisionHandler, registry);
      }
   }

}
