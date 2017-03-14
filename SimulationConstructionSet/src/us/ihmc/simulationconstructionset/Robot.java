package us.ihmc.simulationconstructionset;

import java.io.PrintStream;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraMountInterface;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraMountList;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.robotdefinition.ExternalForcePointDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.robotdefinition.GroundContactDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.robotdefinition.JointDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.robotdefinition.JointDefinitionFixedFrame.JointType;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

/**
 * <p>Title: Robot</p>
 *
 * <p>Description: A Robot is a forest of trees of Joints, each Joint having an associated Link.
 * The Robot contains all the dynamic information, including Joint types, offsets between Joints,
 * Link masses, center of mass locations, and moments of inertia.  Each root joint has children
 * </p>
 *
 * <p>Copyright: Copyright (c) 2000-2005</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 * @author Jerry Pratt
 * @version 1.0
 */
public class Robot implements YoVariableHolder, GroundContactPointsHolder
{
   protected YoVariableRegistry yoVariableRegistry;
   protected final ArrayList<YoGraphicsListRegistry> yoGraphicsListRegistries = new ArrayList<YoGraphicsListRegistry>();
   private ArrayList<Joint> rootJoints;

   private FunctionIntegrators functionIntegrators = null;

   private final String name;

   protected DoubleYoVariable t;

   // The gravitational constants for each axis
   public DoubleYoVariable gravityX;
   public DoubleYoVariable gravityY;
   public DoubleYoVariable gravityZ;

// protected double gX = 0.0, gY = 0.0, gZ = -9.81;

   private ArrayList<RobotControllerAndParameters> controllers = new ArrayList<RobotControllerAndParameters>();

   private GroundContactModel groundContactModel;

   private ExternalForcePoint kp_body;

   private DynamicIntegrationMethod dynamicIntegrationMethod = DynamicIntegrationMethod.RUNGE_KUTTA_FOURTH_ORDER;
   
   private final ArrayList<Graphics3DObject> staticLinkGraphics = new ArrayList<Graphics3DObject>();
   // private VarList robVars;
// private VarList groundVars;
// private ArrayList<VarList> controllerVarLists = new ArrayList<VarList>();

   public Robot(RobotDefinitionFixedFrame definition, String name)
   {
      this(name);
      constructRobotFromDefinition(definition);
   }

   private void constructRobotFromDefinition(RobotDefinitionFixedFrame definition)
   {
      for (JointDefinitionFixedFrame currentRootJoint : definition.getRootJointDefinitions())
      {
         traverseJointDefinitions(currentRootJoint, null);
      }
   }

   private void traverseJointDefinitions(JointDefinitionFixedFrame jointDefinition, Joint parent)
   {
      Joint currentJoint = null;
      if (jointDefinition.getType() == JointType.FLOATING_JOINT)
      {
         currentJoint = new FloatingJoint(jointDefinition.getJointName(), jointDefinition.getOffset(), this);
      }

      else if (jointDefinition.getType() == JointType.FLOATING_PLANAR_JOINT)
      {
         currentJoint = new FloatingPlanarJoint(jointDefinition.getJointName(), this, jointDefinition.getPlanarType());

      }
      else if (jointDefinition.getType() == JointType.PIN_JOINT)
      {
         currentJoint = new PinJoint(jointDefinition.getJointName(), jointDefinition.getOffset(), this, jointDefinition.getJointAxis());
      }
      else if (jointDefinition.getType() == JointType.SLIDER_JOINT)
      {
         currentJoint = new SliderJoint(jointDefinition.getJointName(), jointDefinition.getOffset(), this, jointDefinition.getJointAxis());
      }

      for (GroundContactDefinitionFixedFrame groundContactDefinitionFixedFrame : jointDefinition.getGroundContactDefinitionsFixedFrame())
      {
         GroundContactPoint groundContactPoint = new GroundContactPoint(groundContactDefinitionFixedFrame.getName(),
                                                    groundContactDefinitionFixedFrame.getOffset(), this.getRobotsYoVariableRegistry());
         currentJoint.addGroundContactPoint(groundContactPoint);
      }

      for (ExternalForcePointDefinitionFixedFrame externalForcePointDefinitionFixedFrame : jointDefinition.getExternalForcePointDefinitionsFixedFrame())
      {
         ExternalForcePoint externalForcePoint = new ExternalForcePoint(externalForcePointDefinitionFixedFrame.getName(),
                                                    externalForcePointDefinitionFixedFrame.getOffset(), this.getRobotsYoVariableRegistry());
         currentJoint.addExternalForcePoint(externalForcePoint);
      }

      currentJoint.setLink(new Link(jointDefinition.getLinkDefinition()));
      if (parent == null)
         this.addRootJoint(currentJoint);
      else
         parent.addJoint(currentJoint);

      for (JointDefinitionFixedFrame childJoint : jointDefinition.getChildrenJoints())
      {
         traverseJointDefinitions(childJoint, currentJoint);
      }
   }

   /**
    * Creates a Robot with the specified name. A Robot is a forest of trees of
    * Joints, each Joint having an associated Link.
    *
    * @param name Name of the robot.
    */
   public Robot(String name)
   {
      this.name = name;
      yoVariableRegistry = new YoVariableRegistry(name);

      this.rootJoints = new ArrayList<Joint>();

      t = new DoubleYoVariable("t", yoVariableRegistry);
      gravityX = new DoubleYoVariable("gravityX", yoVariableRegistry);
      gravityY = new DoubleYoVariable("gravityY", yoVariableRegistry);
      gravityZ = new DoubleYoVariable("gravityZ", yoVariableRegistry);

      setDefaultGravityToEarthWithMetricUnits();
   }

   public void setDynamicIntegrationMethod(DynamicIntegrationMethod dynamicIntegrationMethod)
   {
      this.dynamicIntegrationMethod = dynamicIntegrationMethod;
   }
   
   private void setDefaultGravityToEarthWithMetricUnits()
   {
      gravityZ.set(-9.81);
   }

   /**
    * Gets this robot's time.
    *
    * @return double
    */
   public double getTime()
   {
      return t.getDoubleValue();
   }

   /**
    * Sets this robot's time.
    */
   public void setTime(double time)
   {
      t.set(time);
   }

   public void setDynamic(boolean isDynamic)
   {
      for (Joint joint : rootJoints)
      {
         joint.setDynamic(isDynamic);
      }
   }

   /**
    * Gets this robot's time
    * @return YoVariable
    */
   public DoubleYoVariable getYoTime()
   {
      return t;
   }

   /**
    * Gets this robot's component of gravitational acceleration in the x-direction.
    *
    * @return double
    */
   public double getGravityX()
   {
      return gravityX.getDoubleValue();
   }

   /**
    * Gets this robot's component of gravitational acceleration in the y-direction.
    *
    * @return double
    */
   public double getGravityY()
   {
      return gravityY.getDoubleValue();
   }

   /**
    * Gets this robot's component of gravitational acceleration in the z-direction.
    *
    * @return double
    */
   public double getGravityZ()
   {
      return gravityZ.getDoubleValue();
   }

   public void getGravity(Vector3D gravityVectorToPack)
   {
      gravityVectorToPack.set(gravityX.getDoubleValue(), gravityY.getDoubleValue(), gravityZ.getDoubleValue());
   }

   /**
    * Adds a YoVariableRegistry to the robot. Will be added as a child registry to the robot's registry.
    *
    * @param registry YoVariableRegistry
    */
   public void addYoVariableRegistry(YoVariableRegistry registry)
   {
      if (registry == null)
         throw new RuntimeException("Cannot add a null registry to " + this.name + "!!!!");

      getRobotsYoVariableRegistry().addChild(registry);
   }

   public void addYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry == null)
      {
         throw new RuntimeException("Cannot add a null yoGraphicsListRegistry to " + this.name + "!!!!");
      }

      yoGraphicsListRegistries.add(yoGraphicsListRegistry);
   }


   /**
    * Adds a root Joint to this robot.  This joint may have multiple child joints which also can have children.  A robot
    * may have any number of root joints.
    *
    * @param root Joint to be added as the root Joint. A robot is comprised of a forest of trees, with one root Joint per tree.
    */
   public void addRootJoint(Joint root)
   {
      this.rootJoints.add(root);
   }


   /**
    * Retrieves this robot's ground contact model.
    *
    * @return GroundContactModel The GroundContactModel of this robot.
    * @see GroundContactModel GroundContactModel
    */
   public GroundContactModel getGroundContactModel()
   {
      return this.groundContactModel;
   }

   /**
    * Step through each joint to determine if any points are in contact with the ground.
    */
   public void decideGroundContactPointsInContact()
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.physics.recursiveDecideGroundContactPointsInContact();
      }
   }


   /**
    * Retrieves an ArrayList containing the rootJoints of this robot.  These joints make up the entirety of the robot's visual component as all joints and links are at some level their children.
    *
    * @return ArrayList containing the root joints of the robot.
    */
   public ArrayList<Joint> getRootJoints()
   {
      return rootJoints;
   }
   
   public void getRootJoints(ArrayList<Joint> jointsToPack)
   {
      jointsToPack.addAll(rootJoints);
   }


   /**
    * Sets gravity to the specified values. For example, if using
    * meter-kilogram-seconds units of measure with Earth's gravity, then use
    * setGravity(0.0, 0.0, -9.81);
    *
    * @param gravityX X component of the Gravity vector.
    * @param gravityY Y component of the Gravity vector.
    * @param gravityZ Z component of the Gravity vector.
    */
   public void setGravity(double gravityX, double gravityY, double gravityZ)
   {
      this.gravityX.set(gravityX);
      this.gravityY.set(gravityY);
      this.gravityZ.set(gravityZ);
   }
   
   public void setGravity(Vector3D gravity)
   {
      this.gravityX.set(gravity.getX());
      this.gravityY.set(gravity.getY());
      this.gravityZ.set(gravity.getZ());
   }
   
   /**
    * Sets the Z component of the gravity to the specified value. For example, if using
    * meter-kilogram-seconds units of measure with Earth's gravity, then use
    * setGravity(-9.81);
    *
    * @param gZ Z component of the Gravity vector. X and Y components are set to 0.0.
    */
   public void setGravity(double gZ)
   {
      setGravity(0.0, 0.0, gZ);
   }

   /**
    * Adds a FunctionToIntegrate which will be integrated each simulation step.  If no functions are present that step is skipped.
    *
    * @param functionToIntegrate The function to be integrated.
    * @see FunctionToIntegrate FunctionToIntegrate
    */
   public void addFunctionToIntegrate(FunctionToIntegrate functionToIntegrate)
   {
      if (functionToIntegrate == null)
         return;
      if (functionIntegrators == null)
         functionIntegrators = new FunctionIntegrators();

      functionIntegrators.addFunctionIntegrator(new FunctionIntegrator(functionToIntegrate));

      // functionIntegrator = new FunctionIntegrator(functionToIntegrate);
   }

   //TODO: Refactor all setController methods to addController

   /**
    * Adds a controller to use with this Robot.  A single robot can have multiple controllers,
    * all of which execute their {@link RobotController#doControl doControl} methods when called.
    * Controllers added with this function doControl every simulation tick.
    *
    * @param controller RobotController to use with this robot.
    * @see RobotController RobotController
    */
   public void setController(RobotController controller)
   {
      setController(controller, 1);
   }

   /**
    * Adds a controller to use with this Robot. This method provides a method for reducing the number of control ticks
    * per simulation tick.  In other words, the controller only executes once every x simulation ticks.
    *
    * @param controller RobotController to use with this robot.
    * @param simulationTicksPerControlTick Number of simulation ticks per control tick.
    */
   public void setController(RobotController controller, int simulationTicksPerControlTick)
   {
      setController(new RobotControllerAndParameters(controller, simulationTicksPerControlTick));
   }
   
   public void setController(ArrayList<RobotController> controllers, int simulationTicksPerControlTick)
   {
      for(int i = 0; i<controllers.size(); i++)
      {
         RobotController controller = controllers.get(i);
         setController(controller,simulationTicksPerControlTick);
      }
   }
   
   public void setControllersAndCallInitialization(ArrayList<RobotControllerAndParameters> robotControllersAndParameters)
   {
      for (RobotControllerAndParameters robotControllerAndParameters : robotControllersAndParameters)
      {
         setController(robotControllerAndParameters);
         robotControllerAndParameters.getController().initialize();
      }
   }
   
   public void setController(RobotControllerAndParameters controllerAndParameters)
   {
      YoVariableRegistry registry = controllerAndParameters.getController().getYoVariableRegistry();
      this.controllers.add(controllerAndParameters);
      addYoVariableRegistry(registry);
   }

   /**
    * Executes the doControl method for each controller assigned to this robot.  This method
    * is called once per simulation tick.  If simulationTicksPerControlTick for a given controller
    * is something other than one the function will skip that controller.
    */
   protected final void doControllers()
   {
      if (controllers == null)
         return;

//    for(RobotControllerAndParameters controller : controllers)
      for (int i = 0; i < controllers.size(); i++)
      {
         RobotControllerAndParameters controller = controllers.get(i);

         controller.ticks_till_control.set(controller.ticks_till_control.getIntegerValue() - 1);

         if (controller.ticks_till_control.getIntegerValue() <= 0)
         {
            controller.ticks_till_control.set(controller.simulationTicksPerControlTick);
            controller.controller.doControl();
         }
      }
   }


   // public RobotController getController(){return this.controller;}

   /**
    * Sets the ground contact model for this robot.  This allow the robot to interact with
    * its surroundings based on their characteristics.
    *
    * @param gcModel GroundContactModel to be used with this robot.
    * @see GroundContactModel GroundContactModel
    */
   public void setGroundContactModel(GroundContactModel gcModel)
   {
      this.groundContactModel = gcModel;
   }

   /**
    * Retrieves the VarList associated with this robot's GroundContactModel.  Will return null if
    * a model was not added.
    *
    * @return VarList associated with the GroundContactModel
    */

// protected VarList getGroundContactVarList(){return this.groundVars;}
// protected ArrayList<VarList> getControllerVarLists(){return this.controllerVarLists;}
// protected VarList getExternalForceVarList(){return this.groundVars;}

   /**
    * Gets the name of this Robot.
    *
    * @return Name of this Robot.
    */
   public String getName()
   {
      return this.name;
   }

   /**
    * Returns a list of the YoVariables associated with this robot.  This list does
    * not include variables belonging to its GroundContactModel or controllers.
    *
    * @return VarList of the Robot's variables.
    */

   public void update()
   {
      boolean updatePoints = true;
      boolean updateCameraMounts = true;
      boolean updateIMUMounts = true;

      update(updatePoints, updateCameraMounts, updateIMUMounts);
   }

   public void updateForPlayback()
   {
      boolean updatePoints = false;
      boolean updateCameraMounts = true;
      boolean updateIMUMounts = false;
      
      update(updatePoints, updateCameraMounts, updateIMUMounts);
   }

   /**
    * Updates all joint data without updating graphics.
    * TODO: Not sure if this needs to be synchronized anymore.
    */
   protected synchronized void update(boolean updatePoints, boolean updateCameraMounts, boolean updateIMUMounts)
   {
      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);
         rootJoint.recursiveUpdateJoints(null, updatePoints, updateCameraMounts, updateIMUMounts, t.getDoubleValue());
      }
   }
   
   public void updateIMUMountAccelerations()
   {
      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);
         rootJoint.recursiveUpdateJointsIMUMountAccelerations();
      }
   }

   /**
    * Updates the velocities for all ground contact points.  This is called once
    * per simulation tick to ensure all ground contact point velocities are updated.
    */
   public void updateAllGroundContactPointVelocities()
   {
      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);

         // +++JEP: OPTIMIZE
         rootJoint.physics.recursiveUpdateAllGroundContactPointVelocities();
      }
   }

   private Vector3D w_null = new Vector3D();
   private Vector3D v_null = new Vector3D();

   private SpatialVector a_hat_h_null = new SpatialVector();
   private RotationMatrix R_0_i = new RotationMatrix();

   /**
    * Steps through every joint adding each camera mount to the provided list.  This function is called
    * by the GUI on one of two occasions.  When the GUI is created and when a robot is set.  In both cases
    * the method is not called in the null robot case.
    *
    * @param cameraMountList CameraMountList to which all mount points are added.
    */
   public void getCameraMountList(CameraMountList cameraMountList)
   {
      ArrayList<CameraMountInterface> mountArrayList = new ArrayList<CameraMountInterface>();

      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);

         rootJoint.recursiveGetCameraMounts(mountArrayList);
         cameraMountList.addCameraMounts(mountArrayList);
      }
   }
   
   public ArrayList<SimulatedSensor> getSensors()
   {
      ArrayList<SimulatedSensor> ret = new ArrayList<SimulatedSensor>();
      
      ArrayList<Joint> children = this.getRootJoints();
      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.recursiveGetSensors(ret);
      }
      
      return ret;
   }
   
   /**
    * Returns an ArrayList appropriate to the type of sensor being queried containing all the sensors of that type for each joint. 
    */
   @SuppressWarnings("unchecked")
   public <T extends SimulatedSensor> ArrayList<T> getSensors(Class<T> sensorType)
   {
      ArrayList<SimulatedSensor> allSensors = getSensors();
      
      ArrayList<T> specificSensors = new ArrayList<T>();
      
      for(SimulatedSensor sensor : allSensors)
      {
         if(sensorType.isAssignableFrom(sensor.getClass()))
         {
            specificSensors.add((T) sensor);
         }
      }

      return specificSensors;
   }
   
   
   public void getAllOneDegreeOfFreedomJoints(ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints)
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.recursiveGetOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);
      }    
   }

   public void getLidarMounts(ArrayList<LidarMount> lidarMountsToPack)
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.recursiveGetLidarMounts(lidarMountsToPack);
      }    
   }
   
   public void getIMUMounts(ArrayList<IMUMount> imuMountsToPack)
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.recursiveGetIMUMounts(imuMountsToPack);
      }    
   }
   
   public void getForceSensors(ArrayList<WrenchCalculatorInterface> forceSensors)
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.recursiveGetForceSensors(forceSensors);
      }
   }

   /**
    * Retrieves a list of all ground contact points associated with this robot and the groundContactGroupIdentifier.
    *
    * @return ArrayList containing the GroundContactPoints, if none exist the list will be empty.
    */
   @Override
   public ArrayList<GroundContactPoint> getGroundContactPoints(int groundContactGroupIdentifier)
   {
      ArrayList<GroundContactPoint> ret = new ArrayList<GroundContactPoint>();

      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.physics.recursiveGetGroundContactPoints(groundContactGroupIdentifier, ret);
      }

      return ret;
   }

   public ArrayList<ArrayList<GroundContactPoint>> getAllGroundContactPointsGroupedByJoint()
   {
      ArrayList<ArrayList<GroundContactPoint>> ret = new ArrayList<ArrayList<GroundContactPoint>>();

      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.physics.recursiveGetAllGroundContactPointsGroupedByJoint(ret);
      }

      return ret;
   }

   public ArrayList<GroundContactPoint> getAllGroundContactPoints()
   {
      ArrayList<GroundContactPoint> ret = new ArrayList<GroundContactPoint>();

      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.physics.recursiveGetAllGroundContactPoints(ret);
      }

      return ret;
   }
   
   public ArrayList<ExternalForcePoint> getAllExternalForcePoints()
   {
      ArrayList<ExternalForcePoint> ret = new ArrayList<ExternalForcePoint>();

      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.physics.recursiveGetAllExternalForcePoints(ret);
      }

      return ret;
   }
   
   public ArrayList<KinematicPoint> getAllKinematicPoints()
   {
      ArrayList<KinematicPoint> ret = new ArrayList<KinematicPoint>();

      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         rootJoint.physics.recursiveGetAllKinematicPoints(ret);
      }

      return ret;
   }

   public ExternalForcePoint getExternalForcePoint(String name)
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);
         ExternalForcePoint externalForcePoint = rootJoint.recursiveGetExternalForcePoint(name);
         if (externalForcePoint != null) 
            return externalForcePoint;
      }

      return null;
   }

   /**
    * Adds the specified link to the robot.  Static links have no effect on the simulation, they are purely cosmetic.
    *
    * @param staticLink Link to be added.
    */
   public void addStaticLink(Link staticLink)
   {
      addStaticLinkGraphics(staticLink.getLinkGraphics());
   }

   /**
    * Adds the specified LinkGraphics to the robot.  Static LinkGraphics have no effect on the simulation, they are purely cosmetic.
    *
    * @param linkGraphics LinkGraphics to be added.
    */
   public void addStaticLinkGraphics(Graphics3DObject linkGraphics)
   {      
      
      staticLinkGraphics.add(linkGraphics);
      
   }
   
   public void addStaticLinkGraphics(ArrayList<Graphics3DObject> linkGraphicsArray)
   {
      for (Graphics3DObject linkGraphics : linkGraphicsArray)
      {
         addStaticLinkGraphics(linkGraphics);
      }
   }


   /**
    * Updates the velocities and positions at each joint as well as the rotation matrices from and to the base coordinate system.  Each joint stores
    * both its rotational and translational velocities.  The updated velocities are based on those of the previous joint in the chain.
    * This also updates all kinetic and ground contact points.
    */
   public void updateVelocities()
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);

         R_0_i.setIdentity();
         w_null.set(0.0, 0.0, 0.0);
         v_null.set(0.0, 0.0, 0.0);
         a_hat_h_null.top.set(0.0, 0.0, 0.0);
         a_hat_h_null.bottom.set(0.0, 0.0, 0.0);
         rootJoint.physics.featherstonePassOne(w_null, v_null, R_0_i);
      }

      this.update();
   }

   /**
    * This function handles the dynamics of the robot.  While simulating, each simulation "tick" triggers this function four times
    * allowing the calculation of the four Runge-Kutta coefficients.  These coefficients are calculated by executing the four Featherstone
    * passes on each joint tree.
    *
    * The first pass recurses down the tree, calculating the linear and angular velocities for each link.  In the process the rotation
    * matrices and radius vectors are recalculated.  The coordinate system of each joint/link space is centered on the current link's
    * center of mass.  Each joint contains a rotation matrix and corresponding radius vector to translate information to and from the previous
    * link's coordinate system. These functions must be updated before the velocities may be calculated.  The kinematic and ground contact points
    * for each joint are also updated.
    *
    * Pass two recurses down the tree again, calculating the isolated components of the spatial articulated inertia and spatial isolated
    * zero-acceleration force for each link.  Coriolis forces and forces from ground contact and kinetic points are included in these calculations.
    *
    * In the third pass the completed articulated inertia and articulated z.a. forces are calculated for each link by recursing up the tree.  These
    * values are based on the positions, velocities and torques of each link.
    *
    * The final pass recurses down the tree calculating the joint accelerations based on the information generated by the previous three
    * passes.  During this pass the joint positions, velocities and accelerations are stored for use in the Runge-Kutta calculations.
    *
    * @param passNumber Current pass, this is used in featherstonePassFour when saving the current values of k_q, k_qd, and k_qdd
    * @throws UnreasonableAccelerationException
    */
   private void doDynamics(int passNumber) throws UnreasonableAccelerationException
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);

         if (rootJoint.isDynamic())
         {
            R_0_i.setIdentity();
            w_null.set(0.0, 0.0, 0.0);
            v_null.set(0.0, 0.0, 0.0);
            a_hat_h_null.top.set(0.0, 0.0, 0.0);
            a_hat_h_null.bottom.set(0.0, 0.0, 0.0);
            rootJoint.physics.featherstonePassOne(w_null, v_null, R_0_i);

            // System.out.println("\n\n");
            rootJoint.physics.featherstonePassTwo(w_null);
            rootJoint.physics.featherstonePassThree();
            rootJoint.physics.featherstonePassFour(a_hat_h_null, passNumber);
         }

         else
         {
            R_0_i.setIdentity();
            w_null.set(0.0, 0.0, 0.0);
            v_null.set(0.0, 0.0, 0.0);
            a_hat_h_null.top.set(0.0, 0.0, 0.0);
            a_hat_h_null.bottom.set(0.0, 0.0, 0.0);
            rootJoint.physics.featherstonePassOne(w_null, v_null, R_0_i);

            // rootJoint.featherstonePassTwo(w_null);
            // rootJoint.featherstonePassThree();
            // rootJoint.featherstonePassFour(a_hat_h_null, passNumber);
            rootJoint.physics.recordK(passNumber);
         }
      }
   }

   public YoVariableRegistry getRobotsYoVariableRegistry()
   {
      return yoVariableRegistry;
   }
   
   public void setRobotsYoVariableRegistry(YoVariableRegistry registry)
   {
     this.yoVariableRegistry = registry;
   }

   /**
    * Saves the current state of each joint.  Different joint types have different relevant values, pin and slider joints
    * store only position and velocity.  These values are restored prior to each Euler integration in order to properly
    * calculate the Runge-Kutta slope.
    */
   private void rootJointsRecursiveSaveTempState()
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);

         rootJoint.physics.recursiveSaveTempState();
      }

   }

   /**
    * Calculates the position and velocity for each joint using euler integration.
    * y_(n+1) = y_n + h*f(t_n, y_n)  where h is the step size.
    * This function used in the calculation of the Runge-Kutta slope.
    *
    * @param dt Time step size for Euler integration
    */
   public void rootJointsRecursiveEulerIntegrate(double dt)
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);

         rootJoint.physics.recursiveEulerIntegrate(dt);
      }
   }

   /**
    * Steps through evey joint restoring the saved values for use
    * in the next Euler integration.
    */
   private void rootJointsRecursiveRestoreTempState()
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);

         rootJoint.physics.recursiveRestoreTempState();
      }
   }

   /**
    * Steps through each joint updating data using the Runge-Kutta method
    * The next value in a Runge-Kutta series is described by the following formulas:
    * y_(n+1) = y_n + (1/6) * h * (k_1 + 2k_2 + 2 k_3 + k_4)
    * t_(n+1) = t_n + h
    *
    * Where:
    * h is the step size (dt)
    * k_1 is the slope at the beginning of the interval
    * k_2 is the slope at the midpoint of the interval calcuated by using k_1 to predict the value of y at point t_n + h/2 using Euler's method
    * k_3 is the slope at the midpoint recalculated using k_2
    * k_4 is the slope at the end of the interval determined via k_3
    *
    * These coefficents are calculated by the doDynamics function and stored.  That function is called four times, once per k after a Euler integration
    * steps the values.
    *
    * @param dt Step size to be used.
    */
   private void rootJointsRecursiveRungeKuttaSum(double dt)
   {
      ArrayList<Joint> children = this.getRootJoints();

      for (int i = 0; i < children.size(); i++)
      {
         Joint rootJoint = children.get(i);

         rootJoint.physics.recursiveRungeKuttaSum(dt);
      }
   }

   // Override this method if you wish to do something before the dynamics tick,
   // For example, simulate a spring. This will get called only once per tick, even though the Runge-Kutta
   // integrator will do the dynamics 4 times per tick.
   public void doAfterControlBeforeDynamics()
   {
   }


   /**
    * Will update the qdd values of the joints, but not integrate the velocities or anything like that.
    * Good for testing inverse dynamics routines and things like that.
    */
   public void doDynamicsButDoNotIntegrate() throws UnreasonableAccelerationException
   {
      doAfterControlBeforeDynamics();
      doDynamics(0);
   }

   /**
    * <p>Calculates the robot dynamics and integrates to the next step.  This integration is accomplished via the Runge-Kutta method
    * which approximates the next point in the series by calculating four intermediary slopes.  The robot dynamics are calculated four times
    * using the following method to generate these Runge-Kutta slopes.</p>
    *
    * <p>Robot dynamics are calculated via the Featherstone algorithm as discussed by Brian Mirtich in his Thesis, <i>Impule-based Dynamic Simulation of Rigid Body Systems</i>.
    * This algorithm is implemented in four distinct passes as follows:</p>
    *
    * <OL>
    * <LI>The first pass recurses down the tree, calculating the linear and angular velocities for each link.  In the process the rotation
    * matricies and radius vectors are recalculated.  The coordinate system of each joint/link space is centered on the current link's
    * center of mass.  Each joint contains a rotation matrix and corresponding radius vector to translate information to and from the previous
    * link's coordinate system. These functions must be updated before the velocities may be calculated.  The kinematic and ground contact points
    * as well as torque for each joint are also updated.</LI>
    *
    * <LI>Pass two recurses down the tree again, calculating the isolated components of the spatial articulated inertia and spatial isolated
    * zero-acceleration force for each link.  Coriolis forces and forces from ground contact and kinetic points are included in these calculations.</LI>
    *
    * <LI>In the third pass the completed articulated inertia and articulated z.a. forces are calculated for each link by recursing up the tree.  These
    * values are based on the positions, velocities and torques of each link.</LI>
    *
    * <LI>The final pass recurses down the tree calculating the joint accelerations based on the information generated by the previous three
    * passes.  During this pass the joint positions, velocities and accelerations are stored for use in the Runge-Kutta calculations.</LI>
    * </OL>
    *
    * <p>Between each run of the dynamics Euler Integration is used to shift the values in preparation for the next step.  However, the values are returned
    * to their original states before each integration.  Each pass of the dynamics stores the Runge-Kutta slope for each parameter.  Once the dynamics have
    * been calculated the joints are checked for unreasonable accelerations.  If none are present, Runge-Kutta is executed for each joint and the current time
    * is updated.
    *
    * @param DT Step time for the integration.
    * @throws UnreasonableAccelerationException This exception indicates that at least one joint underwent an unreasonable acceleration as defined by its joint type.  This is often caused by overly large step times (DT).
    */
   public void doDynamicsAndIntegrate(double DT) throws UnreasonableAccelerationException
   {
      doAfterControlBeforeDynamics();

      if (functionIntegrators != null)
      {
         doDynamicsAndIntegrateWithFunction(DT);

         return;
      }

      double temp_time = t.getDoubleValue();
      
      switch (dynamicIntegrationMethod)
      {
      case RUNGE_KUTTA_FOURTH_ORDER:
      {
         rootJointsRecursiveSaveTempState();

         doDynamics(0);
         rootJointsRecursiveEulerIntegrate(DT / 2.0);
         doDynamics(1);

         rootJointsRecursiveRestoreTempState();
         rootJointsRecursiveEulerIntegrate(DT / 2.0);
         doDynamics(2);

         rootJointsRecursiveRestoreTempState();
         rootJointsRecursiveEulerIntegrate(DT);
         doDynamics(3);

         rootJointsRecursiveRungeKuttaSum(DT);

         t.set(temp_time + DT);

         break;
      }

      case EULER_DOUBLE_STEPS:
      {
         rootJointsRecursiveSaveTempState();

         doDynamics(0);
         rootJointsRecursiveEulerIntegrate(DT / 2.0);
         t.set(temp_time + DT/2.0);

         rootJointsRecursiveSaveTempState();

         doDynamics(1);
         rootJointsRecursiveEulerIntegrate(DT / 2.0);
         t.set(temp_time + DT);
         
         break;
      }

      default:
         throw new RuntimeException("Should not get here");
      }

      

//    }

//    else
//    {
//       ArrayList<Joint> unreasonableAccelerationJoints = getUnreasonableAccelerationJoints();
//       throw new UnreasonableAccelerationException(unreasonableAccelerationJoints);
//
//       // System.err.println("Unreasonable Accelerations!");
//    }
   }


   private void doDynamicsAndIntegrateWithFunction(double DT) throws UnreasonableAccelerationException
   {
      double temp_time = t.getDoubleValue();

      // Save the temporary state
      functionIntegrators.saveTempState();
      rootJointsRecursiveSaveTempState();

      update();
      functionIntegrators.doDynamics(0);
      doDynamics(0);
      functionIntegrators.eulerIntegrate(DT / 2.0);
      rootJointsRecursiveEulerIntegrate(DT / 2.0);
      update();
      t.set(temp_time + DT / 2.0);
      functionIntegrators.doDynamics(1);
      doDynamics(1);

      functionIntegrators.restoreTempState();
      rootJointsRecursiveRestoreTempState();
      functionIntegrators.eulerIntegrate(DT / 2.0);
      rootJointsRecursiveEulerIntegrate(DT / 2.0);
      update();
      functionIntegrators.doDynamics(2);
      doDynamics(2);

      functionIntegrators.restoreTempState();
      rootJointsRecursiveRestoreTempState();
      functionIntegrators.eulerIntegrate(DT);
      rootJointsRecursiveEulerIntegrate(DT);
      update();
      t.set(temp_time + DT);
      functionIntegrators.doDynamics(3);
      doDynamics(3);

      functionIntegrators.rungeKuttaSum(DT);
      rootJointsRecursiveRungeKuttaSum(DT);

      // Increase time
      t.set(temp_time + DT);
   }

   /**
    * Computes the total translational kinetic energy of this Robot. This is the sum over i of 1/2 m_i v_i*v_i,
    * where m_i is the mass of link i, and v_i is the translational velocity of the center of mass of link i.
    *
    * @return Total translational kinetic energy.
    */
   public double computeTranslationalKineticEnergy()
   {
      double translationalKineticEnergy = 0.0;

      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);

         translationalKineticEnergy = translationalKineticEnergy + computeTranslationalKineticEnergy(rootJoint);
      }

      return translationalKineticEnergy;
   }

   /**
    * Computes the total translational kinetic energy of the subtree rooted at
    * the specified root Joint. This is the sum over i of 1/2 m_i v_i*v_i, where
    * m_i is the mass of link i, and v_i is the translational velocity of the
    * center of mass of link i, and i includes all decendents of the specified
    * root Joint.
    *
    * @return Total translational kinetic energy of the subtree rooted at the
    *   specified root Joint.
    * @param rootJoint Root Joint to compute energy from.
    */
   public double computeTranslationalKineticEnergy(Joint rootJoint)
   {
      return rootJoint.physics.recursiveComputeTranslationalKineticEnergy();
   }

   /**
    * Computes the total rotational kinetic energy of this Robot. This is the sum over i of 1/2 w_i^T*J_i*w_i,
    * where J_i is the ineria matrix of link i, and w_i is the rotational velocity vector of the center of mass of link i.
    *
    * @return Total rotational kinetic energy.
    */
   public double computeRotationalKineticEnergy()
   {
      double rotationalKineticEnergy = 0.0;

      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);

         rotationalKineticEnergy = rotationalKineticEnergy + computeRotationalKineticEnergy(rootJoint);
      }

      return rotationalKineticEnergy;
   }


   /**
    * Computes the total rotational kinetic energy of the subtree rooted at
    * the specified root Joint. This is the sum over i of 1/2 w_i^T*J_i*w_i, where
    * J_i is the inertia matrix of link i, and w_i is the rotational velocity vector of the
    * center of mass of link i, and i consisting of all the decendents of the specified
    * root Joint.
    *
    * @return Total rotational kinetic energy of the subtree rooted at the
    * specified root Joint.
    * @param rootJoint Root Joint to compute energy from.
    */
   public double computeRotationalKineticEnergy(Joint rootJoint)
   {
      return rootJoint.physics.recursiveComputeRotationalKineticEnergy();
   }

   /**
    * Computes the total gravitational potential energy of this Robot. This is the sum over i of m_i*g*h_i,
    * where m_i is the mass of link i, g is the gravitational constant, and h_i is the height above 0 of link i.
    *
    * @return Total gravitational potential energy.
    */
   public double computeGravitationalPotentialEnergy()
   {
      double gravitationalPotentialEnergy = 0.0;

      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);

         gravitationalPotentialEnergy = gravitationalPotentialEnergy + computeGravitationalPotentialEnergy(rootJoint);
      }

      return gravitationalPotentialEnergy;
   }

   /**
    * Computes the total gravitational potential energy of the subtree rooted at the specified root Joint. This is the sum over i of m_i*g*h_i,
    * where m_i is the mass of link i, g is the gravitational constant, and h_i is the height above 0 of link i,
    * with i consisting of all the decendents of the specified root Joint.
    *
    * @return Total gravitational potential energy.
    */
   public double computeGravitationalPotentialEnergy(Joint rootJoint)
   {
      return rootJoint.physics.recursiveComputeGravitationalPotentialEnergy();
   }


   private Point3D tempCOMPoint = new Point3D();    // Temporary point storing the robot's center of mass

   /**
    * Computes the center of mass of this Robot.  This center of mass position is returned
    * by altering the provided Point3D.  If the robot has no mass, it also has no
    * center of mass point.  This value is used in the calculation of center of momentum.
    *
    * @param comPoint Center of Mass point, in World Coordinates, that is computed.
    * @return The total mass of the robot.
    */
   public double computeCenterOfMass(Point3D comPoint)
   {
      double totalMass = 0.0;
      comPoint.set(0.0, 0.0, 0.0);

      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);

         double mass = rootJoint.physics.recursiveComputeCenterOfMass(tempCOMPoint);
         totalMass = totalMass + mass;
         tempCOMPoint.scale(mass);
         comPoint.add(tempCOMPoint);
      }

      if (totalMass > 0.0)
      {
         comPoint.scale(1.0 / totalMass);
      }
      else
         comPoint.set(0.0, 0.0, 0.0);

      return totalMass;
   }

   /**
    * Computes the Center of Mass of the subtree rooted at the specified root Joint.
    * This center of mass position is returned by altering the provided Point3D.
    * If the robot has no mass, it also has no center of mass point.
    *
    * @param comPoint Center of Mass point, in World Coordinates, that is computed.
    * @return The total mass of the robot.
    */
   public double computeCenterOfMass(Joint rootJoint, Point3D comPoint)
   {
      double totalMass = 0.0;
      comPoint.set(0.0, 0.0, 0.0);

      double mass = rootJoint.physics.recursiveComputeCenterOfMass(tempCOMPoint);
      totalMass = totalMass + mass;
      tempCOMPoint.scale(mass);
      comPoint.add(tempCOMPoint);

      if (totalMass > 0.0)
      {
         comPoint.scale(1.0 / totalMass);
      }
      else
         comPoint.set(0.0, 0.0, 0.0);

      return totalMass;
   }

   private Vector3D tempLinearMomentum = new Vector3D();

   /**
    * Computes the total linear momentum of the center of mass for
    * this Robot.  The total mass of the robot is also computed and returned.
    *
    * @param linearMomentum Total linear momentum vector that is computed.
    * @return The total mass of the robot.
    */
   public double computeLinearMomentum(Vector3D linearMomentum)
   {
      double totalMass = 0.0;
      linearMomentum.set(0.0, 0.0, 0.0);

      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);

         double mass = rootJoint.physics.recursiveComputeLinearMomentum(tempLinearMomentum);
         totalMass = totalMass + mass;
         linearMomentum.add(tempLinearMomentum);
      }

      return totalMass;
   }

   /**
    * Computes the total linear momentum of the center of mass for the
    * subtree rooted at the specified root Joint.  The total mass of the
    * robot is also computed and returned.
    *
    * @param rootJoint Root Joint for which linear momentum is computed.
    * @param linearMomentum Total linear momentum vector that is computed.
    * @return The total mass of the robot.
    */
   public double computeLinearMomentum(Joint rootJoint, Vector3D linearMomentum)
   {
      double totalMass = 0.0;
      linearMomentum.set(0.0, 0.0, 0.0);

      double mass = rootJoint.physics.recursiveComputeLinearMomentum(tempLinearMomentum);
      totalMass = totalMass + mass;
      linearMomentum.add(tempLinearMomentum);

      return totalMass;
   }


   private Vector3D tempAngularMomentum = new Vector3D();

   /**
    * Computes the total angular momentum about the center of mass for
    * this Robot.
    *
    * @param angularMomentum Total angular momentum vector that is computed.
    */
   public void computeAngularMomentum(Vector3D angularMomentum)
   {
      angularMomentum.set(0.0, 0.0, 0.0);

      for (int i = 0; i < rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);

         rootJoint.physics.recursiveComputeAngularMomentum(tempAngularMomentum);
         angularMomentum.add(tempAngularMomentum);
      }
   }

   /**
    * Computes the total angular momentum about the center of mass for the
    * subtree rooted at the specified root Joint.
    *
    * @param rootJoint Root Joint for which linear momentum is computed.
    * @param angularMomentum Total angular momentum vector that is computed.
    */
   public void computeAngularMomentum(Joint rootJoint, Vector3D angularMomentum)
   {
      angularMomentum.set(0.0, 0.0, 0.0);

      rootJoint.physics.recursiveComputeAngularMomentum(tempAngularMomentum);
      angularMomentum.add(tempAngularMomentum);
   }


   private Vector3D tempCOMVector = new Vector3D();

   /**
    * Computes the Center of Mass location and total linear and angular momentum
    * about the Center of Mass for this Robot.
    *
    * @param comPoint Center of Mass point that is computed.
    * @param linearMomentum Total linear momentum vector that is computed.
    * @param angularMomentum Total angular momentum vector that is computed.
    * @return Total mass of the robot.
    */
   public double computeCOMMomentum(Point3D comPoint, Vector3D linearMomentum, Vector3D angularMomentum)
   {
      double mass = computeCenterOfMass(comPoint);
      computeLinearMomentum(linearMomentum);
      computeAngularMomentum(angularMomentum);

      // System.out.println("Angular Momentum about 0: " + angularMomentum);
      // Angular momentum is about 0,0,0.  Transform to about com:
      tempCOMVector.set(comPoint);
      tempAngularMomentum.cross(tempCOMVector, linearMomentum);

      // System.out.println("Linear Momentum: " + linearMomentum);
      // System.out.println("com X linearMomentum: " + tempAngularMomentum);

      angularMomentum.sub(tempAngularMomentum);

      return mass;
   }

   /**
    * Computes the Center of Mass location and total linear and angular momentum
    * about the center of mass for the subtree rooted at the specified root
    * Joint.
    *
    * @param rootJoint Root Joint to computed momentum from.
    * @param comPoint Center of Mass point that is computed.
    * @param linearMomentum Total linear momentum vector that is computed.
    * @param angularMomentum Total angular momentum vector that is computed.
    * @return Total mass of the robot.
    */
   public double computeCOMMomentum(Joint rootJoint, Point3D comPoint, Vector3D linearMomentum, Vector3D angularMomentum)
   {
      double mass = computeCenterOfMass(rootJoint, comPoint);
      computeLinearMomentum(rootJoint, linearMomentum);
      computeAngularMomentum(rootJoint, angularMomentum);

      // System.out.println("Angular Momentum about 0: " + angularMomentum);
      // Angular momentum is about 0,0,0.  Transform to about com:
      tempCOMVector.set(comPoint);
      tempAngularMomentum.cross(tempCOMVector, linearMomentum);

      // System.out.println("Linear Momentum: " + linearMomentum);
      // System.out.println("com X linearMomentum: " + tempAngularMomentum);

      angularMomentum.sub(tempAngularMomentum);

      return mass;
   }


   private final Vector3D tempRVector = new Vector3D();
   private final Vector3D tempRCrossF = new Vector3D(), tempForce = new Vector3D();

   /**
    * Computes the Center of Pressure of the GroundContactPoints attached to this
    * Robot.
    *
    * @param copPoint Center of Pressure point that is computed.
    * @param copForce Center of Pressure total force vector that is computed.
    * @param copMoment Total moment generated about the Center of Pressure from all the GroundContactPoints.
    */
   public void computeCenterOfPressure(Point3D copPoint, Vector3D copForce, Vector3D copMoment)
   {
      copPoint.set(0.0, 0.0, 0.0);
      copForce.set(0.0, 0.0, 0.0);
      copMoment.set(0.0, 0.0, 0.0);

      ArrayList<GroundContactPoint> gcPoints = this.getAllGroundContactPoints();

      for (int i = 0; i < gcPoints.size(); i++)
      {
         GroundContactPoint gc = gcPoints.get(i);
         gc.getForce(tempForce);
         copForce.add(tempForce);

         double fz = tempForce.getZ();
         copPoint.setX(copPoint.getX() + gc.getX() * fz);
         copPoint.setY(copPoint.getY() + gc.getY() * fz);
         copPoint.setZ(copPoint.getZ() + gc.getZ() * fz);
      }

      if (copForce.getZ() < 1e-14)
      {
         copPoint.set(Double.NaN, Double.NaN, Double.NaN);
         copMoment.set(0.0, 0.0, 0.0);
         return;
      }
        
      copPoint.scale(1.0 / copForce.getZ());

      for (int i = 0; i < gcPoints.size(); i++)
      {
         GroundContactPoint gc = gcPoints.get(i);

         tempRVector.setX(copPoint.getX() - gc.getX());
         tempRVector.setY(copPoint.getY() - gc.getY());
         tempRVector.setZ(copPoint.getZ() - gc.getZ());

         gc.getForce(tempForce);
         tempRCrossF.cross(tempRVector, tempForce);

         copMoment.add(tempRCrossF);
      }
   }

   /**
    * This method returns the display name of the robot followed
    * by the names of each joint.
    *
    * @return String, display name of the robot
    */
   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();

      Queue<Joint> queue = new LinkedList<Joint>();
      
      retBuffer.append("Robot: " + name + "\n\n");

      ArrayList<Joint> children = this.getRootJoints();

      queue.addAll(children);
      
      while(!queue.isEmpty())
      {
        Joint joint = queue.poll();
            
        retBuffer.append("\n"+joint.toString());
        ArrayList<Joint> childrenJoints = joint.getChildrenJoints();

        queue.addAll(childrenJoints);
      }

      return retBuffer.toString();
   }
   
   public void printRobotJointsAndMasses(StringBuffer stringBuffer)
   {
      Queue<Joint> queue = new LinkedList<Joint>();
      ArrayList<Joint> children = this.getRootJoints();

      queue.addAll(children);

      while(!queue.isEmpty())
      {
         Joint joint = queue.poll();

         stringBuffer.append("\n" + joint.getName() + ": mass = " + joint.getLink().getMass());
         ArrayList<Joint> childrenJoints = joint.getChildrenJoints();

         queue.addAll(childrenJoints);
      }

   }

   /**
    * {@literal Outputs a Java class which can be used as a base for this Robot's RobotController.  This
    * class contains all of the variables associated with the robot preregistered saving the user
    * time when creating their RobotController.  The class will be a member of the same package as
    * parent Robot with a name of the following format: <ThisRobotClassName without 'Robot'>ControllerBase
    * For example, a robot named SpringFlamingoRobot would produce a controller base named SpringFlamingoControllerBase.
    * Any names not ending in robot are left untouched, therefore a robot named SpringRobotFlamingo would result in
    * a controller base named SpringRobotFlamingoControllerBase.}
    *
    * @param stream PrintStream to output the class to. Use System.out to output to the screen.
    */

// public void createControllerBase(PrintStream stream)
// {
//   // Retrieve the class and package names
//   String name = this.getClass().getSimpleName();  // Robot Class
//   String baseName; // Name of ControllerBase
//   // Lets remove any Robot endings from the ControllerBase name
//   if (name.endsWith("Robot"))
//     baseName = name.substring(0, name.length() - "Robot".length());
//   else
//     baseName = name;
//
//   String packageName = this.getClass().getPackage().getName(); // Robot Package
//
//   stream.println("package " + packageName + ";");
//   stream.println("");
//   stream.println("import us.ihmc.simulationconstructionset.*;");
//   stream.println("import java.util.*;");
//   stream.println("");
//   stream.println("public class " + baseName + "ControllerBase implements YoVariableRegistry");
//   stream.println("{");
//   println(stream, 3, "protected " + name + " rob;");
//   stream.println("");
//   println(stream, 3, "// These are the variables that are automatically created when the robot is created:");
//   //println(stream, 3, "YoVariable t;");
//
//   //VarList robVariables = new VarList("Robot Variables");
//   // Print the joint variables:
//   this.printVarBase(stream, robVars);
//   stream.println("");
//   this.printVarBase(stream, groundVars);
//
//   //for(int jointNum = 0; jointNum < rootJoints.size(); jointNum++)
//   //{
//    // Joint rootJoint = (Joint) rootJoints.get(jointNum);
//    // recursivelyPrintJointBase(rootJoint, stream, robVariables);
//   //}
//
//   stream.println("");
//   println(stream, 3, "// User defined control variables will be placed in this ArrayList when they are registered:");
//   println(stream, 3, "ArrayList controlVars = new ArrayList();");
//   println(stream, 3, "LinkedHashMap controlVarsHashMap = new LinkedHashMap();");
//   stream.println("");
//   println(stream, 3, "public " + baseName + "ControllerBase(" + name + " rob)");
//   println(stream, 3, "{");
//   println(stream, 5, "this.rob = rob;");
//   stream.println("");
//   println(stream, 5, "// Get the variables that are stored with the robot:");
//   stream.println("");
//
//   //println(stream, 5, "t = rob.getVar(\"t\");");
//   stream.println("");
//
//   //
//   for(int varNum=0; varNum<robVars.size(); varNum++)
//   {
//     YoVariable var = robVars.get(varNum);
//
//     if(varNum % 3 == 0) {stream.print("     ");}
//     stream.print(var.getName() + " = rob.getVariable(\"" + var.getName() + "\"); ");
//     if(varNum % 3 == 2) {stream.println("");}
//   }
//
//   stream.println("");
//   for(int varNum=0; varNum<groundVars.size(); varNum++)
//   {
//     YoVariable var = groundVars.get(varNum);
//
//     if(varNum % 3 == 0) {stream.print("     ");}
//     stream.print(var.getName() + " = rob.getVariable(\"" + var.getName() + "\"); ");
//     if(varNum % 3 == 2) {stream.println("");}
//   }
//
//
//   stream.println("");
//   println(stream, 3, "}");
//   stream.println("");
//   println(stream, 3, "public void registerVariable(YoVariable var)");
//   println(stream, 3, "{");
//   println(stream, 5, "if (controlVarsHashMap.containsKey(var.getName())) System.err.println(\"Warning:  \" + var.getName() + \" has already been registered\");");
//   println(stream, 5, "controlVarsHashMap.put(var.getName(), var);");
//   println(stream, 5, "controlVars.add(var);");
//   println(stream, 3, "}");
//   stream.println("");
//   println(stream, 3, "public YoVariable[] getControlVars()");
//   println(stream, 3, "{");
//   println(stream, 5, "YoVariable[] ret = new YoVariable[controlVars.size()];");
//   println(stream, 5, "controlVars.toArray(ret);");
//   println(stream, 5, "return ret;");
//   println(stream, 3, "}");
//
//   stream.println("");
//   println(stream, 3, "public YoVariable getVariable(String name)");
//   println(stream, 3, "{");
//   println(stream, 5, "YoVariable ret = null;");
//   println(stream, 5, "ret = (YoVariable) controlVarsHashMap.get(name);");
//   println(stream, 5, "if (ret != null) return ret;");
//   stream.println("");
//   println(stream, 5, "if (rob.hasVar(name)) return rob.getVariable(name);");
//   stream.println("");
//   println(stream, 5, "System.err.println(\"Variable \" + name + \" not found in " + name + "ControllerBase.getVariable()\");");
//   println(stream, 5, "return null;");
//   println(stream, 3, "}");
//
//
//   stream.println("");
//   println(stream, 3, "public boolean hasVar(String name)");
//   println(stream, 3, "{");
//   println(stream, 5, "YoVariable ret = null;");
//   println(stream, 5, "ret = (YoVariable) controlVarsHashMap.get(name);");
//   println(stream, 5, "if (ret != null) return true;");
//   stream.println("");
//   println(stream, 5, "if (rob.hasVar(name)) return true;");
//   stream.println("");
//   println(stream, 5, "return false;");
//   println(stream, 3, "}");
//
//
//   stream.println("}");
// }


   /**
    * {@literal Outputs a Java class which can be used as a base for this Robot's RobotController.  This
    * class contains all of the variables associated with the robot preregistered saving the user
    * time when creating their RobotController.  The class will be a member of the same package as
    * parent Robot with a name of the following format:  <ThisRobotClassName without 'Robot'>ControllerBase
    * For example, a robot named SpringFlamingoRobot would produce a controller base named SpringFlamingoControllerBase.
    * Any names not ending in robot are left untouched, therefore a robot named SpringRobotFlamingo would result in
    * a controller base named SpringRobotFlamingoControllerBase.}
    *
    * @param file File where the code will be stored.  Ensure that the name of this file corresponds to the
    * name of the class it contains.
    */
// public void createControllerBase(File file)
// {
//   try
//   {
//     PrintStream stream = new PrintStream(file);
//     createControllerBase(stream);
//   }
//   catch (FileNotFoundException missing)
//   {
//       System.out.println("The file could not be written for this reason:\n\t" + missing.getMessage());
//   }
// }

   /**
    * {@literal Outputs a Java class which can be used as a base for this Robot's RobotController.  This
    * class contains all of the variables associated with the robot preregistered saving the user
    * time when creating their RobotController.  The file will be named &ltThisRobotClassName
    * without 'Robot'&gtControllerBase.java and stored in the same directory as the class file
    * that created it.}
    */
// public void createControllerBase()
// {
//   File controllerBase;
//   try
//   {
//     controllerBase = new File(getPath().toURI());
//     PrintStream stream = new PrintStream(controllerBase);
//     createControllerBase(stream);
//   }
//   catch (FileNotFoundException missing)
//   {
//     System.out.println("The specified file could not be created and generated the following error message:\n\t" + missing.getMessage());
//   }
//   catch (URISyntaxException badFormat)
//   {
//     System.out.println("Path to " + this.getClass().getName() + " could not be calculated and the following error was observed:\n\t" + badFormat.getMessage());
//   }
//
// }

   /**
    * Build a path to a new ControllerBase file in the same directory as the robot's .class file.
    *
    * @return URL containing the new file path.
    */
   @SuppressWarnings("unused")
   private URL getPath()
   {
      Class<?> c = this.getClass();
      URL path;

      URL url = c.getResource(c.getSimpleName() + ".class");

      if (url == null)
         return null;

      String temp = url.toExternalForm();

      // First, attempt to remove just the name from the end.
      // For example, a path might be "C:/RobotStuff/Robot.class"
      // this would remove "/Robot.class" giving just the location
      // If the first attempt doesn't work, temp probably ends with
      // /<full class name with '.' replaced by '/'>.class; try to remove.
      // If that doesn't work either, try backslashes instead of
      // forward slashes.  As a last resort return null.
      String end = "/" + c.getSimpleName() + ".class";
      if (temp.endsWith(end))
      {
         temp = temp.substring(0, temp.length() - end.length());
      }
      else
      {
         end = "/" + c.getName().replace("\\.", "/") + ".class";

         if (temp.endsWith(end))
         {
            temp = temp.substring(0, temp.length() - end.length());
         }
         else
         {
            end = end.replace("/", "\\");

            if (temp.endsWith(end))
            {
               temp = temp.substring(0, temp.length() - end.length());
            }
            else
               return null;
         }
      }

      // Modify the path with the appropriate ControllerBase name.
      // For example, if this robot is named SpringFlamingo the controller
      // base would be SpringFlamingoControllerBase.java
      // If the name ends with "Robot", eg SpringFlamingoRobot, remove it.
      String temp2 = c.getSimpleName();
      temp += "/" + ((temp2.endsWith("Robot")) ? temp2.substring(0, temp2.length() - "Robot".length()) : temp2) + "ControllerBase.java";

      // Attempt to create the new URL
      try
      {
         path = new URL(temp);

         return path;
      }
      catch (MalformedURLException broken)
      {
         // Something is wrong with the path, tell the user
         System.err.println("Path to " + c.getName() + " could not be calculated and the following error was observed:\n\t" + broken.getMessage()
                            + "\n\tPath: " + temp);

         return null;
      }
   }

   /**
    * Function used while generating the controller base code.
    *
    * @param stream PrintStream to output on.
    * @param indent Number of spaces to indent.
    * @param line Text to output.
    */
   @SuppressWarnings("unused")
   private void println(PrintStream stream, int indent, String line)
   {
      for (int i = 0; i < indent; i++)
      {
         stream.print(" ");
      }

      stream.println(line);
   }


   /**
    * This method provides a convienent means to print out VarLists in a pretty class friendly form.
    * All variants of CreateControllerBase use this function during generation.
    *
    * @param stream PrintStream to output with.
    * @param list VarList to print.
    */
   @SuppressWarnings("unused")
   private void printVarBase(PrintStream stream, YoVariableList list)
   {
      if (list.size() == 0)
         return;

      // Joint Variables
      stream.print("   YoVariable ");

      for (int varNum = 0; varNum < list.size(); varNum++)
      {
         YoVariable<?> var = list.getVariable(varNum);

         if (varNum != 0)
         {
            if (varNum % 10 == 0)
            {
               stream.println(";");
               stream.print("   YoVariable ");
            }
            else
               stream.print(", ");
         }

         stream.print(var.getName());
      }

      stream.println(";");

   }

   public ArrayList<YoVariableList> createAllVarLists()
   {
      return getRobotsYoVariableRegistry().createVarListsIncludingChildren();
   }

   public ArrayList<RewoundListener> getSimulationRewoundListeners()
   {
      return getRobotsYoVariableRegistry().getAllSimulationRewoundListeners();
   }

   public ArrayList<RobotControllerAndParameters> getControllers()
   {
      return this.controllers;
   }

   public void setBodyExternalForcePoint(double fx, double fy, double fz)
   {
      kp_body.setForce(fx, fy, fz);
   }


   @Override
   public YoVariable<?> getVariable(String variableName)
   {
      return getRobotsYoVariableRegistry().getVariable(variableName);
   }

   @Override
   public boolean hasUniqueVariable(String variableName)
   {
      return getRobotsYoVariableRegistry().hasUniqueVariable(variableName);
   }

   @Override
   public ArrayList<YoVariable<?>> getAllVariables()
   {
      return getRobotsYoVariableRegistry().getAllVariablesIncludingDescendants();
   }

   @Override
   public YoVariable<?>[] getAllVariablesArray()
   {
      return getRobotsYoVariableRegistry().getAllVariablesArray();
   }

   @Override
   public YoVariable<?> getVariable(String nameSpaceEnding, String name)
   {
      return getRobotsYoVariableRegistry().getVariable(nameSpaceEnding, name);
   }

   @Override
   public boolean hasUniqueVariable(String nameSpaceEnding, String name)
   {
      return getRobotsYoVariableRegistry().hasUniqueVariable(nameSpaceEnding, name);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String nameSpaceEnding, String name)
   {
      return getRobotsYoVariableRegistry().getVariables(nameSpaceEnding, name);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String name)
   {
      return getRobotsYoVariableRegistry().getVariables(name);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(NameSpace nameSpace)
   {
      return getRobotsYoVariableRegistry().getVariables(nameSpace);
   }
   
   public ArrayList<Graphics3DObject> getStaticLinkGraphics()
   {
      return staticLinkGraphics;
   }
   
   public Link getLink(String linkName)
   {
      for(Joint rootJoint : rootJoints)
      {
         Link link = rootJoint.getLink(linkName);
         if (link != null) return link;
      }
      
      return null;
   }
   
   public void freezeJointAtZero(Joint jointToFreeze)
   {
      Vector3D jointToFreezeOffset = new Vector3D();
      jointToFreeze.getOffset(jointToFreezeOffset);
      System.out.println("jointToFreezeOffset = " + jointToFreezeOffset);
      Joint parentJoint = jointToFreeze.getParentJoint();
      parentJoint.removeChildJoint(jointToFreeze);

      Link parentLink = parentJoint.getLink();
      parentLink = Link.combineLinks(parentLink.getName(), parentLink, jointToFreeze.getLink(), jointToFreezeOffset);
      parentJoint.setLink(parentLink);
      
      ArrayList<Joint> jointsToMove = new ArrayList<Joint>();
      
      ArrayList<Joint> childrenJoints = jointToFreeze.getChildrenJoints();
      for (Joint childJoint : childrenJoints)
      {
         jointsToMove.add(childJoint);

         Vector3D childOffset = new Vector3D();
         childJoint.getOffset(childOffset);
         childOffset.add(jointToFreezeOffset);
         childJoint.changeOffsetVector(childOffset);
      }
      
      for (Joint jointToMove : jointsToMove)
      {
         jointToFreeze.removeChildJoint(jointToMove);
         parentJoint.addJoint(jointToMove);
      }
   }
   
   public boolean verifySetupProperly(double epsilon)
   {
      for (Joint rootJoint : rootJoints)
      {
         boolean rootJointSetupProperly = rootJoint.physics.verifySetupProperly(epsilon);
         if (!rootJointSetupProperly) return false;
      }
      
      return true;
   }

   public Joint getJoint(String name)
   {
      for (int i=0; i<rootJoints.size(); i++)
      {
         Joint rootJoint = rootJoints.get(i);
         Joint joint = rootJoint.recursivelyGetJoint(name);
         if (joint != null)
            return joint;
      }
      return null;
   }
   
//   public void resetup()
//   {
//      for (Joint rootJoint : rootJoints)
//      {
//         rootJoint.resetup();
//      }
//   }
}
