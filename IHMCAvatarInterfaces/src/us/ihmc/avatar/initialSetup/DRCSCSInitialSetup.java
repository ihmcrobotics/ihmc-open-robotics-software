package us.ihmc.avatar.initialSetup;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.DynamicIntegrationMethod;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class DRCSCSInitialSetup
{
   private static final boolean SHOW_WORLD_COORDINATE_FRAME = false;

   private final double simulateDT;
   private int recordFrequency = 50;
   private boolean drawGroundProfile = false;
   private boolean enableGroundSlipping = false;
   private double groundAlphaStick = Double.NaN;
   private double groundAlphaSlip = Double.NaN;

   private int simulationDataBufferSize = 16000;
   private Vector3D gravity = new Vector3D(0.0, 0.0, -9.81);
   private boolean runMultiThreaded = true;

   private boolean usePerfectSensors = false;
   private boolean initializeEstimatorToActual = false;

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
      if (enableGroundSlipping)
         groundContactModel.enableSlipping();
      if (Double.isFinite(groundAlphaStick) && Double.isFinite(groundAlphaSlip))
         groundContactModel.setAlphaStickSlip(groundAlphaStick, groundAlphaSlip);

      if (groundProfile3D != null)
         groundContactModel.setGroundProfile3D(groundProfile3D);
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

   public void disableGroundSlipping()
   {
      enableGroundSlipping = false;
      groundAlphaStick = Double.NaN;
      groundAlphaSlip = Double.NaN;
   }

   public void enableGroundSlipping(double alphaStick, double alphaSlip)
   {
      enableGroundSlipping = true;
      groundAlphaStick = alphaStick;
      groundAlphaSlip = alphaSlip;
   }

   private Graphics3DObject createGroundLinkGraphicsFromGroundProfile(GroundProfile3D groundProfile)
   {
      Graphics3DObject texturedGroundLinkGraphics = new Graphics3DObject();

      HeightMap heightMap = null;

      if (groundProfile != null)
         heightMap = groundProfile.getHeightMapIfAvailable();

      texturedGroundLinkGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.DarkGreen());

      return texturedGroundLinkGraphics;
   }

   public void initializeSimulation(SimulationConstructionSet scs)
   {
      scs.setDT(simulateDT, recordFrequency);

      if (drawGroundProfile)
      {
         scs.addStaticLinkGraphics(createGroundLinkGraphicsFromGroundProfile(groundProfile3D));
      }

      if (SHOW_WORLD_COORDINATE_FRAME)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.3);
         scs.addStaticLinkGraphics(linkGraphics);
      }

      /*
       * This makes sure that the initial values of all YoVariables that are added to the scs (i.e.
       * at index 0 of the data buffer) are properly stored in the data buffer
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
      if (recordFrequency < 1)
         recordFrequency = 1;
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
