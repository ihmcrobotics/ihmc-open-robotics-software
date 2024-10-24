package us.ihmc.avatar;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.simulationConstructionSetTools.util.environments.CTTSOObstacleCourseEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.DynamicIntegrationMethod;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class CTTSOEnvironmentViewer
{
   private final boolean SHOW_GRAPHICS = true;
   private final boolean SHOW_COLLISION_TERRAIN = false;

   private final double[] CAMFIX = {0, 0, 0};
   private final double[] CAMPOS = {-14, -7, 12};

   Robot robot = new Robot("NotARobot");
   private double gravity = -9.81;

   CTTSOEnvironmentViewer()
   {
      double simulateDT = .001;
      int initialBufferSize = 16342;

      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new CTTSOObstacleCourseEnvironment();

      GroundProfile3D groundProfile3D = commonAvatarEnvironmentInterface.getTerrainObject3D();

      robot.setGravity(gravity);

      double groundKz = 500;
      double groundBz = 300.0;
      double groundKxy = 50000.0;
      double groundBxy = 2000.0;

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, groundKxy, groundBxy, groundKz, groundBz,
                                                                                 robot.getRobotsYoRegistry());

      groundContactModel.setGroundProfile3D(groundProfile3D);

      robot.setGroundContactModel(groundContactModel);
      robot.setDynamicIntegrationMethod(DynamicIntegrationMethod.EULER_DOUBLE_STEPS);
      JMEGraphics3DAdapter graphicsAdapter = new JMEGraphics3DAdapter();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(initialBufferSize);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, graphicsAdapter, parameters);

      scs.setDT(simulateDT, 16342);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);

      scs.setCameraFix(CAMFIX[0], CAMFIX[1], CAMFIX[2]);
      scs.setCameraPosition(CAMPOS[0], CAMPOS[1], CAMPOS[2]);

      scs.setGroundVisible(SHOW_COLLISION_TERRAIN);

      if (SHOW_GRAPHICS)
      {
         TerrainObject3D environmentTerrain3D = commonAvatarEnvironmentInterface.getTerrainObject3D();
         if (environmentTerrain3D != null)
            scs.addStaticLinkGraphics(environmentTerrain3D.getLinkGraphics());
      }

      Thread simThread = new Thread(scs, "SCS simulation thread");

      simThread.start();

   }

   public static void main(String[] args)
   {
      new CTTSOEnvironmentViewer();
   }
}
