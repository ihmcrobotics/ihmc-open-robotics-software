package us.ihmc.atlas.behaviors.scsSensorSimulation;

import org.apache.commons.lang3.SystemUtils;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.gui.AWTTools;

public class SensorOnlySimulation
{
   private final SimulationConstructionSet scs;

   public SensorOnlySimulation()
   {
      SensorOnlyRobot robot = new SensorOnlyRobot();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);

      SensorOnlyController controller = new SensorOnlyController(robot, yoGraphicsListRegistry, scs);
      robot.setController(controller);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      CameraConfiguration camera = new CameraConfiguration("camera");
      camera.setCameraMount("camera");
      scs.setupCamera(camera);

      addSphere(2.0, 2.0, scs);
      addSphere(-2.0, 2.0, scs);
      addSphere(2.0, -2.0, scs);
      addSphere(-2.0, -2.0, scs);

      controller.initialize();

      if (!SystemUtils.IS_OS_WINDOWS)
         scs.getGUI().getFrame().setSize(AWTTools.getDimensionOfSmallestScreenScaled(2.0 / 3.0));

      scs.startOnAThread();
   }

   public SimulationConstructionSet getSCS()
   {
      return scs;
   }

   public void addSphere(double x, double y, SimulationConstructionSet scs)
   {
      Graphics3DObject sphere = new Graphics3DObject();
      sphere.translate(x, y, 0.5);
      sphere.addSphere(1.0, YoAppearance.Gray());
      scs.addStaticLinkGraphics(sphere);
   }

   public static void main(String[] args)
   {
      new SensorOnlySimulation();
   }
}
