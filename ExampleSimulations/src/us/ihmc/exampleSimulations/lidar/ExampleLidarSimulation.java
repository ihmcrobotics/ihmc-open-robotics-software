package us.ihmc.exampleSimulations.lidar;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ExampleLidarSimulation
{
   public ExampleLidarSimulation()
   {
      ExampleLidarRobot robot = new ExampleLidarRobot();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);

      ExampleLidarController controller = new ExampleLidarController(robot, yoGraphicsListRegistry, scs);
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

      scs.startOnAThread();
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
      new ExampleLidarSimulation();
   }
}
