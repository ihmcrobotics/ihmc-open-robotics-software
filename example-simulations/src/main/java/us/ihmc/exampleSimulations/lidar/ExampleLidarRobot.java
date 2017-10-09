package us.ihmc.exampleSimulations.lidar;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.simulationconstructionset.CameraMount;
import us.ihmc.simulationconstructionset.GimbalJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;

public class ExampleLidarRobot extends Robot
{
   private final LidarScanParameters lidarScanParameters;
   private final GimbalJoint gimbalJoint;

   public ExampleLidarRobot()
   {
      super("ExampleLidarRobot");

      double height = 0.2;
      double radius = 0.05;

      gimbalJoint = new GimbalJoint("gimbalZ", "gimbalX", "gimbalY", new Vector3D(0.0, 0.0, 1.0), this, Axis.Z, Axis.X, Axis.Y);
      Link link = new Link("lidar");
      link.setMassAndRadiiOfGyration(1.0, radius, radius, radius);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      link.setLinkGraphics(linkGraphics);
      gimbalJoint.setLink(link);
      gimbalJoint.setDamping(1.0);

      CameraMount robotCam = new CameraMount("camera", new Vector3D(radius + 0.001, 0.0, height / 2.0), this);
      gimbalJoint.addCameraMount(robotCam);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3D(radius + 0.001, 0.0, height / 2.0));
      lidarScanParameters = new LidarScanParameters(720, (float) (-Math.PI / 2), (float) (Math.PI / 2), 0f, 0.1f, 30.0f, 0f);
      LidarMount lidarMount = new LidarMount(transform, lidarScanParameters, "lidar");

      gimbalJoint.addLidarMount(lidarMount);

      linkGraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
      linkGraphics.translate(0, 0, -0.1);
      link.setLinkGraphics(linkGraphics);
      this.addRootJoint(gimbalJoint);
   }

   public GimbalJoint getLidarZJoint()
   {
      return gimbalJoint;
   }

   public PinJoint getLidarXJoint()
   {
      return (PinJoint) gimbalJoint.getChildrenJoints().get(0);
   }

   public LidarScanParameters getLidarScanParameters()
   {
      return lidarScanParameters;
   }
}
