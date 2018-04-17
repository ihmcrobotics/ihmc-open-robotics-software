package us.ihmc.robotEnvironmentAwareness.simulation;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;

public class SimpleLidarRobot extends Robot
{
   private final LidarScanParameters lidarScanParameters;
   
   private final FloatingJoint rootJoint;
   private final PinJoint lidarJoint;
   
   public SimpleLidarRobot()
   {
      super("SimpleLidarRobot");

      double height = 0.2;
      double radius = 0.05;

      setDynamic(false);

      rootJoint = new FloatingJoint("rootJoint", new Vector3D(0.0, 0.0, 1.0), this);
      rootJoint.setLink(new Link("Dummy"));
      rootJoint.setDynamic(false);
      addRootJoint(rootJoint);

      lidarJoint = new PinJoint("lidarJoint", new Vector3D(), this, Axis.X);
      
      Link link = new Link("lidar");
      link.setMassAndRadiiOfGyration(1.0, radius, radius, radius);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      link.setLinkGraphics(linkGraphics);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3D(radius + 0.001, 0.0, height / 2.0));
      lidarScanParameters = new LidarScanParameters(720, -Math.PI / 2.0f, Math.PI / 2.0f, 0f, 0.1f, 30.0f, 0f);
      LidarMount lidarMount = new LidarMount(transform, lidarScanParameters, "lidar");
      lidarJoint.addLidarMount(lidarMount);

      linkGraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
//      linkGraphics.addCone(1.0, 0.1);
      linkGraphics.translate(0, 0, -0.1);
      link.setLinkGraphics(linkGraphics);
      lidarJoint.setLink(link);
      rootJoint.addJoint(lidarJoint);
   }

   public FloatingJoint getRootJoint()
   {
      return rootJoint;
   }

   public PinJoint getLidarJoint()
   {
      return lidarJoint;
   }

   public LidarScanParameters getLidarScanParameters()
   {
      return lidarScanParameters;
   }
}
