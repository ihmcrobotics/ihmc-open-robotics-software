package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;

public class FloatingObjectBoxRobot extends Robot
{
   private final FloatingJoint qrCodeJoint;

   public FloatingObjectBoxRobot(String imageResourcePath)
   {
      super(FloatingObjectBoxRobot.class.getSimpleName());

      qrCodeJoint = new FloatingJoint("object", "object", new Vector3D(), this);
      Link qrCodeLink = new Link("object");
      qrCodeLink.setMassAndRadiiOfGyration(1.0, 0.1, 0.1, 0.1);
      Graphics3DObject qrCodeLinkGraphics = new Graphics3DObject();
      //      qrCodeLinkGraphics.addCoordinateSystem(2.0);
      double cubeLength = 1.0;
      qrCodeLinkGraphics.translate(0.0, 0.0, -0.99 * cubeLength);
      AppearanceDefinition cubeAppearance = YoAppearance.Texture(imageResourcePath);
      qrCodeLinkGraphics.addCube(cubeLength * 0.98, cubeLength * 1.01, cubeLength * 0.98, YoAppearance.Yellow());

      boolean[] textureFaces = new boolean[] { true, true, false, false, false, false };
      qrCodeLinkGraphics.translate(0.0, 0.0, -0.01 * cubeLength);
      qrCodeLinkGraphics.addCube(cubeLength, cubeLength, cubeLength, cubeAppearance, textureFaces);

      qrCodeLink.setLinkGraphics(qrCodeLinkGraphics);
      qrCodeJoint.setLink(qrCodeLink);
      addRootJoint(qrCodeJoint);
      setGravity(0.0);
   }

   public void setPosition(Tuple3DBasics position)
   {
      qrCodeJoint.setPosition(position);
   }

   public void setPosition(double x, double y, double z)
   {
      qrCodeJoint.setPosition(x, y, z);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      qrCodeJoint.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setAngularVelocity(Vector3D angularVelocityInBody)
   {
      qrCodeJoint.setAngularVelocityInBody(angularVelocityInBody);
   }

   public void setLinearVelocity(Vector3D linearVelocityInWorld)
   {
      qrCodeJoint.setVelocity(linearVelocityInWorld);
   }
}
