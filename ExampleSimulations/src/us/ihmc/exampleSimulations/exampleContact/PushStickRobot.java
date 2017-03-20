package us.ihmc.exampleSimulations.exampleContact;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;

public class PushStickRobot extends Robot
{
   private static final long serialVersionUID = 6953401273041281579L;

   private static final double MASS = 2.0;

   private static final double Ixx = 5.0;
   private static final double Iyy = 5.0;
   private static final double Izz = 5.0;

   private static final double LENGTH = 1.0;
   private static final double RADIUS = 0.01;

   private final SliderJoint xJoint, yJoint, zJoint;
   private final PinJoint yawJoint, pitchJoint;
   private final SliderJoint pushJoint;
   
   private final KinematicPoint kp_PushStickBackEnd;
   private final ExternalForcePoint ef_PushStickTip;
   
   public PushStickRobot()
   {
      super("PushStickRobot");
      
      this.setGravity(0.0);
      
      xJoint = new SliderJoint("pushStickX", new Vector3D(), this, Axis.X);
      Link xLink = new Link("xLink");
      xJoint.setLink(xLink);
      
      yJoint = new SliderJoint("pushStickY", new Vector3D(), this, Axis.Y);
      Link yLink = new Link("yLink");
      yJoint.setLink(yLink);

      zJoint = new SliderJoint("pushStickZ", new Vector3D(), this, Axis.Z);
      Link zLink = new Link("zLink");
      zLink.setMass(100.0);
      zLink.setMomentOfInertia(10.0, 10.0, 10.0);
      Graphics3DObject zLinkLinkGraphics = new Graphics3DObject();
      zLinkLinkGraphics.addCube(0.1, 0.1, 0.1, YoAppearance.Gray());
      zLink.setLinkGraphics(zLinkLinkGraphics);
      zJoint.setLink(zLink);
      
      yawJoint = new PinJoint("pushStickYaw", new Vector3D(), this, Axis.Z);
      Link yawLink = new Link("yawLink");
      yawJoint.setLink(yawLink);
      
      pitchJoint = new PinJoint("pushStickPitch", new Vector3D(), this, Axis.Y);
      Link pitchLink = new Link("pitchLink");
      pitchJoint.setLink(pitchLink);
      
      pushJoint = new SliderJoint("pushStickPush", new Vector3D(), this, Axis.X);
      
      Link link = new Link("pushStick");
      
      link.setMass(MASS);
      link.setComOffset(new Vector3D(-LENGTH/2.0, 0.0, 0.0));
      link.setMomentOfInertia(Ixx, Iyy, Izz);
            
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -LENGTH);
      linkGraphics.addCylinder(LENGTH, RADIUS);
      
      link.setLinkGraphics(linkGraphics);
//      link.addCoordinateSystemToCOM(0.2);

      pushJoint.setLink(link);
      
      ef_PushStickTip = new ExternalForcePoint("ef_PushStickTip", new Vector3D(), this);
      pushJoint.addExternalForcePoint(ef_PushStickTip);
      kp_PushStickBackEnd = new ExternalForcePoint("kp_PushStickBackEnd", new Vector3D(-LENGTH/2.0, 0.0, 0.0), this);
      pushJoint.addKinematicPoint(kp_PushStickBackEnd);
      
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoGraphicPosition pushStickForcePosition = new YoGraphicPosition("pushStickForce", ef_PushStickTip.getYoPosition(), 0.01, YoAppearance.Red());
      YoGraphicVector pushStickForceVector = new YoGraphicVector("forceToPushOnSphere", ef_PushStickTip.getYoPosition(), ef_PushStickTip.getYoForce(), 0.1, YoAppearance.Red());
      YoGraphicPosition pushStickBackEndGraphicPosition = new YoGraphicPosition("pushStickBackEnd", kp_PushStickBackEnd.getYoPosition(), 0.01, YoAppearance.Black());

      yoGraphicsListRegistry.registerYoGraphic("PushStickRobot", pushStickForcePosition);
      yoGraphicsListRegistry.registerYoGraphic("PushStickRobot", pushStickForceVector);
      yoGraphicsListRegistry.registerYoGraphic("PushStickRobot", pushStickBackEndGraphicPosition);
      
      this.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
      addRootJoint(xJoint);
      xJoint.addJoint(yJoint);
      yJoint.addJoint(zJoint);
      zJoint.addJoint(yawJoint);
      yawJoint.addJoint(pitchJoint);
      pitchJoint.addJoint(pushJoint);
      
      xJoint.setInitialState(-1.0, 0.0);
      yJoint.setInitialState(0.0, 0.0);
      zJoint.setInitialState(0.3, 0.0);
      yawJoint.setInitialState(0.0, 0.0);
      pitchJoint.setInitialState(0.0, 0.0);
      pushJoint.setInitialState(0.0, 0.0);      
   }

   
   Point3D tempPoint3d = new Point3D();
   public void getPushStickUnitDirectionInWorld(Vector3D directionInWorldToPack)
   {
      ef_PushStickTip.getPosition(directionInWorldToPack);
      kp_PushStickBackEnd.getPosition(tempPoint3d);
      directionInWorldToPack.sub(tempPoint3d);
      directionInWorldToPack.normalize();
   }

   public ExternalForcePoint getPushStickTipPoint()
   {
      return ef_PushStickTip;
   }
  
}
