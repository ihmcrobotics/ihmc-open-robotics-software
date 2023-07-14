package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRTeleporter
{
   private boolean preparingToTeleport = false;

   private ModelInstance lineModel;
   private ModelInstance ring;
   private ModelInstance arrow;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D pickRayPose = new FramePose3D();
   private final FrameLine3D pickRay = new FrameLine3D();
   private final FramePose3D currentPlayArea = new FramePose3D();
   private final Plane3D currentPlayAreaPlane = new Plane3D();
   private final FrameVector3D controllerZAxisVector = new FrameVector3D();
   private final Point3D controllerZAxisProjectedToPlanePoint = new Point3D();
   private final Point3D planeRayIntesection = new Point3D();
   private final Vector3D orientationDeterminationVector = new Vector3D();
   private final FramePose3D proposedTeleportPose = new FramePose3D();
   private final RigidBodyTransform xyYawHeadsetToTeleportTransform = new RigidBodyTransform();
   private double lineLength = 1.0;
   private final Color color = Color.WHITE;
   private double lastTouchpadY = Double.NaN;

   // Reference frame of mid-feet to be set from syncedRobot (used for teleporting vr viewPoint to robot position)
   private ReferenceFrame robotMidFeetZUpReferenceFrame = null;

   public void create(RDXVRContext context)
   {
      double ringThickness = 0.005;
      double innerRadius = 0.4;
      double outerRadius = 0.5;
      lineModel = RDXModelBuilder.buildModelInstance(this::buildLineMesh, "line");
      ring = RDXModelBuilder.buildModelInstance(meshBuilder ->
      {
         meshBuilder.addHollowCylinder(ringThickness, outerRadius, innerRadius, new Point3D(), color);
      }, "ring");
      arrow = RDXModelBuilder.buildModelInstance(meshBuilder ->
      {
         Point3D offset = new Point3D();
         offset.setX(outerRadius + 0.05);
         YawPitchRoll orientation = new YawPitchRoll();
         orientation.setYaw(Math.toRadians(-90.0));
         orientation.setPitch(Math.toRadians(0.0));
         orientation.setRoll(Math.toRadians(-90.0));
         meshBuilder.addIsoscelesTriangularPrism(0.2, 0.2, ringThickness, offset, orientation, color);
      }, "arrow");
      context.addVRInputProcessor(this::processVRInput);
   }

   private void buildLineMesh(RDXMultiColorMeshBuilder meshBuilder)
   {
      meshBuilder.addLine(0.0, 0.0, 0.0, lineLength, 0.0, 0.0, 0.005, color);
   }

   private void processVRInput(RDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
     {
        InputDigitalActionData bButton = controller.getBButtonActionData();
        InputDigitalActionData joystickButton = controller.getJoystickPressActionData();

        if(bButton.bChanged() && bButton.bState()) //pressed B button
           preparingToTeleport = true;
        else if(bButton.bChanged() && !bButton.bState())
        {
           vrContext.teleport(teleportIHMCZUpToIHMCZUpWorld ->
           {
              xyYawHeadsetToTeleportTransform.setIdentity();
              vrContext.getHeadset().runIfConnected(headset -> // teleport such that your headset ends up where you're trying to go
              {
                 headset.getXForwardZUpHeadsetFrame().getTransformToDesiredFrame(xyYawHeadsetToTeleportTransform, vrContext.getTeleportFrameIHMCZUp());
                 xyYawHeadsetToTeleportTransform.getTranslation().setZ(0.0);
                 xyYawHeadsetToTeleportTransform.getRotation().setYawPitchRoll(xyYawHeadsetToTeleportTransform.getRotation().getYaw(), 0.0, 0.0);
              });
              teleportIHMCZUpToIHMCZUpWorld.set(xyYawHeadsetToTeleportTransform);
              teleportIHMCZUpToIHMCZUpWorld.invert();
              proposedTeleportPose.get(tempTransform);
              tempTransform.transform(teleportIHMCZUpToIHMCZUpWorld);
           });
           preparingToTeleport = false;
        }

        // Pressed right joystick button
        if (robotMidFeetZUpReferenceFrame != null && joystickButton.bChanged() && !joystickButton.bState())
        {
           snapToMidFeetZUp(vrContext);
        }
        else if (preparingToTeleport) // holding B button
        {
           // Ray teleport
           pickRay.set(controller.getPickRay());

           pickRayPose.setToZero(controller.getPickPoseFrame());
           pickRayPose.changeFrame(ReferenceFrame.getWorldFrame());

           currentPlayArea.setToZero(vrContext.getTeleportFrameIHMCZUp());
           currentPlayArea.changeFrame(ReferenceFrame.getWorldFrame());

           currentPlayAreaPlane.getNormal().set(Axis3D.Z);
           currentPlayAreaPlane.getPoint().set(currentPlayArea.getPosition());

           currentPlayAreaPlane.intersectionWith(pickRay, planeRayIntesection);

           controllerZAxisVector.setIncludingFrame(controller.getPickPoseFrame(), Axis3D.Z);
           controllerZAxisVector.changeFrame(ReferenceFrame.getWorldFrame());

           controllerZAxisProjectedToPlanePoint.set(planeRayIntesection);
           controllerZAxisProjectedToPlanePoint.add(controllerZAxisVector);
           currentPlayAreaPlane.orthogonalProjection(controllerZAxisProjectedToPlanePoint);

           orientationDeterminationVector.sub(controllerZAxisProjectedToPlanePoint, planeRayIntesection);

           proposedTeleportPose.setToZero(ReferenceFrame.getWorldFrame());
           proposedTeleportPose.getPosition().set(planeRayIntesection);
           EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X,
                                                                      orientationDeterminationVector,
                                                                      proposedTeleportPose.getOrientation());

           lineLength = pickRayPose.getPosition().distance(proposedTeleportPose.getPosition());
           RDXModelBuilder.rebuildMesh(lineModel.nodes.get(0), this::buildLineMesh);

           pickRayPose.get(tempTransform);
           LibGDXTools.toLibGDX(tempTransform, lineModel.transform);
           proposedTeleportPose.get(tempTransform);
           LibGDXTools.toLibGDX(tempTransform, ring.transform);
           LibGDXTools.toLibGDX(tempTransform, arrow.transform);
        }

        if (controller.getTouchpadTouchedActionData().bState())
        {
           double y = controller.getTouchpadActionData().y();
           if (!Double.isNaN(lastTouchpadY))
           {
              double delta = y - lastTouchpadY;
              vrContext.teleport(teleportIHMCZUpToIHMCZUpWorld -> teleportIHMCZUpToIHMCZUpWorld.getTranslation().addZ(delta * 0.3));
           }
           lastTouchpadY = y;
        }
        else
        {
           lastTouchpadY = Double.NaN;
        }
     });
   }

   private void snapToMidFeetZUp(RDXVRContext vrContext)
   {
      vrContext.teleport(teleportIHMCZUpToIHMCZUpWorld ->
      {
         xyYawHeadsetToTeleportTransform.setIdentity();
         vrContext.getHeadset().runIfConnected(headset -> // teleport such that your headset ends up where you're trying to go
         {
            headset.getXForwardZUpHeadsetFrame().getTransformToDesiredFrame(xyYawHeadsetToTeleportTransform, vrContext.getTeleportFrameIHMCZUp());
            xyYawHeadsetToTeleportTransform.getTranslation().setZ(0.0);
            xyYawHeadsetToTeleportTransform.getRotation().setYawPitchRoll(xyYawHeadsetToTeleportTransform.getRotation().getYaw(), 0.0, 0.0);
      });
      teleportIHMCZUpToIHMCZUpWorld.set(xyYawHeadsetToTeleportTransform);
      teleportIHMCZUpToIHMCZUpWorld.invert();
      // set tempTransform to incoming rigidbodyTransform.
      tempTransform.set(robotMidFeetZUpReferenceFrame.getTransformToWorldFrame());
      tempTransform.transform(teleportIHMCZUpToIHMCZUpWorld);
      });
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (preparingToTeleport)
      {
         // render stuff
         lineModel.getRenderables(renderables, pool);
         ring.getRenderables(renderables, pool);
         arrow.getRenderables(renderables, pool);
      }
   }

   public void setRobotMidFeetZUpReferenceFrame(ReferenceFrame robotMidFeetZUpReferenceFrame)
   {
      this.robotMidFeetZUpReferenceFrame = robotMidFeetZUpReferenceFrame;
   }
}
