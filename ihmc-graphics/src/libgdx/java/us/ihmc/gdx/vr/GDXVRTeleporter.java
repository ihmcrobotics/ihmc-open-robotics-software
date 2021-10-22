package us.ihmc.gdx.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVRTeleporter
{
   private boolean preparingToTeleport = false;

   private ModelInstance lineModel;
   private ModelInstance ring;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D linePose = new FramePose3D();
   private FrameLine3D frameLine;

   public void create()
   {
      frameLine = new FrameLine3D(ReferenceFrame.getWorldFrame());
      frameLine.getDirection().setX(1.0);
      lineModel = GDXModelPrimitives.buildModelInstance(meshBuilder ->
      {
         meshBuilder.addLine(0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.003, Color.BLUE);
      }, "line");


      ring = GDXModelPrimitives.buildModelInstance(meshBuilder ->
      {
         meshBuilder.addHollowCylinder(0.005, 0.5, 0.4, new Point3D(), Color.BLUE);
      }, "ring");
      

   }

   public void processVRInput(GDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         InputDigitalActionData bButton = controller.getBButtonActionData();
         preparingToTeleport = bButton.bState();

         linePose.setToZero(controller.getXForwardZUpControllerFrame());
         linePose.changeFrame(ReferenceFrame.getWorldFrame());
         linePose.get(tempTransform);
         GDXTools.toGDX(tempTransform, lineModel.transform);
      });
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (preparingToTeleport)
      {
         // render stuff
         lineModel.getRenderables(renderables, pool);
      }
   }
}
