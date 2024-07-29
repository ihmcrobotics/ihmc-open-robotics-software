package us.ihmc.rdx.imgui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.graphics.glutils.GLFrameBuffer;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImGuiPlatformIO;
import imgui.flag.ImGuiMouseButton;
import imgui.gl3.ImGuiImplGl3;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;

/**
 * WARNING: Probably should not be used anymore.
 * @deprecated Use RDX3DSituatedImGuiPanel
 */
public class RDXSingleContext3DSituatedImGuiPanel implements RenderableProvider
{
   private final boolean ENABLE_EXPERIMENTAL_TRACKING = false;
   private final AtomicLong INDEX = new AtomicLong(0);
   private ModelInstance modelInstance = null;
   private ImGuiImplGl3 imGuiGl3;
   private int panelWidth;
   private int panelHeight;
   private Runnable renderImGuiWidgets;
   private RDXVRContext vrContext;
   private FrameBuffer frameBuffer;
   private float mousePosX;
   private float mousePosY;
   private boolean leftMouseDown;
   private final int metersToPixels = 10 * 100; // 10 pixels per centimeter
   private final float pixelsToMeters = 1.0f / (float) metersToPixels;
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final RigidBodyTransform graphicsXRightYDownToCenterXThroughZUpTransform = new RigidBodyTransform();
   private PlanarRegion planarRegion;
   private final ReferenceFrame centerXThroughZUpFrame
         = ReferenceFrameTools.constructFrameWithChangingTransformToParent("centerXThroughZUpFrame" + INDEX.getAndIncrement(),
                                                                           ReferenceFrame.getWorldFrame(),
                                                                           transform);
   private final FramePose3D desiredPose = new FramePose3D();
   private final RigidBodyTransform desiredTransformToHeadset = new RigidBodyTransform();
   private final FramePose3D currentPose = new FramePose3D();
   private final ReferenceFrame graphicsXRightYDownFrame
         = ReferenceFrameTools.constructFrameWithChangingTransformToParent("graphicsXRightYDownFrame" + INDEX.getAndIncrement(),
                                                                           centerXThroughZUpFrame,
                                                                           graphicsXRightYDownToCenterXThroughZUpTransform);
   private final FrameLine3D pickRay = new FrameLine3D();
   private final FramePoint3D pickIntersection = new FramePoint3D();
   private final Plane3D plane = new Plane3D();
   private ModelInstance centerFrameCoordinateFrame;
   private ModelInstance graphicsFrameCoordinateFrame;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D centerFrameCoordinateFramePose = new FramePose3D();
   private final FramePose3D graphicsFrameCoordinateFramePose = new FramePose3D();
   private final RigidBodyTransform gripOffsetTransform = new RigidBodyTransform();
   private boolean grippedLastTime = false;
   private final Stopwatch timerForFollowSpeed = new Stopwatch().start();

   public void create(int panelWidth, int panelHeight, Runnable renderImGuiWidgets, RDXVRContext vrContext)
   {
      this.panelWidth = panelWidth;
      this.panelHeight = panelHeight;
      this.renderImGuiWidgets = renderImGuiWidgets;
      this.vrContext = vrContext;

      ImGui.createContext();

      ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
      io.setMouseDrawCursor(true);
      ImGui.styleColorsLight();

      imGuiGl3 = new ImGuiImplGl3();
      imGuiGl3.init();

      centerFrameCoordinateFrame = RDXModelBuilder.createCoordinateFrameInstance(0.3);
      graphicsFrameCoordinateFrame = RDXModelBuilder.createCoordinateFrameInstance(0.3);

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Counter clockwise order
      // Draw so thumb faces away and index right
      float halfWidth = (float) panelWidth / 2.0f;
      float halfHeight = (float) panelHeight / 2.0f;
      Vector3 topLeftPosition = new Vector3(0.0f, halfWidth, halfHeight).scl(pixelsToMeters);
      Vector3 bottomLeftPosition = new Vector3(0.0f, halfWidth, -halfHeight).scl(pixelsToMeters);
      Vector3 bottomRightPosition = new Vector3(0.0f, -halfWidth, -halfHeight).scl(pixelsToMeters);
      Vector3 topRightPosition = new Vector3(0.0f, -halfWidth, halfHeight).scl(pixelsToMeters);
      Vector3 topLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 bottomLeftNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 bottomRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector3 topRightNormal = new Vector3(0.0f, 0.0f, 1.0f);
      Vector2 topLeftUV = new Vector2(0.0f, 1.0f);
      Vector2 bottomLeftUV = new Vector2(0.0f, 0.0f);
      Vector2 bottomRightUV = new Vector2(1.0f, 0.0f);
      Vector2 topRightUV = new Vector2(1.0f, 1.0f);
      meshBuilder.vertex(topLeftPosition, topLeftNormal, Color.WHITE, topLeftUV);
      meshBuilder.vertex(bottomLeftPosition, bottomLeftNormal, Color.WHITE, bottomLeftUV);
      meshBuilder.vertex(bottomRightPosition, bottomRightNormal, Color.WHITE, bottomRightUV);
      meshBuilder.vertex(topRightPosition, topRightNormal, Color.WHITE, topRightUV);
      meshBuilder.triangle((short) 3, (short) 0, (short) 1);
      meshBuilder.triangle((short) 1, (short) 0, (short) 3);
      meshBuilder.triangle((short) 1, (short) 2, (short) 3);
      meshBuilder.triangle((short) 3, (short) 2, (short) 1);

      Mesh mesh = meshBuilder.end();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();

      GLFrameBuffer.FrameBufferBuilder frameBufferBuilder = new GLFrameBuffer.FrameBufferBuilder(this.panelWidth, this.panelHeight);
      frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
      frameBuffer = frameBufferBuilder.build();
      Texture colorBufferTexture = frameBuffer.getColorBufferTexture();
      material.set(TextureAttribute.createDiffuse(colorBufferTexture));

      material.set(ColorAttribute.createDiffuse(Color.WHITE));
      modelBuilder.part(meshPart, material);

      Model model = modelBuilder.end();
      modelInstance = new ModelInstance(model);

      // set up graphicsXRightYDownToCenterXThroughZUpTransform
      graphicsXRightYDownToCenterXThroughZUpTransform.appendYawRotation(-Math.toRadians(90.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendPitchRotation(Math.toRadians(0.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendRollRotation(-Math.toRadians(90.0));
      graphicsXRightYDownToCenterXThroughZUpTransform.appendTranslation(-halfWidth * pixelsToMeters, -halfHeight * pixelsToMeters, 0.0f);
      graphicsXRightYDownFrame.update();

      plane.getNormal().set(Axis3D.X);

      updateDesiredPose(transform ->
      {
         transform.getTranslation().set(1.0f, 0.0f, 1.0f);
      });
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         pickRay.setToZero(controller.getXForwardZUpControllerFrame());
         pickRay.getDirection().set(Axis3D.X);
         pickRay.changeFrame(ReferenceFrame.getWorldFrame());

         pickIntersection.setToZero(ReferenceFrame.getWorldFrame());
         plane.intersectionWith(pickRay, pickIntersection);

         pickIntersection.changeFrame(graphicsXRightYDownFrame);

         float scaledX = Math.round((float) pickIntersection.getX() * metersToPixels);
         float scaledY = Math.round((float) pickIntersection.getY() * metersToPixels);

         if (scaledX > 0 && scaledX < panelWidth && scaledY > 0 && scaledY < panelHeight)
         {
            mousePosX = scaledX;
            mousePosY = scaledY;
            leftMouseDown = controller.getClickTriggerActionData().bState();
         }
         else
         {
            leftMouseDown = false;
         }

         if ((grippedLastTime || controller.getXForwardZUpPose().getPosition().distance(centerFrameCoordinateFramePose.getPosition()) < 0.05)
             && controller.getGripActionData().x() > 0.9)
         {
            if (!grippedLastTime) // set up offset
            {
               centerFrameCoordinateFramePose.changeFrame(controller.getXForwardZUpControllerFrame());
               centerFrameCoordinateFramePose.get(gripOffsetTransform);
               centerFrameCoordinateFramePose.changeFrame(ReferenceFrame.getWorldFrame());
            }

            updateDesiredPose(transform ->
            {
               transform.set(gripOffsetTransform);
               controller.getXForwardZUpControllerFrame().getTransformToWorldFrame().transform(transform);
            });

            grippedLastTime = true;
         }
         else
         {
            grippedLastTime = false;
         }


         if (controller.getAButtonActionData().bChanged() && !controller.getAButtonActionData().bState())
         {
            updateDesiredPose(transform ->
            {
               desiredPose.setToZero(vrContext.getHeadset().getXForwardZUpHeadsetFrame());
               desiredPose.getPosition().set(1.0, 0.0, 0.0);
               desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
               desiredPose.get(transform);
            });
         }
      });

      // current moves towards the desired
      currentPose.setToZero(centerXThroughZUpFrame);
      currentPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPose.setToZero(vrContext.getHeadset().getXForwardZUpHeadsetFrame());
      desiredPose.set(desiredTransformToHeadset);
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());
      updateCurrentPose(transform ->
      {
         if (ENABLE_EXPERIMENTAL_TRACKING)
         {
//            transform.getTranslation().set(desiredPose.getPosition());
//            transform.getRotation().set(desiredPose.getOrientation());
            if (currentPose.getPosition().distance(desiredPose.getPosition()) > 0.003 && currentPose.getOrientation().distance(desiredPose.getOrientation()) > Math.toRadians(0.5))
            {

            }

            double maxLinearSpeed = 0.1;
            double maxAngularSpeed = 1.0;
            double delta = timerForFollowSpeed.lap();
            double linearDistance = currentPose.getPosition().distance(desiredPose.getPosition());
            double angularDistance = currentPose.getOrientation().distance(desiredPose.getOrientation());
            double distanceToMove = linearDistance * maxLinearSpeed * delta;
            double angleToMove = angularDistance * maxAngularSpeed * delta;
            if (linearDistance > 1e-5)
            {
               if (linearDistance < distanceToMove)
               {
                  transform.getTranslation().set(desiredPose.getPosition());
               }
               else
               {
                  currentPose.getPosition().interpolate(desiredPose.getPosition(), distanceToMove / linearDistance);
                  transform.getTranslation().set(currentPose.getPosition());
               }
            }
            if (angularDistance > 1e-5)
            {
               if (angularDistance < angleToMove)
               {
                  transform.getRotation().set(desiredPose.getOrientation());
               }
               else
               {
                  currentPose.getOrientation().interpolate(desiredPose.getOrientation(), angleToMove / angularDistance);
                  transform.getRotation().set(currentPose.getOrientation());
               }
            }
         }
         else
         {
         }
      });
   }

   public void render()
   {
      ImGuiIO io = ImGui.getIO();
      io.setDisplaySize(panelWidth, panelHeight);
      io.setDisplayFramebufferScale(1.0f, 1.0f);
      io.setMousePos(mousePosX, mousePosY);
      io.setMouseDown(ImGuiMouseButton.Left, leftMouseDown);

      ImGuiPlatformIO platformIO = ImGui.getPlatformIO();
      platformIO.resizeMonitors(0);
      platformIO.pushMonitors(0.0f, 0.0f, panelWidth, panelHeight, 0.0f, 0.0f, panelWidth, panelHeight, 1.0f);

      float deltaTime = Gdx.app.getGraphics().getDeltaTime();
      io.setDeltaTime(deltaTime > 0.0f ? deltaTime : 1.0f / 60.0f);

      ImGui.newFrame();
      ImGui.pushFont(ImGuiTools.getSmallFont());

      ImGui.setNextWindowPos(0.0f, 0.0f);
      ImGui.setNextWindowSize(panelWidth, panelHeight);
      ImGui.begin("Main Panel");
      renderImGuiWidgets.run();
      ImGui.end();

      ImGui.popFont();
      ImGui.render();

      frameBuffer.begin();
      ImGuiTools.glClearDarkGray();
      imGuiGl3.renderDrawData(ImGui.getDrawData());
      frameBuffer.end();

      centerFrameCoordinateFramePose.setToZero(centerXThroughZUpFrame);
      centerFrameCoordinateFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      centerFrameCoordinateFramePose.get(tempTransform);
      LibGDXTools.toLibGDX(tempTransform, centerFrameCoordinateFrame.transform);
      graphicsFrameCoordinateFramePose.setToZero(graphicsXRightYDownFrame);
      graphicsFrameCoordinateFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      graphicsFrameCoordinateFramePose.get(tempTransform);
      LibGDXTools.toLibGDX(tempTransform, graphicsFrameCoordinateFrame.transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
//      centerFrameCoordinateFrame.getRenderables(renderables, pool);
//      graphicsFrameCoordinateFrame.getRenderables(renderables, pool);
   }

   public void updateDesiredPose(Consumer<RigidBodyTransform> transformUpdater)
   {
      updateCurrentPose(transformUpdater);

      desiredPose.setToZero(centerXThroughZUpFrame);
      desiredPose.changeFrame(vrContext.getHeadset().getXForwardZUpHeadsetFrame());
      desiredPose.get(desiredTransformToHeadset);
   }

   private void updateCurrentPose(Consumer<RigidBodyTransform> transformUpdater)
   {
      transform.setToZero();
      transformUpdater.accept(transform);

      plane.setToZero();
      plane.getNormal().set(Axis3D.X);
      plane.applyTransform(transform);
      LibGDXTools.toLibGDX(transform, modelInstance.transform);
      centerXThroughZUpFrame.update();

      desiredPose.setToZero(centerXThroughZUpFrame);
      desiredPose.changeFrame(vrContext.getHeadset().getXForwardZUpHeadsetFrame());
   }

   public Plane3D getPlane()
   {
      return plane;
   }

   public void dispose()
   {
      frameBuffer.dispose();
   }
}
