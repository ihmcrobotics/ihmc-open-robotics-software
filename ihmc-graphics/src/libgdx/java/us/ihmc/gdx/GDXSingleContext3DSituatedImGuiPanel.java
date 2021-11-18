package us.ihmc.gdx;

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
import imgui.ImFont;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImGuiPlatformIO;
import imgui.flag.ImGuiMouseButton;
import imgui.gl3.ImGuiImplGl3;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRContext;

import java.util.function.Consumer;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;

public class GDXSingleContext3DSituatedImGuiPanel implements RenderableProvider
{
   private ModelInstance modelInstance = null;
   private ImGuiImplGl3 imGuiGl3;
   private int panelWidth;
   private int panelHeight;
   private Runnable renderImGuiWidgets;
   private FrameBuffer frameBuffer;
   private float mousePosX;
   private float mousePosY;
   private boolean leftMouseDown;
   private ImFont font;
   private final int pixelsPerMeter = 10 * 100; // 10 pixels per centimeter
   private final float metersPerPixel = 1.0f / (float) pixelsPerMeter;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public void create(int panelWidth, int panelHeight, Runnable renderImGuiWidgets)
   {
      this.panelWidth = panelWidth;
      this.panelHeight = panelHeight;
      this.renderImGuiWidgets = renderImGuiWidgets;

      ImGui.createContext();

      ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
      io.setMouseDrawCursor(true);
      font = ImGuiTools.setupFonts(io);
      //            ImGui.styleColorsLight();

      imGuiGl3 = new ImGuiImplGl3();
      imGuiGl3.init();

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Counter clockwise order
      // Draw so thumb faces away and index right
      float halfWidth = (float) panelWidth / 2.0f;
      float halfHeight = (float) panelHeight / 2.0f;
      Vector3 topLeftPosition = new Vector3(0.0f, halfWidth, halfHeight).scl(metersPerPixel);
      Vector3 bottomLeftPosition = new Vector3(0.0f, halfWidth, -halfHeight).scl(metersPerPixel);
      Vector3 bottomRightPosition = new Vector3(0.0f, -halfWidth, -halfHeight).scl(metersPerPixel);
      Vector3 topRightPosition = new Vector3(0.0f, -halfWidth, halfHeight).scl(metersPerPixel);
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
   }

   public void processVRInput(GDXVRContext vrContext)
   {
//      mousePosX = input.getMousePosX();
//      mousePosY = input.getMousePosY();
//      leftMouseDown = ImGui.isMouseDown(ImGuiMouseButton.Left);
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
      ImGui.pushFont(font);

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
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void updatePose(Consumer<RigidBodyTransform> transformUpdater)
   {
      tempTransform.setToZero();
      transformUpdater.accept(tempTransform);
      GDXTools.toGDX(tempTransform, modelInstance.transform);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void dispose()
   {
      frameBuffer.dispose();
   }
}
