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
import imgui.*;
import imgui.flag.ImGuiMouseButton;
import imgui.gl3.ImGuiImplGl3;
import imgui.internal.ImGuiContext;
import org.lwjgl.opengl.GL41;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;

import java.util.ArrayList;

import static com.badlogic.gdx.graphics.VertexAttributes.Usage.*;
import static com.badlogic.gdx.graphics.VertexAttributes.Usage.TextureCoordinates;

public class GDXMultiContext3DSituatedImGuiPanelManager implements RenderableProvider
{
   private final ArrayList<GDX3DSituatedImGuiPanel> panels = new ArrayList<>();
   private ImFont font = null;
   private ModelInstance modelInstance = null;
   private ImGuiImplGl3 imGuiGl3;
   private ImGuiContext savedImGuiContext = null;
   private ImGuiContext virtualImGuiContext = null;
   private int width = 800;
   private int height = 600;
   private FrameBuffer frameBuffer;
   private float mousePosX;
   private float mousePosY;
   private boolean leftMouseDown;

   public void create(ImGuiImplGl3 imGuiGl3, ImFont font)
   {
      this.imGuiGl3 = imGuiGl3;
      this.font = font;

      savedImGuiContext = ImGui.getCurrentContext();
      virtualImGuiContext = ImGui.createContext(ImGui.getIO().getFonts());
      ImGui.setCurrentContext(virtualImGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
      io.setMouseDrawCursor(true);

      ImGui.setCurrentContext(savedImGuiContext);

      ModelBuilder modelBuilder = new ModelBuilder();
      modelBuilder.begin();

      MeshBuilder meshBuilder = new MeshBuilder();
      meshBuilder.begin(Position | Normal | ColorUnpacked | TextureCoordinates, GL41.GL_TRIANGLES);

      // Counter clockwise order
      // Draw so thumb faces away and index right
      Vector3 topLeftPosition = new Vector3(0.0f, 0.0f, 0.0f);
      Vector3 bottomLeftPosition = new Vector3(0.0f, (float) height, 0.0f);
      Vector3 bottomRightPosition = new Vector3((float) width, (float) height, 0.0f);
      Vector3 topRightPosition = new Vector3((float) width, 0.0f, 0.0f);
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
      meshBuilder.triangle((short) 1, (short) 2, (short) 3);

      Mesh mesh = meshBuilder.end();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();

      GLFrameBuffer.FrameBufferBuilder frameBufferBuilder = new GLFrameBuffer.FrameBufferBuilder(width, height);
      frameBufferBuilder.addBasicColorTextureAttachment(Pixmap.Format.RGBA8888);
      frameBuffer = frameBufferBuilder.build();
      Texture colorBufferTexture = frameBuffer.getColorBufferTexture();
      material.set(TextureAttribute.createDiffuse(colorBufferTexture));

      material.set(ColorAttribute.createDiffuse(Color.WHITE));
      modelBuilder.part(meshPart, material);

      Model model = modelBuilder.end();
      modelInstance = new ModelInstance(model);
      modelInstance.transform.scale(0.005f, 0.005f, 0.005f);
   }

   public void processImGuiInput(ImGui3DViewInput input)
   {
      mousePosX = input.getMousePosX();
      mousePosY = input.getMousePosY();
      leftMouseDown = ImGui.isMouseDown(ImGuiMouseButton.Left);
   }

   public void render()
   {
      savedImGuiContext = ImGui.getCurrentContext();
      ImGui.setCurrentContext(virtualImGuiContext);

      ImGuiIO io = ImGui.getIO();
      io.setDisplaySize(width, height);
      io.setDisplayFramebufferScale(1.0f, 1.0f);
      io.setMousePos(mousePosX, mousePosY);
      io.setMouseDown(ImGuiMouseButton.Left, leftMouseDown);

      ImGuiPlatformIO platformIO = ImGui.getPlatformIO();
      platformIO.resizeMonitors(0);
      platformIO.pushMonitors(0.0f, 0.0f, width, height, 0.0f, 0.0f, width, height, 1.0f);

      float deltaTime = Gdx.app.getGraphics().getDeltaTime();
      io.setDeltaTime(deltaTime > 0.0f ? deltaTime : 1.0f / 60.0f);

      ImGui.newFrame();
//      ImGui.pushFont(font);

      ImGui.setNextWindowPos(0.0f, 0.0f);
      ImGui.setNextWindowSize(width, height);
      ImGui.begin("Meow");
      ImGui.text("Hi there.");
      ImGui.button("I'm a Button!");float[] values = new float[100];
      for (int i = 0; i < 100; i++)
      {
         values[i] = i;
      }
      ImGui.plotLines("Histogram", values, 100);
      ImGui.end();

//      for (GDX3DSituatedImGuiPanel panel : panels)
//      {
//         if (ImGui.begin(panel.getPanelName()))
//         {
//            panel.renderImGuiWidgets();
//            ImGui.end();
//         }
//      }

//      ImGui.popFont();
      ImGui.render();

      frameBuffer.begin();
      ImGuiTools.glClearDarkGray();
      imGuiGl3.renderDrawData(ImGui.getDrawData());
      frameBuffer.end();

      ImGui.setCurrentContext(savedImGuiContext);
   }

   public void addPanel(GDX3DSituatedImGuiPanel panel)
   {
      panels.add(panel);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      modelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      frameBuffer.dispose();
   }
}
