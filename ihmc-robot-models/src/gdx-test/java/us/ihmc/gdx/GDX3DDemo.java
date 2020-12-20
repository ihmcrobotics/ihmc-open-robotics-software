package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Image;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton.TextButtonStyle;
import com.badlogic.gdx.scenes.scene2d.utils.TextureRegionDrawable;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

/**
 * TODO: Pause and resume?
 */
public class GDX3DDemo extends Lwjgl3ApplicationAdapter
{
   private static final int INITIAL_WIDTH = 1100;
   private static final int INITIAL_HEIGHT = 800;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private Viewport viewport;
   private ModelBatch modelBatch;
   private ModelInstance boxes;
   private ModelInstance coordinateFrame;

   private OrthographicCamera camera2D;
   private int currentWindowWidth;
   private int currentWindowHeight;
   private Stage stage;
   private Table table;

   public GDX3DDemo()
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle("GDX3DDemo");
      applicationConfiguration.setWindowedMode(INITIAL_WIDTH, INITIAL_HEIGHT);
      applicationConfiguration.useVsync(true);

      new Lwjgl3Application(this, applicationConfiguration);
   }

   @Override
   public void create()
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      directionalLights.lights.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -1f, -0.8f, -0.2f));
      environment.set(directionalLights);

      modelBatch = new ModelBatch();

      camera3D = new FocusBasedGDXCamera();
      viewport = new ScreenViewport(camera3D);


      coordinateFrame = new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3));
      boxes = new BoxesDemoModel().newInstance();

      stage = new Stage(new ScreenViewport());
//      Gdx.input.setInputProcessor(stage);

      table = new Table();
      table.setFillParent(true);
      table.setDebug(true);
      stage.addActor(table);

      Skin skin = new Skin(Gdx.files.internal("uiskin.json"));
//      Skin skin = new Skin();
//
//      // Generate a 1x1 white texture and store it in the skin named "white".
//      Pixmap pixmap = new Pixmap(1, 1, Pixmap.Format.RGBA8888);
//      pixmap.setColor(Color.WHITE);
//      pixmap.fill();
//      skin.add("white", new Texture(pixmap));
//
//      // Store the default libgdx font under the name "default".
//      skin.add("default", new BitmapFont());
//
//      TextButtonStyle style = new TextButtonStyle();
//      style.up = skin.newDrawable("white", Color.DARK_GRAY);
//      style.down = skin.newDrawable("white", Color.DARK_GRAY);
//      style.checked = skin.newDrawable("white", Color.BLUE);
//      style.over = skin.newDrawable("white", Color.LIGHT_GRAY);
//      style.font = skin.getFont("default");
//      skin.add("default", style);

//      TextureRegion upRegion = ...
//      TextureRegion downRegion = ...
//      BitmapFont buttonFont = ...
//      TextButtonStyle style = new TextButtonStyle();
//      style.up = new TextureRegionDrawable(upRegion);
//      style.down = new TextureRegionDrawable(downRegion);
//      style.font = buttonFont;

      TextButton button1 = new TextButton("Button 1", skin);
      table.add(button1);

      TextButton button2 = new TextButton("Button 2", skin);
      table.add(button2);

//      table.add(new Image(skin.newDrawable("white", Color.RED))).size(64);

//      table.setSize();

      camera2D = new OrthographicCamera(INITIAL_WIDTH, INITIAL_HEIGHT / 4f);
      camera2D.position.set(camera3D.viewportWidth / 3f, camera3D.viewportHeight / 3f, 0);
      camera2D.update();
   }

   @Override
   public void resize(int width, int height)
   {
      this.currentWindowWidth = width;
      this.currentWindowHeight = height;
   }

   @Override
   public void render()
   {
      Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);
      Gdx.gl.glEnable(GL20.GL_TEXTURE_2D);

      viewport.update(currentWindowWidth, currentWindowHeight * 3 / 4);
      Gdx.gl.glViewport(0, Gdx.graphics.getHeight()* 1 / 4, Gdx.graphics.getWidth(), Gdx.graphics.getHeight()* 3 / 4);

      camera3D.render();

      modelBatch.begin(camera3D);
      modelBatch.render(camera3D.getFocusPointSphere(), environment);
      modelBatch.render(boxes, environment);
      modelBatch.render(coordinateFrame, environment);
      modelBatch.end();
//
      Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight()* 1 / 4);
      stage.getViewport().update(currentWindowWidth, currentWindowHeight * 1 / 4, true);

      stage.act(Gdx.graphics.getDeltaTime());
      stage.draw();

//      camera2D.update();
   }

   @Override
   public void dispose()
   {
      boxes.model.dispose();
      coordinateFrame.model.dispose();
      camera3D.dispose();
      modelBatch.dispose();
      stage.dispose();
   }

   @Override
   public boolean closeRequested()
   {
      return true;
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}