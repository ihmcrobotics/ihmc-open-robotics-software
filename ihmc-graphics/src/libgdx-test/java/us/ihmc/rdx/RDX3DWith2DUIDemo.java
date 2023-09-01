package us.ihmc.rdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.RDXModelBuilder;

public class RDX3DWith2DUIDemo
{
   private Stage stage;
   private Table table;

   public RDX3DWith2DUIDemo()
   {
      RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.setViewportBounds(0,
                              (int) (sceneManager.getCurrentWindowHeight() * 1.0 / 4.0),
                              (int) (sceneManager.getCurrentWindowWidth() * 1.0),
                              (int) (sceneManager.getCurrentWindowHeight() * 3.0 / 4.0));

            sceneManager.addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());

            stage = new Stage(new ScreenViewport());
            sceneManager.addLibGDXInputProcessor(stage);

            table = new Table();
            table.setFillParent(true);
            //      table.setDebug(true);
            stage.addActor(table);

            Skin skin = new Skin(Gdx.files.internal("uiskin.json"));

            table.left();
            table.top();
            TextButton button1 = new TextButton("Button 1", skin);
            table.add(button1);

            TextButton button2 = new TextButton("Button 2", skin);
            table.add(button2);
         }

         @Override
         public void render()
         {
            sceneManager.render();

            GL41.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight() * 1 / 4);
            stage.getViewport().update(sceneManager.getCurrentWindowWidth(), sceneManager.getCurrentWindowHeight() * 1 / 4, true);

            stage.act(Gdx.graphics.getDeltaTime());
            stage.draw();
         }

         @Override
         public void dispose()
         {
            sceneManager.dispose();
            stage.dispose();
         }
      }, getClass().getSimpleName(), 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDX3DWith2DUIDemo();
   }
}
