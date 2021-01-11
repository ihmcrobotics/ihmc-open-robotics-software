package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.utils.viewport.ScreenViewport;

public class GDX3DWith2DUIDemo
{
   private ModelInstance boxes;
   private ModelInstance coordinateFrame;

   private Stage stage;
   private Table table;

   public GDX3DWith2DUIDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), "GDX3DDemo", 1100, 800);
   }

   class PrivateGDXApplication extends GDX3DApplication
   {
      @Override
      public void create()
      {
         super.create();

         setViewportBounds(0,
                           (int) (getCurrentWindowHeight() * 1.0 / 4.0),
                           (int) (getCurrentWindowWidth() * 1.0),
                           (int) (getCurrentWindowHeight() * 3.0 / 4.0));

         coordinateFrame = new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3));
         boxes = new BoxesDemoModel().newInstance();

         stage = new Stage(new ScreenViewport());
         addInputProcessor(stage);

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
         renderBefore();

         getModelBatch().render(coordinateFrame, getEnvironment());
         getModelBatch().render(boxes, getEnvironment());

         renderAfter();

         Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight() * 1 / 4);
         stage.getViewport().update(getCurrentWindowWidth(), getCurrentWindowHeight() * 1 / 4, true);

         stage.act(Gdx.graphics.getDeltaTime());
         stage.draw();
      }

      @Override
      public void dispose()
      {
         super.dispose();
         stage.dispose();
      }
   }

   public static void main(String[] args)
   {
      new GDX3DWith2DUIDemo();
   }
}
