package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.log.LogTools;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class GDXMocapVisualizer implements RenderableProvider
{
   private final ArrayList<ModelInstance> poseModels = new ArrayList<>();
   private String mocapLogFilename;
   private BufferedReader br;
   public GDXMocapVisualizer(String filename) throws FileNotFoundException
   {
      this.mocapLogFilename = filename;
      br = new BufferedReader(new FileReader(this.mocapLogFilename));
      LogTools.info("Buffered Reader: " + br);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      String row;
      try
      {
         if (br == null) return;

         if((row = br.readLine()) != null) {
            String[] values = row.split("\t");

            List<String> records = new ArrayList<>();
            records = Arrays.asList(values);

            LogTools.info("Line: " + row);


            Point3D position = new Point3D(Float.parseFloat(records.get(0)), Float.parseFloat(records.get(1)), Float.parseFloat(records.get(2)));

            ModelInstance modelInstance = GDXModelBuilder.createSphere(0.05f, Color.YELLOW);
            modelInstance.transform.translate(position.getX32(), position.getY32(), position.getZ32());

            modelInstance.getRenderables(renderables, pool);


//            modelInstance = GDXModelBuilder.createCoordinateFrameInstance(0.1);
//            GDXTools.toGDX(framePose, tempTransform, modelInstance.transform);
//            poseModels.clear();
//            poseModels.add(modelInstance);

         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
}
