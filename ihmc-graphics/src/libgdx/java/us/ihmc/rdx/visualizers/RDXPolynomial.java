package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.commons.trajectories.core.Polynomial3D;
import us.ihmc.commons.trajectories.interfaces.PolynomialReadOnly;

import java.awt.*;
import java.util.List;

public class RDXPolynomial implements RenderableProvider
{
   private static final int COLOR_RESOLUTION = 128;

   private final int resolution;

   private YoGraphicPolynomial3D.TrajectoryColorType currentColorType = YoGraphicPolynomial3D.TrajectoryColorType.VELOCITY_BASED;
   private final RDXSplineBody line;

   private final AppearanceDefinition[] colorPalette = createColorPalette(COLOR_RESOLUTION);

   public RDXPolynomial(double radius,
                        int resolution)
   {
      this.resolution = resolution;

      line = new RDXSplineBody((float) radius);
   }

   public void clear()
   {
      line.clear();
   }

   public void compute(List<? extends Polynomial3D> polynomials)
   {
      line.clear();
      if (polynomials.isEmpty())
         return;

      List<Polynomial3D> yoPolynomial3Ds = polynomials.stream().map(Polynomial3D::new).toList();

      Point3D[] intermediatePositions = new Point3D[resolution];
      Vector3D[] intermediateVelocities = new Vector3D[resolution];
      Vector3D[] intermediateAccelerations = new Vector3D[resolution];

      double trajectoryTime = polynomials.get(polynomials.size() - 1).getTimeInterval().getEndTime();

      int polynomialIndex = 0;

      double maxVelocity = 0.0;
      double maxAcceleration = 0.0;

      for (int i = 0; i < resolution; i++)
      {
         double t = i / (resolution - 1.0) * trajectoryTime;

         while (t > polynomials.get(polynomialIndex).getTimeInterval().getEndTime())
            polynomialIndex++;

         Point3D intermediatePosition = new Point3D();
         Vector3D intermediateVelocity = new Vector3D();
         Vector3D intermediateAcceleration = new Vector3D();

         Polynomial3D activePolynomial3D = yoPolynomial3Ds.get(polynomialIndex);
         activePolynomial3D.compute(t);
         intermediatePosition.set(activePolynomial3D.getPosition());
         intermediateVelocity.set(activePolynomial3D.getVelocity());
         intermediateAcceleration.set(activePolynomial3D.getAcceleration());

         intermediatePositions[i] = intermediatePosition;
         intermediateVelocities[i] = intermediateVelocity;
         intermediateAccelerations[i] = intermediateAcceleration;

         maxVelocity = Math.max(maxVelocity, activePolynomial3D.getVelocity().normSquared());
         maxAcceleration = Math.max(maxAcceleration, activePolynomial3D.getAcceleration().normSquared());
      }

      maxVelocity = Math.sqrt(maxVelocity);
      maxAcceleration = Math.sqrt(maxAcceleration);

      for (int i = 0; i < resolution - 1; i++)
      {
         com.badlogic.gdx.graphics.Color color;
         switch (getCurrentColorType())
         {
            case VELOCITY_BASED:
            {
               double velocity = intermediateVelocities[i].norm();
               int colorIndex = (int) Math.round((colorPalette.length - 1.0) * (velocity / maxVelocity));
               color = new com.badlogic.gdx.graphics.Color(colorPalette[colorIndex].getAwtColor().getRGB());
               break;
            }
            case ACCELERATION_BASED:
            {
               double acceleration = intermediateAccelerations[i].norm();
               int colorIndex = (int) Math.round((colorPalette.length - 1.0) * (acceleration / maxAcceleration));
               color = new com.badlogic.gdx.graphics.Color(colorPalette[colorIndex].getAwtColor().getRGB());
               break;
            }
            default:
               color = com.badlogic.gdx.graphics.Color.BLACK;
               break;
         }

         line.setColor(color);
         line.generateMeshes(intermediatePositions[i], intermediatePositions[i + 1]);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      line.getRenderables(renderables, pool);
   }

   private void setCurrentColorType(YoGraphicPolynomial3D.TrajectoryColorType colorType)
   {
      currentColorType = colorType;
   }

   private YoGraphicPolynomial3D.TrajectoryColorType getCurrentColorType()
   {
      return currentColorType;
   }

   private static AppearanceDefinition[] createColorPalette(int size)
   {
      AppearanceDefinition[] colorPalette = new AppearanceDefinition[size];

      for (int i = 0; i < size; i++)
      {
         float hue = 240.0f * (1.0f - i / (size - 1.0f)) / 360.0f;
         colorPalette[i] = YoAppearance.Color(Color.getHSBColor(hue, 0.9f, 0.9f));
      }

      return colorPalette;
   }



   public static interface Polynomial3DVariableHolder
   {
      double getInitialTime();

      double getFinalTime();

      PolynomialReadOnly getPolynomialX();

      PolynomialReadOnly getPolynomialY();

      PolynomialReadOnly getPolynomialZ();

   }

   public void dispose()
   {
      line.dispose();
   }
}
