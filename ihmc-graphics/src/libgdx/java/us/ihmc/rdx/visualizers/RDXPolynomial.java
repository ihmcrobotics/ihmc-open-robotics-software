package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

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

   public void compute(List<? extends Polynomial3DVariableHolder> polynomials)
   {
      line.clear();
      if (polynomials.isEmpty())
         return;

      List<Polynomial3DVariables> yoPolynomial3Ds = polynomials.stream().map(Polynomial3DVariables::new).toList();

      Point3D[] intermediatePositions = new Point3D[resolution];
      Vector3D[] intermediateVelocities = new Vector3D[resolution];
      Vector3D[] intermediateAccelerations = new Vector3D[resolution];

      double trajectoryTime = polynomials.get(polynomials.size() - 1).getFinalTime();

      int polynomialIndex = 0;

      double maxVelocity = 0.0;
      double maxAcceleration = 0.0;

      for (int i = 0; i < resolution; i++)
      {
         double t = i / (resolution - 1.0) * trajectoryTime;

         while (t > polynomials.get(polynomialIndex).getFinalTime())
            polynomialIndex++;

         Point3D intermediatePosition = new Point3D();
         Vector3D intermediateVelocity = new Vector3D();
         Vector3D intermediateAcceleration = new Vector3D();

         Polynomial3DVariables activePolynomial3D = yoPolynomial3Ds.get(polynomialIndex);
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

   public static class Polynomial3DVariables implements Polynomial3DVariableHolder
   {
      private final PolynomialVariables xPolynomial, yPolynomial, zPolynomial;
      private final Point3DReadOnly position;
      private final Vector3DReadOnly velocity;
      private final Vector3DReadOnly acceleration;

      private Polynomial3DVariables(Polynomial3DVariableHolder holder)
      {
         this(holder.getPolynomialX(), holder.getPolynomialY(), holder.getPolynomialZ());
      }

      public Polynomial3DVariables(PolynomialReadOnly xPolynomial, PolynomialReadOnly yPolynomial, PolynomialReadOnly zPolynomial)
      {
         this(new PolynomialVariables(xPolynomial), new PolynomialVariables(yPolynomial), new PolynomialVariables(zPolynomial));
      }

      private Polynomial3DVariables(PolynomialVariables xPolynomial, PolynomialVariables yPolynomial, PolynomialVariables zPolynomial)
      {
         this.xPolynomial = xPolynomial;
         this.yPolynomial = yPolynomial;
         this.zPolynomial = zPolynomial;

         position = EuclidCoreFactories.newLinkedPoint3DReadOnly(xPolynomial::getPosition, yPolynomial::getPosition, zPolynomial::getPosition);
         velocity = EuclidCoreFactories.newLinkedVector3DReadOnly(xPolynomial::getVelocity, yPolynomial::getVelocity, zPolynomial::getVelocity);
         acceleration = EuclidCoreFactories.newLinkedVector3DReadOnly(xPolynomial::getAcceleration, yPolynomial::getAcceleration, zPolynomial::getAcceleration);
      }

      public void compute(double x)
      {
         xPolynomial.compute(x);
         yPolynomial.compute(x);
         zPolynomial.compute(x);
      }

      public double getInitialTime()
      {
         return xPolynomial.getInitialTime();
      }

      public double getFinalTime()
      {
         return xPolynomial.getFinalTime();
      }

      public Point3DReadOnly getPosition()
      {
         return position;
      }

      public Vector3DReadOnly getVelocity()
      {
         return velocity;
      }

      public Vector3DReadOnly getAcceleration()
      {
         return acceleration;
      }

      @Override
      public PolynomialVariables getPolynomialX()
      {
         return xPolynomial;
      }

      @Override
      public PolynomialVariables getPolynomialY()
      {
         return yPolynomial;
      }

      @Override
      public PolynomialVariables getPolynomialZ()
      {
         return zPolynomial;
      }
   }

   static class PolynomialVariables implements PolynomialReadOnly
   {
      private final double[] coefficients;
      private final int numberOfCoefficients;
      private final double[] xPowers;
      private double pos, vel, acc;

      private double t = Double.NaN;
      private final TimeIntervalBasics timeInterval = new TimeInterval();

      private PolynomialVariables(PolynomialReadOnly holder)
      {
         this(holder.getCoefficients(), holder.getNumberOfCoefficients(), holder.getTimeInterval());
      }

      private PolynomialVariables(double[] coefficients, int numberOfCoefficients, TimeIntervalReadOnly timeInterval)
      {
         this.coefficients = coefficients;
         this.numberOfCoefficients = numberOfCoefficients;
         xPowers = new double[coefficients.length];
         this.timeInterval.set(timeInterval);
      }

      @Override
      public void initialize()
      {

      }

      public void compute(double x)
      {
         setXPowers(xPowers, x);

         pos = vel = acc = 0.0;
         for (int i = 0; i < numberOfCoefficients; i++)
         {
            pos += coefficients[i] * xPowers[i];
         }

         for (int i = 1; i < numberOfCoefficients; i++)
         {
            vel += i * coefficients[i] * xPowers[i - 1];
         }

         for (int i = 2; i < numberOfCoefficients; i++)
         {
            acc += (i - 1) * i * coefficients[i] * xPowers[i - 2];
         }
      }

      public void setXPowers(double[] xPowers, double x)
      {
         xPowers[0] = 1.0;
         for (int i = 1; i < xPowers.length; i++)
         {
            xPowers[i] = xPowers[i - 1] * x;
         }
      }

      public int getMaximumNumberOfCoefficients()
      {
         return coefficients.length;
      }

      public double getPosition()
      {
         return pos;
      }

      public double getVelocity()
      {
         return vel;
      }

      public double getAcceleration()
      {
         return acc;
      }

      @Override
      public double[] getCoefficients()
      {
         return coefficients;
      }

      @Override
      public int getNumberOfCoefficients()
      {
         return numberOfCoefficients;
      }

      @Override
      public double getCurrentTime()
      {
         return t;
      }

      @Override
      public double getCoefficient(int idx)
      {
         return getCoefficients()[idx];
      }

      @Override
      public TimeIntervalBasics getTimeInterval()
      {
         return timeInterval;
      }

      @Override
      public double getValue()
      {
         return getPosition();
      }
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
