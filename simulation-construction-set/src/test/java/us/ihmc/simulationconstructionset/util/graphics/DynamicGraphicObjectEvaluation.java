package us.ihmc.simulationconstructionset.util.graphics;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicText3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class DynamicGraphicObjectEvaluation
{
   public static void main(String[] args)
   {
      Graphics3DAdapter jmeGraphics3dAdapter = new JMEGraphics3DAdapter();
      evaluate(jmeGraphics3dAdapter);
   }



   public static void evaluate(Graphics3DAdapter graphicsAdapter)
   {
      Robot robot = new Robot("null");

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(100);

      final SimulationConstructionSet scs = new SimulationConstructionSet(robot, graphicsAdapter, parameters);
      scs.setDT(0.1, 1);

      YoVariableRegistry registry = new YoVariableRegistry("Polygon");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      // Polygon:
      final double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.5, 1.2}, {0.5, -0.2}
      };

      ConvexPolygon2D polygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointList));

      AppearanceDefinition appearance = YoAppearance.Red();
      appearance.setTransparency(0.8);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      int maxNumberOfVertices = polygon.getNumberOfVertices();
      final YoFrameConvexPolygon2D yoPolygon = new YoFrameConvexPolygon2D("PolyPoints", "", worldFrame, maxNumberOfVertices, registry);
      yoPolygon.set(polygon);
      YoFramePoseUsingYawPitchRoll yoPolyOrigin = new YoFramePoseUsingYawPitchRoll("PolyOrigin", worldFrame, registry);
      YoGraphicPolygon yoGraphicPolygon = new YoGraphicPolygon("Polygon", yoPolygon, yoPolyOrigin, 3.0, appearance);
      yoPolyOrigin.setXYZ(0.1, 0.2, 1.0);
      yoPolyOrigin.setYawPitchRoll(-0.1, -0.4, -0.3);

      // 3D Text:
      final YoGraphicText3D yoGraphicText = new YoGraphicText3D("Text", "Hello", "text", "", registry, 0.2, YoAppearance.Blue());
      yoGraphicText.setPosition(1.0, 0.0, 0.2);
      yoGraphicText.setYawPitchRoll(0.3, 0.0, 0.0);

      // Vector:
      YoFramePoint3D yoFramePoint = new YoFramePoint3D("position", "", worldFrame, registry);
      YoFrameVector3D yoFrameVector = new YoFrameVector3D("vector", "", worldFrame, registry);

      yoFramePoint.set(0.3, 0.4, 0.2);
      yoFrameVector.set(1.0, 2.0, 3.0);

      final YoGraphicVector yoGraphicVector = new YoGraphicVector("Vector", yoFramePoint, yoFrameVector, 1.0, YoAppearance.Yellow());

      // YoFrameConvexPolygon2d:
      final YoFrameConvexPolygon2D yoFramePolygon = new YoFrameConvexPolygon2D("yoPolygon", "", worldFrame, 10, registry);
      YoFramePoint3D yoFramePolygonPosition = new YoFramePoint3D("yoPolygonPosition", "", worldFrame, registry);
      yoFramePolygonPosition.set(2.0, 1.0, 0.3);
      final YoFrameYawPitchRoll yoFramePolygonOrientation = new YoFrameYawPitchRoll("yoPolygonOrientation", "", worldFrame, registry);
      yoFramePolygonOrientation.setYawPitchRoll(1.2, 0.1, 0.4);
      final YoGraphicPolygon yoGraphicYoFramePolygon = new YoGraphicPolygon("YoFramePolygon", yoFramePolygon,
                                                                           yoFramePolygonPosition, yoFramePolygonOrientation, 1.0, YoAppearance.DarkBlue());

      // Box Ghost:
      Graphics3DObject boxGhostGraphics = new Graphics3DObject();
      AppearanceDefinition transparantBlue = YoAppearance.Blue();
      YoAppearance.makeTransparent(transparantBlue, 0.5);
      boxGhostGraphics.translate(0.0, 0.0, -0.5);
      boxGhostGraphics.addCube(1.0, 1.0, 1.0, transparantBlue);

      YoFramePoint3D boxPosition = new YoFramePoint3D("boxPosition", "", worldFrame, registry);
      double boxSize = 0.3;
      boxPosition.set(boxSize / 2.0, boxSize / 2.0, boxSize / 2.0);
      YoFrameYawPitchRoll boxOrientation = new YoFrameYawPitchRoll("boxOrientation", worldFrame, registry);
      YoGraphicShape yoGraphicBoxGhost = new YoGraphicShape("boxGhost", boxGhostGraphics, boxPosition, boxOrientation, boxSize);

      YoGraphicsList yoGraphicsList = new YoGraphicsList("Polygon");
      yoGraphicsList.add(yoGraphicPolygon);
      yoGraphicsList.add(yoGraphicText);
      yoGraphicsList.add(yoGraphicVector);
      yoGraphicsList.add(yoGraphicYoFramePolygon);
      yoGraphicsList.add(yoGraphicBoxGhost);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(yoGraphicPolygon);
      yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(yoGraphicYoFramePolygon);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.addYoVariableRegistry(registry);

      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(coordinateSystem);

      scs.startOnAThread();

      final double[][] secondPointList = new double[][]
      {
         {0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}
      };


      Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            int i = 0;

            while (i++ < 20)
            {
               quickPause();

               ConvexPolygon2D newPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(secondPointList));
               yoPolygon.set(newPolygon);

               ConvexPolygon2D newYoPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointList));
               yoFramePolygon.set(newYoPolygon);
               yoGraphicYoFramePolygon.update();

               Vector3D eulerAngles = new Vector3D();
               yoFramePolygonOrientation.getEulerAngles(eulerAngles);
               eulerAngles.setY(eulerAngles.getY() + 0.1);
               yoFramePolygonOrientation.setEulerAngles(eulerAngles);

               yoGraphicText.setText("Hello");
               yoGraphicText.update();

               scs.tickAndUpdate();

               quickPause();
               newPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointList));
               yoPolygon.set(newPolygon);
               yoGraphicYoFramePolygon.update();

               newYoPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(secondPointList));
               yoFramePolygon.set(newYoPolygon);

               yoGraphicText.setText("GoodBye");
               yoGraphicText.update();

               scs.tickAndUpdate();

            }
         }

      };

      Thread thread = new Thread(runnable);
      thread.setDaemon(true);
      thread.start();
   }

   private static void quickPause()
   {
      try
      {
         Thread.sleep(200L);
      }
      catch (InterruptedException e)
      {
      }
   }
}
