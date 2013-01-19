package com.yobotics.simulationconstructionset.util.graphics;

import us.ihmc.graphics3DAdapter.Graphics3DAdapter;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.java3D.Java3DGraphicsAdapter;
import us.ihmc.graphics3DAdapter.jme.JMEGraphics3dAdapter;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class DynamicGraphicObjectEvaluation
{
   public static void main(String[] args)
   {
      Graphics3DAdapter jmeGraphics3dAdapter = new JMEGraphics3dAdapter();
      Graphics3DAdapter java3DGraphicsAdapter = new Java3DGraphicsAdapter();

      evaluate(jmeGraphics3dAdapter);
      evaluate(java3DGraphicsAdapter);
   }
   
   public static void evaluate(Graphics3DAdapter graphicsAdapter)
   {
      Robot robot = new Robot("null");


      SimulationConstructionSet scs = new SimulationConstructionSet(robot, graphicsAdapter, 100);

      YoVariableRegistry registry = new YoVariableRegistry("Polygon");
      DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      final double[][] pointList = new double[][]
      {
         {0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.5, 1.2}, {0.5, -0.2}
      };

      ConvexPolygon2d polygon = new ConvexPolygon2d(pointList);

      AppearanceDefinition appearance = YoAppearance.Red();
      appearance.setTransparancy(0.8);

      final DynamicGraphicPolygon dynamicGraphicPolygon = new DynamicGraphicPolygon("Position", polygon, "polygon", "", registry, 3.0, appearance);
      final DynamicGraphicText3D dynamicGraphicText = new DynamicGraphicText3D("Text", "Hello", "text", "", registry, 0.2, YoAppearance.Blue());
      
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      YoFramePoint yoFramePoint = new YoFramePoint("position", "", worldFrame , registry);
      YoFrameVector yoFrameVector = new YoFrameVector("vector", "", worldFrame, registry);
      
      yoFramePoint.set(0.3, 0.4, 0.2);
      yoFrameVector.set(1.0, 2.0, 3.0);
      
      final DynamicGraphicVector dynamicGraphicVector = new DynamicGraphicVector("Vector", yoFramePoint, yoFrameVector, 1.0, YoAppearance.Yellow());

      dynamicGraphicPolygon.setPosition(0.1, 0.2, 1.0);
      dynamicGraphicPolygon.setYawPitchRoll(-0.1, -0.4, -0.3);

      dynamicGraphicText.setPosition(1.0, 0.0, 0.2);
      dynamicGraphicText.setYawPitchRoll(0.3, 0.0, 0.0);

      DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("Polygon");
      dynamicGraphicObjectsList.add(dynamicGraphicPolygon);
      dynamicGraphicObjectsList.add(dynamicGraphicText);
      dynamicGraphicObjectsList.add(dynamicGraphicVector);

      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);

      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
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
         public void run()
         {
            while (true)
            {
               quickPause();

               ConvexPolygon2d newPolygon = new ConvexPolygon2d(secondPointList);
               dynamicGraphicPolygon.updateConvexPolygon2d(newPolygon);
               dynamicGraphicText.setText("Hello");

               quickPause();
               newPolygon = new ConvexPolygon2d(pointList);
               dynamicGraphicPolygon.updateConvexPolygon2d(newPolygon);
               dynamicGraphicText.setText("GoodBye");

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
