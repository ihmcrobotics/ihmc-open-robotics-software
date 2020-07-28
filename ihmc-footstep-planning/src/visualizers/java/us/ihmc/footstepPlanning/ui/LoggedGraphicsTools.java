package us.ihmc.footstepPlanning.ui;

import javafx.scene.paint.Color;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.VariableDescriptor;
import us.ihmc.footstepPlanning.log.graphics.SphereGraphic;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.function.BiConsumer;

public class LoggedGraphicsTools
{
   private enum Type
   {
      SPHERE,
      CYLINDER,
      BOX;
   }

   public static void logSphereHeaderData(String linePrefix, String linePostfix, SphereGraphic sphereGraphic, List<YoVariable> allVariables, FileWriter fileWriter) throws IOException
   {
      fileWriter.write(linePrefix);
      fileWriter.write(Type.SPHERE.ordinal() + ":");
      fileWriter.write(getVariableIndex(sphereGraphic.getRadius(), allVariables) + ",");
      fileWriter.write(getVariableIndex(sphereGraphic.getCenter().getYoX(), allVariables) + ",");
      fileWriter.write(getVariableIndex(sphereGraphic.getCenter().getYoY(), allVariables) + ",");
      fileWriter.write(getVariableIndex(sphereGraphic.getCenter().getYoZ(), allVariables) + ":");
      fileWriter.write(sphereGraphic.getColor().getRed() + ",");
      fileWriter.write(sphereGraphic.getColor().getGreen() + ",");
      fileWriter.write(sphereGraphic.getColor().getBlue() + ",");
      fileWriter.write(sphereGraphic.getColor().getOpacity() + "");
      fileWriter.write(linePostfix);
   }

   public static BiConsumer<JavaFXMeshBuilder, FootstepPlannerEdgeData> generateGraphicsLoader(String line, VariableDescriptor variableDescriptor)
   {
      String[] graphicsDescription = line.split(":");
      Type type = Type.values()[Integer.parseInt(graphicsDescription[0])];

      String[] rgdDescription = graphicsDescription[2].split(",");
      Color color = Color.color(Double.parseDouble(rgdDescription[0]),
                                Double.parseDouble(rgdDescription[1]),
                                Double.parseDouble(rgdDescription[2]),
                                Double.parseDouble(rgdDescription[3]));

      switch (type)
      {
         case SPHERE:
            return generateSphereLoader(graphicsDescription[1], color, variableDescriptor);
      }

      return null;
   }

   private static BiConsumer<JavaFXMeshBuilder, FootstepPlannerEdgeData> generateSphereLoader(String variableDescriptions, Color color, VariableDescriptor variableDescriptor)
   {
      String[] indexStrings = variableDescriptions.split(",");
      int[] variableIndices = new int[indexStrings.length];
      for (int i = 0; i < indexStrings.length; i++)
      {
         variableIndices[i] = Integer.parseInt(indexStrings[i]);
      }

      return (meshBuilder, edgeData) ->
      {
         double radius = Double.longBitsToDouble(edgeData.getDataBuffer()[0]);
         double x = Double.longBitsToDouble(edgeData.getDataBuffer()[1]);
         double y = Double.longBitsToDouble(edgeData.getDataBuffer()[2]);
         double z = Double.longBitsToDouble(edgeData.getDataBuffer()[3]);
         meshBuilder.addSphere(radius, new Point3D(x, y, z));
      };
   }

   private static int getVariableIndex(YoVariable variable, List<YoVariable> allVariables)
   {
      for (int i = 0; i < allVariables.size(); i++)
      {
         if (allVariables.get(i) == variable)
            return i;
      }

      throw new RuntimeException("Could not find variable: " + variable.getName());
   }
}
