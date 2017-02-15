package us.ihmc.simulationconstructionset;

import java.util.StringTokenizer;

import us.ihmc.tools.io.xml.XMLReaderUtility;

public class GraphConfiguration
{
   private final String name;
   private static int id = 1;

   // private String[] varNames;

   public static final int
      INDIVIDUAL_SCALING = 0, AUTO_SCALING = 1, MANUAL_SCALING = 2;
   public static final int
      TIME_PLOT = 100, PHASE_PLOT = 101;
   private double manualMinScaling = 0.0, manualMaxScaling = 1.0;

   // private final double manualMinScaling, manualMaxScaling;
   @SuppressWarnings("unused")
   private double minPhaseXScaling = 0.0, maxPhaseXScaling = 1.0;
   private int scalingMethod = AUTO_SCALING;

   private int plotType = TIME_PLOT;

   private boolean showBaseLines = true;
   private double[] baseLines = new double[] {0};

   private static final GraphConfiguration standardAutoScalingConfiguration = new GraphConfiguration("auto", GraphConfiguration.AUTO_SCALING);

   static
   {
      standardAutoScalingConfiguration.setBaseLine(0.0);
      standardAutoScalingConfiguration.setShowBaseLines(true);
   }

   public static GraphConfiguration getStandardAutoScalingConfiguration()
   {
      return standardAutoScalingConfiguration;
   }

   public GraphConfiguration(String name)
   {
      this.name = name;
   }

   public GraphConfiguration(String name, int scalingMethod)
   {
      this.name = name;
      this.scalingMethod = scalingMethod;

      // System.out.println("Creating Graph Configuration with name " + name + ", scalingMethod = " + scalingMethod);
   }

   public GraphConfiguration(String name, int scalingMethod, double minScaling, double maxScaling)
   {
      this.name = name;
      this.scalingMethod = scalingMethod;

      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;

      // System.out.println("Creating Graph Configuration with name " + name + ", scalingMethod = " + scalingMethod);
      // System.out.println("  manualMinScaling = " + manualMinScaling + ", manualMaxScaling = " + manualMaxScaling);
   }


   public String getName()
   {
      return this.name;
   }


   public void setScalingMethod(int scalingMethod)
   {
      this.scalingMethod = scalingMethod;
   }

   public int getScalingMethod()
   {
      return this.scalingMethod;
   }

   public void setPlotType(int plotType)
   {
      this.plotType = plotType;
   }

   public int getPlotType()
   {
      return this.plotType;
   }

   public void setShowBaseLines(boolean showBaseLines)
   {
      this.showBaseLines = showBaseLines;
   }

   public void setBaseLine(double baseLine)
   {
      this.baseLines = new double[] {baseLine};
   }

   public void setBaseLines(double baseLine1, double baseLine2)
   {
      this.baseLines = new double[] {baseLine1, baseLine2};
   }

   public void setPositiveNegativeBaseLines(double baseLine)
   {
      this.baseLines = new double[] {-baseLine, baseLine};
   }

   public void setBaseLines(double[] baseLines)
   {
      this.baseLines = baseLines;
   }

   public void setBaseLine(int baseLineIndex, double value)
   {
      if (baseLineIndex >= baseLines.length) return;
      this.baseLines[baseLineIndex] = value;
   }
   
   public void incrementBaseLine(int baseLineIndex, double amountToIncrement)
   {
      if (baseLineIndex >= baseLines.length) return;
      this.baseLines[baseLineIndex] += amountToIncrement;
   }

   public boolean getShowBaseLines()
   {
      return this.showBaseLines;
   }

   public double[] getBaseLines()
   {
      return this.baseLines;
   }

   public void setManualScalingMinMax(double minScaling, double maxScaling)
   {
      // System.out.println("Changing " + name + ". Values were:  manualMinScaling = " + manualMinScaling + ", manualMaxScaling = " + manualMaxScaling);

      this.manualMinScaling = minScaling;
      this.manualMaxScaling = maxScaling;

      // System.out.println("Changing " + name + ". Values are now:  manualMinScaling = " + manualMinScaling + ", manualMaxScaling = " + manualMaxScaling);

   }

   public void setPhasePlotXScalingMinMax(double minPhaseXScaling, double maxPhaseXScaling)
   {
      this.minPhaseXScaling = minPhaseXScaling;
      this.maxPhaseXScaling = maxPhaseXScaling;
   }


   public double getManualScalingMin()
   {
      return manualMinScaling;
   }

   public double getManualScalingMax()
   {
      return manualMaxScaling;
   }

   public String getXMLStyleRepresentationOfClass()
   {
      String returnString = "\t\t<GraphConfiguration>\n";

      returnString += "\t\t\t<Name>";
      returnString += "config" + id;
      id++;
      returnString += "</Name>\n";
      returnString += "\t\t\t<ScalingMethod>";
      returnString += scalingMethod;
      returnString += "</ScalingMethod>\n";
      returnString += "\t\t\t<PlotType>";
      returnString += plotType;
      returnString += "</PlotType>\n";
      returnString += "\t\t\t<ShowBaseLines>";
      returnString += showBaseLines;
      returnString += "</ShowBaseLines>\n";
      returnString += "\t\t\t<BaseLines>";

      if (baseLines != null)
      {
         for (double baseLine : baseLines)
         {
            returnString += baseLine + ",";
         }
      }

      returnString += "</BaseLines>\n";
      returnString += "\t\t\t<MaxScaling>";
      returnString += manualMaxScaling;
      returnString += "</MaxScaling>\n";
      returnString += "\t\t\t<MinScaling>";
      returnString += manualMinScaling;
      returnString += "</MinScaling>\n";
      returnString += "\t\t</GraphConfig>";

      return returnString;
   }

   public static GraphConfiguration createClassBasedOnXMLRepresentation(int start, String xmlRepresentation)
   {
      GraphConfiguration tmp = null;
      try
      {
         String graphConfigurationString = XMLReaderUtility.getMiddleString(start, xmlRepresentation, "<GraphConfiguration>", "</GraphConfig>");

//       System.out.println("        GraphConfiguration: " + graphConfigurationString);

         String name = XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<Name>", "</Name>");

//       System.out.println("            Name: " + name);

         int scalingMethod = XMLReaderUtility.parseIntegerBetweenTwoStrings(0, graphConfigurationString, "<ScalingMethod>", "</ScalingMethod>");    // Integer.parseInt(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<ScalingMethod>", "</ScalingMethod>"));

//       System.out.println("            ScalingMethod: " + scalingMethod);

         int plotType = XMLReaderUtility.parseIntegerBetweenTwoStrings(0, graphConfigurationString, "<PlotType>", "</PlotType>");    // Integer.parseInt(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<PlotType>", "</PlotType>"));

//       System.out.println("            PlotType: " + plotType);

         boolean showBaseLines = XMLReaderUtility.parseBooleanBetweenTwoStrings(0, graphConfigurationString, "<ShowBaseLines>", "</ShowBaseLines>");    // Boolean.parseBoolean(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<ShowBaseLines>", "</ShowBaseLines>"));

//       System.out.println("            ShowBaseLines: " + showBaseLines);

         String baseLinesString = XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<BaseLines>", "</BaseLines>");

//       System.out.println("            BaseLines: " + baseLines);

         StringTokenizer tokenizer = new StringTokenizer(baseLinesString, " /t/n/r/f,");
         double[] baseLines = new double[tokenizer.countTokens()];
         int numberOfTokens = tokenizer.countTokens();
         for (int i = 0; i < numberOfTokens; i++)
         {
            baseLines[i] = XMLReaderUtility.parseDouble(tokenizer.nextToken());    // Double.parseDouble(tokenizer.nextToken());

//          System.out.println("                BaseLine: " + baseLines[i]);
         }

         double manualMaxScaling = XMLReaderUtility.parseDoubleBetweenTwoStrings(0, graphConfigurationString, "<MaxScaling>", "</MaxScaling>");    // Double.parseDouble(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<MaxScaling>", "</MaxScaling>"));

//       System.out.println("            ManualMaxScaling: " + manualMaxScaling);

         double manualMinScaling = XMLReaderUtility.parseDoubleBetweenTwoStrings(0, graphConfigurationString, "<MinScaling>", "</MinScaling>");    // Double.parseDouble(XMLReaderUtility.getMiddleString(0, graphConfigurationString, "<MinScaling>", "</MinScaling>"));

//       System.out.println("            ManualMinScaling: " + manualMinScaling);


         tmp = new GraphConfiguration(name, scalingMethod, manualMinScaling, manualMaxScaling);


         // this.name = name;
         // this.scalingMethod = scalingMethod;
         tmp.setPlotType(plotType);
         tmp.setShowBaseLines(showBaseLines);
         tmp.setBaseLines(baseLines);

         // this.manualMaxScaling = manualMaxScaling;
         // this.manualMinScaling = manualMinScaling;
      }
      catch (Exception e)
      {
         e.printStackTrace();
         XMLReaderUtility.displayErrorMessage();

         return null;
      }

      return tmp;
   }






}
