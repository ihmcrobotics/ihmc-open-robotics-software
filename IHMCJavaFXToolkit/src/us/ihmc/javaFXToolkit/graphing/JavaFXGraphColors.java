package us.ihmc.javaFXToolkit.graphing;

import javafx.scene.paint.Color;

/**
 * Future improvements should be influenced by http://www.kennethmoreland.com/color-maps/ColorMapsExpanded.pdf
 * 
 * @author Duncan Calvert (dcalvert@ihmc.us)
 */
public class JavaFXGraphColors
{
   private Color labelColor;
   private Color backgroundColor;
   private Color gridAxisColor;
   private Color gridEveryOneColor;
   private Color gridEveryFiveColor;
   private Color gridEveryTenColor;
   private Color selectionColor;
   
   private JavaFXGraphColors()
   {
      // Disallow public construction
   }
   
   public static JavaFXGraphColors simulationConstructionSetStyle()
   {
      JavaFXGraphColors simulationConstructionSetStyle = new JavaFXGraphColors();
      simulationConstructionSetStyle.setLabelColor(Color.WHITE);
      simulationConstructionSetStyle.setBackgroundColor(new Color(180.0 / 255.0, 220 / 255.0, 240 / 255.0, 1.0));
      simulationConstructionSetStyle.setGridAxisColor(Color.GRAY);
      simulationConstructionSetStyle.setGridEveryOneColor(new Color(230 / 255.0, 240 / 255.0, 250 / 255.0, 1.0));
      simulationConstructionSetStyle.setGridEveryFiveColor(new Color(180 / 255.0, 190 / 255.0, 210 / 255.0, 1.0));
      simulationConstructionSetStyle.setGridEveryTenColor(new Color(180 / 255.0, 190 / 255.0, 210 / 255.0, 1.0));
      simulationConstructionSetStyle.setSelectionColor(Color.RED);
      return simulationConstructionSetStyle;
   }
   
   public static JavaFXGraphColors javaFXStyle()
   {
      JavaFXGraphColors simulationConstructionSetStyle = new JavaFXGraphColors();
      simulationConstructionSetStyle.setLabelColor(Color.BLACK);
      simulationConstructionSetStyle.setBackgroundColor(Color.hsb(0.0, 0.0, 0.93));
      simulationConstructionSetStyle.setGridAxisColor(Color.hsb(0.0, 0.0, 0.81));
      simulationConstructionSetStyle.setGridEveryOneColor(Color.hsb(0.0, 0.0, 0.81));
      simulationConstructionSetStyle.setGridEveryFiveColor(Color.hsb(0.0, 0.0, 0.81));
      simulationConstructionSetStyle.setGridEveryTenColor(Color.hsb(0.0, 0.0, 0.81));
      simulationConstructionSetStyle.setSelectionColor(Color.RED);
      return simulationConstructionSetStyle;
   }

   private void setLabelColor(Color labelColor)
   {
      this.labelColor = labelColor;
   }

   private void setBackgroundColor(Color backgroundColor)
   {
      this.backgroundColor = backgroundColor;
   }

   private void setGridAxisColor(Color gridAxisColor)
   {
      this.gridAxisColor = gridAxisColor;
   }

   private void setGridEveryOneColor(Color gridEveryOneColor)
   {
      this.gridEveryOneColor = gridEveryOneColor;
   }

   private void setGridEveryFiveColor(Color gridEveryFiveColor)
   {
      this.gridEveryFiveColor = gridEveryFiveColor;
   }

   private void setGridEveryTenColor(Color gridEveryTenColor)
   {
      this.gridEveryTenColor = gridEveryTenColor;
   }

   private void setSelectionColor(Color selectionColor)
   {
      this.selectionColor = selectionColor;
   }

   public Color getLabelColor()
   {
      return labelColor;
   }

   public Color getBackgroundColor()
   {
      return backgroundColor;
   }

   public Color getGridAxisColor()
   {
      return gridAxisColor;
   }

   public Color getGridEveryOneColor()
   {
      return gridEveryOneColor;
   }

   public Color getGridEveryFiveColor()
   {
      return gridEveryFiveColor;
   }

   public Color getGridEveryTenColor()
   {
      return gridEveryTenColor;
   }

   public Color getSelectionColor()
   {
      return selectionColor;
   }
}
