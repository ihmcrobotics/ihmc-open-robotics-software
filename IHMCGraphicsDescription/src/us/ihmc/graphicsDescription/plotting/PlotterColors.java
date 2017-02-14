package us.ihmc.graphicsDescription.plotting;

import java.awt.Color;

/**
 * Future improvements should be influenced by http://www.kennethmoreland.com/color-maps/ColorMapsExpanded.pdf
 * 
 * @author Duncan Calvert (dcalvert@ihmc.us)
 */
public class PlotterColors
{
   private Color labelColor;
   private Color backgroundColor;
   private Color gridAxisColor;
   private Color gridEveryOneColor;
   private Color gridEveryFiveColor;
   private Color gridEveryTenColor;
   private Color selectionColor;
   
   private PlotterColors()
   {
      // Disallow public construction
   }
   
   public static PlotterColors simulationConstructionSetStyle()
   {
      PlotterColors simulationConstructionSetStyle = new PlotterColors();
      simulationConstructionSetStyle.setLabelColor(Color.WHITE);
      simulationConstructionSetStyle.setBackgroundColor(new Color(180, 220, 240));
      simulationConstructionSetStyle.setGridAxisColor(Color.GRAY);
      simulationConstructionSetStyle.setGridEveryOneColor(new Color(230, 240, 250));
      simulationConstructionSetStyle.setGridEveryFiveColor(new Color(180, 190, 210));
      simulationConstructionSetStyle.setGridEveryTenColor(new Color(180, 190, 210));
      simulationConstructionSetStyle.setSelectionColor(Color.RED);
      return simulationConstructionSetStyle;
   }
   
   public static PlotterColors javaFXStyle()
   {
      PlotterColors simulationConstructionSetStyle = new PlotterColors();
      simulationConstructionSetStyle.setLabelColor(Color.BLACK);
      simulationConstructionSetStyle.setBackgroundColor(Color.getHSBColor(0.0f, 0.0f, 0.93f));
      simulationConstructionSetStyle.setGridAxisColor(Color.getHSBColor(0.0f, 0.0f, 0.81f));
      simulationConstructionSetStyle.setGridEveryOneColor(Color.getHSBColor(0.0f, 0.0f, 0.81f));
      simulationConstructionSetStyle.setGridEveryFiveColor(Color.getHSBColor(0.0f, 0.0f, 0.81f));
      simulationConstructionSetStyle.setGridEveryTenColor(Color.getHSBColor(0.0f, 0.0f, 0.81f));
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
