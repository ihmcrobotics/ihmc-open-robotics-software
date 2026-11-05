package us.ihmc.avatar.logging;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotDataLogger.yoGraphics.YoGraphicFieldData;
import us.ihmc.robotDataLogger.yoGraphics.YoGraphicFieldsData;
import us.ihmc.robotDataLogger.yoGraphics.YoGraphicsData;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition.YoGraphicFieldInfo;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition.YoGraphicFieldsSummary;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;

/**
 * Flattens an scs2 {@link YoGraphicGroupDefinition} tree into the logger's wire-format
 * {@link YoGraphicsData}, so it can be published through {@code YoVariableServer#addRegistry}.
 * <p>
 * The logger module intentionally has no dependency on scs2-definition, so this conversion has to
 * happen on the caller's side rather than inside {@code YoVariableServer}.
 */
public class SCS2YoGraphicLogTools
{
   public static YoGraphicsData toYoGraphicsData(YoGraphicGroupDefinition scs2YoGraphics)
   {
      if (scs2YoGraphics == null)
         return null;

      List<YoGraphicFieldsSummary> summaries = YoGraphicDefinition.exportSubtreeYoGraphicFieldsSummaryList(scs2YoGraphics);

      List<YoGraphicFieldsData> yoGraphicFieldsDataList = new ArrayList<>();
      for (YoGraphicFieldsSummary summary : summaries)
      {
         YoGraphicFieldsData fieldsData = new YoGraphicFieldsData();
         for (YoGraphicFieldInfo field : summary)
            fieldsData.addField(field.getFieldName(), field.getFieldValue());
         yoGraphicFieldsDataList.add(fieldsData);
      }

      return new YoGraphicsData(yoGraphicFieldsDataList);
   }
}
