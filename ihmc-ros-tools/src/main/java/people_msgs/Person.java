package people_msgs;

import geometry_msgs.Point;
import java.util.List;
import org.ros.internal.message.Message;

public interface Person extends Message {
   String _TYPE = "people_msgs/Person";
   String _DEFINITION = "string              name\ngeometry_msgs/Point position\ngeometry_msgs/Point velocity\nfloat64             reliability\nstring[]            tagnames\nstring[]            tags\n\n";

   String getName();

   void setName(String var1);

   Point getPosition();

   void setPosition(Point var1);

   Point getVelocity();

   void setVelocity(Point var1);

   double getReliability();

   void setReliability(double var1);

   List<String> getTagnames();

   void setTagnames(List<String> var1);

   List<String> getTags();

   void setTags(List<String> var1);
}