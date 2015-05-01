package us.ihmc.ihmcPerception.linemod;

public class LineModDetection implements Comparable<LineModDetection>
{
     /** \brief x-position of the detection. */
     int x;
     /** \brief y-position of the detection. */
     int y;
     /** \brief ID of the detected template. */
     int template_id;
     /** \brief score of the detection. */
     float score;
     /** \brief scale at which the template was detected. */
     float scale;
     
     LineModTemplate template;
     
     public float getScaledWidth()
     {
        return template.region.width*scale;
     }

     public float getScaledHeight()
     {
        return template.region.height*scale;
     }
     
     public String toString()
     {
        return "x " + x + " y " + y + " template id " + template_id + " score " + score + " scale " + scale;
     }

   @Override
   public int compareTo(LineModDetection o)
   {
      return Float.compare(score, o.score);
   }
}
